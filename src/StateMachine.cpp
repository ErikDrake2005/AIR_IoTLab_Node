#include "StateMachine.h"
#include "CRC32.h"
#include "esp_sleep.h"

// --- TIMING CONFIG (ms) ---
const unsigned long TIME_PREPARE   = 10000;   
const unsigned long TIME_MEASURE_1 = 180000;  
const unsigned long TIME_MEASURE_2 = 480000;  
const unsigned long TIME_MEASURE_3 = 900000;  
const unsigned long TIMEOUT_CYCLE  = 960000;  
const unsigned long DEFAULT_MANUAL_INT = 300000; 
const unsigned long UART_TIMEOUT = 15000;     

StateMachine::StateMachine(Measurement& meas, RelayController& relay, UARTCommander& cmd, TimeSync& timeSync)
    : _meas(meas), _relay(relay), _cmd(cmd), _timeSync(timeSync) 
{
    _mode = MANUAL; 
    _cycleState = STATE_IDLE;
    _measuresPerDay = 4;
    _currentManualInterval = DEFAULT_MANUAL_INT;
    _lastTriggerMinute = -1;
    _measureCount = 0;
    _stopRequested = false;
    _isDoorOpen = true; 
    _isFanOn = false;   
    _isUartWakeupActive = false;
    _uartWakeupMillis = 0;
    _cycleStartMillis = 0;
    _statusChanged = true;  // Always send initial status
    _lastTimeSyncRequest = 0;
    _nextTimeSyncTime = 0;
    _waitingForTimeSync = false;
    for(int i=0; i<3; i++) _miniData[i] = {-1,-1,-1,-1,-1,-1,-1};
}

void StateMachine::begin() {
    #ifdef PIN_SLEEP_STATUS
    pinMode(PIN_SLEEP_STATUS, OUTPUT);
    _setBusyPin(true); 
    #endif
    
    #ifdef PIN_WAKEUP_GPIO
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKEUP_GPIO, 1);
    #endif
    
    _calculateGridInterval();
    _stopAndResetCycle(); 
    _updateMachineStatus();
    _sendMachineStatus();
    
    delay(50);
    // Request time sync from Gateway immediately
    _sendTimeSyncRequest();
}

void StateMachine::update() {
    // Check if time sync needed (every 1 hour)
    _checkAndRequestTimeSync();
    
    if (_stopRequested) {
        _stopAndResetCycle();
        _stopRequested = false;
        return;
    }

    // Logic Cycle
    if (_cycleState != STATE_IDLE && _cycleState != STATE_MANUAL_LOOP) {
        _processCycleLogic();
        if (_mode == AUTO && (_cycleState >= STATE_WAIT_MEASURE_1 && _cycleState <= STATE_WAIT_MEASURE_3)) {
            _tryLightSleep();
        }
        return;
    }

    // Logic Manual Loop
    if (_cycleState == STATE_MANUAL_LOOP && _mode == MANUAL) {
        if (millis() >= _nextManualMeasureMillis) _startCycle();
        return;
    }

    // Logic Auto Idle
    if (_mode == AUTO) {
        if (_isUartWakeupActive) {
            if (millis() - _uartWakeupMillis > UART_TIMEOUT) _isUartWakeupActive = false;
            else return; 
        }

        unsigned long now = _timeSync.getCurrentTime();
        if (now > 100000) { 
            if (_isTimeForScheduledMeasure() || _isTimeForGridMeasure()) {
                _startCycle();
                return;
            }
        }
        _tryLightSleep();
    }
}

// --- COMMAND HANDLING (Priority Logic) ---
void StateMachine::handleCommand(String jsonStr) {
    if (_isUartWakeupActive) _uartWakeupMillis = millis();

    // Parse command using CommandProcessor with priority extraction
    CommandData cmd = CommandProcessor::parseCommand(jsonStr);

    Serial.println("[CMD] EXEC: " + jsonStr);

    // Process all commands in strict priority order
    _processCommandByPriority(cmd);
    
    // Send status update if anything changed
    if (_statusChanged) {
        _updateMachineStatus();
        _sendMachineStatus();
        _statusChanged = false;
    }
}

void StateMachine::setStopFlag(bool flag) { _stopRequested = flag; }

// --- PRIORITY-BASED COMMAND PROCESSING ---
void StateMachine::_processCommandByPriority(const CommandData& cmd) {
    // Priority 0: EN Command (Sleep/Wake) - HIGHEST PRIORITY - ALWAYS PROCESS FIRST
    if (cmd.hasEnSleep) {
        _handleEnCommand(cmd.enValue);
        return;  // CRITICAL: Stop processing other commands after EN, EN takes absolute precedence
    }
    
    // Priority 1: Time Sync from Gateway
    if (cmd.hasTimeSync) {
        _handleTimeSync(cmd.epochTime);
        _statusChanged = true;
    }
    
    // Priority 2: Mode Change
    if (cmd.hasMode) {
        _handleModeCommand(cmd.mode);
        _statusChanged = true;
    }
    
    // Priority 3: Time Functions (cycle_manual, measures_per_day, schedules)
    if (cmd.hasCycleManual || cmd.hasMeasuresPerDay || cmd.hasSchedules) {
        _handleTimeFunctions(cmd);
        _statusChanged = true;
    }
    
    // Priority 4: Measurement Control (only in MANUAL mode)
    if (cmd.hasMeasurement) {
        if (_mode == MANUAL) {
            _handleMeasurementCommand(cmd.measureCmd);
            _statusChanged = true;
        } else {
            Serial.println("[CMD] Measurement cmd ignored in AUTO mode");
        }
    }
    
    // Priority 5: Relay Control (door/fan) (only in MANUAL mode)
    if (cmd.hasDoor || cmd.hasFan) {
        if (_mode == MANUAL) {
            _handleRelayControl(cmd);
            _statusChanged = true;
        } else {
            Serial.println("[CMD] Relay control ignored in AUTO mode");
        }
    }
}

void StateMachine::_handleEnCommand(int enValue) {
    if (enValue == 0) {
        // EN=0: Deep sleep command from Gateway - HIGHEST PRIORITY, skip other commands
        Serial.println("[CMD] EN:0 received - initiating deep sleep sequence");
        _stopAndResetCycle();
        _handleDeepSleepSequence();
        // Note: esp_deep_sleep_start() never returns
    } else if (enValue == 1) {
        // EN=1: Wake up / continue operation
        Serial.println("[CMD] EN:1 Wake up command received");
        _isUartWakeupActive = true;
        _uartWakeupMillis = millis();
        _statusChanged = true;
    }
}

void StateMachine::_handleTimeSync(unsigned long epochTime) {
    Serial.printf("[CMD] Time sync from Gateway: %lu\n", epochTime);
    _timeSync.updateEpoch(epochTime);
    _lastTimeSyncRequest = millis();
    _waitingForTimeSync = false;
    _nextTimeSyncTime = millis() + (TIME_SYNC_INTERVAL_SECONDS * 1000UL);
}

void StateMachine::_handleModeCommand(const String& mode) {
    if (mode == "auto" || mode == "AUTO") {
        if (_mode != AUTO) {
            Serial.println("[CMD] Mode changed: AUTO");
            _mode = AUTO;
            _lastTriggerMinute = -1;
            _stopAndResetCycle();
        }
    } else if (mode == "manual" || mode == "MANUAL") {
        if (_mode != MANUAL) {
            Serial.println("[CMD] Mode changed: MANUAL");
            _mode = MANUAL;
            _stopAndResetCycle();
        }
    }
}

void StateMachine::_handleTimeFunctions(const CommandData& cmd) {
    // Handle cycle_manual (in minutes)
    if (cmd.hasCycleManual && cmd.cycleManualMin > 0) {
        _currentManualInterval = cmd.cycleManualMin * 60000UL;
        Serial.printf("[CMD] Manual cycle set to %d minutes\n", cmd.cycleManualMin);
    }
    
    // Handle measures_per_day
    if (cmd.hasMeasuresPerDay && cmd.measuresPerDay > 0) {
        _measuresPerDay = cmd.measuresPerDay;
        _calculateGridInterval();
        Serial.printf("[CMD] Measures per day set to %d\n", cmd.measuresPerDay);
    }
    
    // Handle schedules (time grid)
    if (cmd.hasSchedules && cmd.schedulesArray.size() > 0) {
        _schedules.clear();
        for (JsonObject item : cmd.schedulesArray) {
            if (item["hour"].is<int>() && item["minute"].is<int>()) {
                Schedule s;
                s.hour = item["hour"].as<int>();
                s.minute = item["minute"].as<int>();
                s.second = item["second"].is<int>() ? item["second"].as<int>() : 0;
                _schedules.push_back(s);
                Serial.printf("[CMD] Schedule added: %02d:%02d:%02d\n", s.hour, s.minute, s.second);
            }
        }
    }
}

void StateMachine::_handleMeasurementCommand(const String& cmd) {
    if (cmd == "start") {
        Serial.println("[CMD] Measurement start (MANUAL mode)");
        _startCycle();
    } else if (cmd == "stop") {
        Serial.println("[CMD] Measurement stop");
        _stopRequested = true;
    }
}

void StateMachine::_handleRelayControl(const CommandData& cmd) {
    if (cmd.hasDoor) {
        if (cmd.doorCmd == "open" || cmd.doorCmd == "OPEN") {
            Serial.println("[CMD] Door: OPEN");
            _ctrlDoor(true);
        } else if (cmd.doorCmd == "close" || cmd.doorCmd == "CLOSE") {
            Serial.println("[CMD] Door: CLOSE");
            _ctrlDoor(false);
        }
    }
    
    if (cmd.hasFan) {
        if (cmd.fanCmd == "on" || cmd.fanCmd == "ON") {
            Serial.println("[CMD] Fan: ON");
            _ctrlFan(true);
        } else if (cmd.fanCmd == "off" || cmd.fanCmd == "OFF") {
            Serial.println("[CMD] Fan: OFF");
            _ctrlFan(false);
        }
    }
}

// --- CYCLE LOGIC ---
void StateMachine::_startCycle() {
    _cycleState = STATE_PREPARING;
    _cycleStartMillis = millis();
    _measureCount = 0;
    _ctrlDoor(false); 
    _ctrlFan(true);
    _statusChanged = true;
    _updateMachineStatus();
    _sendMachineStatus();
    for(int i=0;i<3;i++) _miniData[i] = {-1,-1,-1,-1,-1,-1,-1};
}

void StateMachine::_processCycleLogic() {
    unsigned long el = millis() - _cycleStartMillis;
    if (el > TIMEOUT_CYCLE) { _finishCycle(); return; }

    switch (_cycleState) {
        case STATE_PREPARING: 
            if (el >= TIME_PREPARE) _cycleState = STATE_WAIT_MEASURE_1; 
            break;
        case STATE_WAIT_MEASURE_1: 
            if (el >= TIME_MEASURE_1) { 
                _setBusyPin(true); _meas.doFullMeasurement(_miniData[0]); 
                _cycleState = STATE_WAIT_MEASURE_2; 
            } break;
        case STATE_WAIT_MEASURE_2: 
            if (el >= TIME_MEASURE_2) { 
                _setBusyPin(true); _meas.doFullMeasurement(_miniData[1]); 
                _cycleState = STATE_WAIT_MEASURE_3; 
            } break;
        case STATE_WAIT_MEASURE_3: 
            if (el >= TIME_MEASURE_3) { 
                _setBusyPin(true); _meas.doFullMeasurement(_miniData[2]); 
                _cycleState = STATE_FINISHING; 
            } break;
        case STATE_FINISHING: _finishCycle(); break;
        default: break;
    }
}

void StateMachine::_finishCycle() {
    // Average the 3 measurements
    float s[7]={0}; 
    int c[7]={0};
    for(int i=0; i<3; i++) {
        float v[]={_miniData[i].ch4, _miniData[i].co, _miniData[i].alc, _miniData[i].nh3, _miniData[i].h2, _miniData[i].temp, _miniData[i].hum};
        for(int k=0; k<7; k++) if(v[k] > -90) { s[k]+=v[k]; c[k]++; }
    }
    float avg[7]; 
    for(int k=0;k<7;k++) avg[k] = (c[k]>0) ? (s[k]/c[k]) : -1.0;
    
    // Send data message with type "data"
    JsonDocument d;
    d["type"] = RESP_TYPE_DATA;
    d["device_id"] = DEVICE_ID;
    d["timestamp"] = _timeSync.getCurrentTime();
    d["ch4"] = avg[0]; 
    d["co"] = avg[1]; 
    d["alc"] = avg[2];
    d["nh3"] = avg[3]; 
    d["h2"] = avg[4]; 
    d["temp"] = avg[5]; 
    d["hum"] = avg[6];
    
    String out; 
    serializeJson(d, out);
    _sendResponse(out);
    Serial.printf("[DATA] Measurement sent: temp=%.2f°C, hum=%.2f%%\n", avg[5], avg[6]);
    
    _ctrlFan(false);
    _ctrlDoor(true);
    
    if (_mode == MANUAL) {
        _cycleState = STATE_MANUAL_LOOP; 
        _nextManualMeasureMillis = millis() + _currentManualInterval;
    } else {
        _stopAndResetCycle();
    }
    
    _statusChanged = true;
    _updateMachineStatus();
    _sendMachineStatus();
}

void StateMachine::_stopAndResetCycle() {
    _cycleState = STATE_IDLE; 
    _ctrlFan(false); 
    _ctrlDoor(true); 
    _setBusyPin(true);
    _statusChanged = true;
}

// --- SLEEP & UTILS ---
void StateMachine::_handleDeepSleepSequence() {
    _ctrlDoor(false); _ctrlFan(false);  
    delay(100); 
    _sendMachineStatus(); // Final status before sleep
    delay(500); 
    _setBusyPin(false); // Signal Bridge: I am sleeping
    esp_deep_sleep_start();
}

void StateMachine::_tryLightSleep() {
    unsigned long sleepT = 0;
    unsigned long el = millis() - _cycleStartMillis;
    
    if (_cycleState == STATE_IDLE) sleepT = _calculateNextWakeTime();
    else if (_cycleState == STATE_WAIT_MEASURE_1) sleepT = TIME_MEASURE_1 - el;
    else if (_cycleState == STATE_WAIT_MEASURE_2) sleepT = TIME_MEASURE_2 - el;
    else if (_cycleState == STATE_WAIT_MEASURE_3) sleepT = TIME_MEASURE_3 - el;

    if (sleepT > 5000) {
        esp_sleep_enable_timer_wakeup((sleepT - 2000) * 1000ULL); 
        esp_sleep_enable_uart_wakeup(0); 
        _setBusyPin(false); 
        esp_light_sleep_start();
        _setBusyPin(true); 
        if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UART) {
            while(Serial.available()) Serial.read(); 
            _sendMachineStatus();
            _isUartWakeupActive = true; 
            _uartWakeupMillis = millis();
        }
    }
}

unsigned long StateMachine::_calculateNextWakeTime() {
    unsigned long now = _timeSync.getCurrentTime();
    if (now < 100000) return 0;
    unsigned long minW = 2147483647; bool f = false;
    
    if (_gridIntervalSeconds > 0) {
        unsigned long nxt = ((now / _gridIntervalSeconds) + 1) * _gridIntervalSeconds;
        long w = nxt - now;
        if (w > 0 && w < minW) { minW = w; f = true; }
    }
    long secToday = now % 86400;
    for (auto& s : _schedules) {
        long schS = s.hour * 3600 + s.minute * 60;
        long w = (schS > secToday) ? (schS - secToday) : ((86400 - secToday) + schS);
        if (w > 0 && w < minW) { minW = w; f = true; }
    }
    return f ? (minW * 1000UL) : 0;
}

void StateMachine::_ctrlDoor(bool open) {
    if (open) { _relay.ON_DOOR(); _isDoorOpen = true; }
    else { _relay.OFF_DOOR(); _isDoorOpen = false; }
}

void StateMachine::_ctrlFan(bool on) {
    if (on) { _relay.ON_FAN(); _isFanOn = true; }
    else { _relay.OFF_FAN(); _isFanOn = false; }
}

void StateMachine::_updateMachineStatus() {
    // Update machine status struct with current state
    _lastStatus.type = RESP_TYPE_MACHINE_STATUS;
    _lastStatus.deviceId = DEVICE_ID;
    _lastStatus.door = _isDoorOpen ? "open" : "close";
    _lastStatus.fan = _isFanOn ? "on" : "off";
    _lastStatus.mode = (_mode == AUTO) ? "auto" : "manual";
    _lastStatus.measuring = (_cycleState != STATE_IDLE && _cycleState != STATE_MANUAL_LOOP) ? "YES" : "NO";
    _lastStatus.cycleManual = String(_currentManualInterval / 60000);  // Convert ms to minutes
    _lastStatus.measuresPerDay = String(_measuresPerDay);
    _lastStatus.timestamp = _timeSync.getCurrentTime();
}

void StateMachine::_sendMachineStatus() {
    // Always send machine_status when called
    JsonDocument doc;
    doc["type"] = RESP_TYPE_MACHINE_STATUS;
    doc["device_id"] = DEVICE_ID;
    doc["door"] = _isDoorOpen ? "open" : "close";
    doc["fan"] = _isFanOn ? "on" : "off";
    doc["mode"] = (_mode == AUTO) ? "auto" : "manual";
    doc["measuring"] = (_cycleState != STATE_IDLE && _cycleState != STATE_MANUAL_LOOP) ? "YES" : "NO";
    doc["cycle_manual"] = _currentManualInterval / 60000;  // Convert to minutes
    doc["measures_per_day"] = _measuresPerDay;
    doc["timestamp"] = _timeSync.getCurrentTime();
    
    String output; 
    serializeJson(doc, output);
    _sendResponse(output);
    Serial.printf("[STATUS] door=%s fan=%s mode=%s measuring=%s\n", 
        doc["door"].as<const char*>(), 
        doc["fan"].as<const char*>(),
        doc["mode"].as<const char*>(),
        doc["measuring"].as<const char*>());
}

void StateMachine::_sendResponse(String json) {
    unsigned long crc = CRC32::calculate(json);
    _cmd.pushToQueue(json + "|" + String(crc, HEX));
}

void StateMachine::_setBusyPin(bool isBusy) { 
    #ifdef PIN_SLEEP_STATUS
    digitalWrite(PIN_SLEEP_STATUS, isBusy ? HIGH : LOW); 
    #endif
}

void StateMachine::_calculateGridInterval() {
    if (_measuresPerDay <= 0) _measuresPerDay = 1;
    _gridIntervalSeconds = 86400 / _measuresPerDay;
}

void StateMachine::_parseSetTime(String tStr) {
    int h, m; 
    if (sscanf(tStr.c_str(), "%d:%d", &h, &m)==2) {
        for (auto& s : _schedules) if (s.hour==h && s.minute==m) return;
        if (_schedules.size()>=10) _schedules.erase(_schedules.begin());
        _schedules.push_back({h, m, 0});
    }
}

bool StateMachine::_isTimeForGridMeasure() {
    unsigned long now = _timeSync.getCurrentTime();
    return (now > 100000) && ((now % _gridIntervalSeconds) < 5);
}

bool StateMachine::_isTimeForScheduledMeasure() {
    unsigned long now = _timeSync.getCurrentTime();
    time_t r = (time_t)now; struct tm* t = localtime(&r);
    if (t->tm_min == _lastTriggerMinute) return false;
    for (auto& s : _schedules) {
        if (t->tm_hour==s.hour && t->tm_min==s.minute) { _lastTriggerMinute = t->tm_min; return true; }
    }
    return false;
}

// --- AUTO TIME SYNC MECHANISM ---
void StateMachine::_checkAndRequestTimeSync() {
    // Every TIME_SYNC_INTERVAL_SECONDS (3600s = 1 hour), request time sync
    unsigned long currentMillis = millis();
    
    if (_nextTimeSyncTime == 0) {
        // Initialize first sync time
        _nextTimeSyncTime = currentMillis + (TIME_SYNC_INTERVAL_SECONDS * 1000UL);
        return;
    }
    
    if (currentMillis >= _nextTimeSyncTime) {
        // Time to request sync
        _sendTimeSyncRequest();
        _nextTimeSyncTime = currentMillis + (TIME_SYNC_INTERVAL_SECONDS * 1000UL);
    }
}

void StateMachine::_sendTimeSyncRequest() {
    // Send time_req to Gateway for time synchronization
    JsonDocument doc;
    doc["type"] = "time_req";
    doc["device_id"] = DEVICE_ID;
    doc["current_time"] = _timeSync.getCurrentTime();
    
    String output;
    serializeJson(doc, output);
    _sendResponse(output);
    
    Serial.printf("[TIME] Requesting time sync from Gateway\n");
    _waitingForTimeSync = true;
    _lastTimeSyncRequest = millis();
}