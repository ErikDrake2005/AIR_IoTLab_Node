#include "StateMachine.h"
#include "esp_sleep.h"
#include "CRC32.h"
#include "Config.h"

const unsigned long T_PREPARE   = 10000;
const unsigned long T_MEASURE_1 = 180000;
const unsigned long T_MEASURE_2 = 480000;
const unsigned long T_MEASURE_3 = 900000;
const unsigned long T_TIMEOUT   = 960000;
const unsigned long T_SYNC      = 3600000;
const unsigned long T_UART_WAIT = 15000;

StateMachine::StateMachine(Measurement& meas, RelayController& relay, UARTCommander& cmd, TimeSync& timeSync)
    : _meas(meas), _relay(relay), _cmd(cmd), _timeSync(timeSync) 
{
    _status.mode = MODE_MANUAL;
    _status.isMeasuring = false;
    _status.isDoorOpen = true;
    _status.isFanOn = false;
    _status.saved_manual_cycle = 5; 
    _status.saved_daily_measures = 4;
    
    _cycleState = STATE_IDLE;
    _gridInterval = 0;
    _startTimeSeconds = 0;
    _nextTimeSync = 0;
    _stopRequested = false;
    _isUartWakeup = false;
    _uartWakeupMs = 0;
    _nextManualRun = 0;
    _lastCycleStartSeconds = 0;
    
    _resetMiniData();
}

void StateMachine::begin() {
    #ifdef PIN_SLEEP_STATUS
    pinMode(PIN_SLEEP_STATUS, OUTPUT);
    digitalWrite(PIN_SLEEP_STATUS, HIGH);
    #endif
    
    #ifdef PIN_WAKEUP_GPIO
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKEUP_GPIO, 1);
    #endif

    _recalcGrid();
    
    Serial.println("[BOOT] Resetting to safe state...");
    _resetCycle();
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    Serial.println("[BOOT] Sending initial machine_status...");
    _sendMachineStatus();
    
    Serial.println("[BOOT] Requesting time sync...");
    _requestTimeSync();
}

void StateMachine::update() {
    if (millis() > _nextTimeSync && _nextTimeSync != 0) {
        _requestTimeSync();
    }

    if (_stopRequested) {
        Serial.println("[STOP] Forced Stop Requested!");
        _resetCycle();
        _sendMachineStatus();
        _stopRequested = false;
        return;
    }

    if (_status.mode == MODE_AUTO && _cycleState != STATE_IDLE) {
        _processAutoCycle();
        return;
    }

    unsigned long now = millis();

    if (_status.mode == MODE_MANUAL) {
        if (_status.isMeasuring) {
            if (_status.saved_manual_cycle > 0 && now >= _nextManualRun) {
                _performManualMeasurement(); 
                _nextManualRun = now + (_status.saved_manual_cycle * 60000UL);
            }
        }
    }
    else if (_status.mode == MODE_AUTO) {
        if (_isUartWakeup && now - _uartWakeupMs > T_UART_WAIT) {
            _isUartWakeup = false;
        }
        
        if ((_startTimeSeconds > 0 && _checkSchedule()) || (_checkGrid())) {
            _startAutoCycle();
            return;
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void StateMachine::processRawCommand(String rawLine) {
    CommandData cmd = _cmdProcessor.parse(rawLine);
    
    if (!cmd.isValid) {
        Serial.println("[CMD] Invalid command received, ignoring...");
        return;
    }

    bool hasChanges = false;

    if (cmd.setMode == MODE_SLEEP) {
        Serial.println("[CMD] SLEEP command received");
        _sendMachineStatus();
        _enterDeepSleep(); 
        return; 
    }

    if (cmd.setMode == MODE_TIMESTAMP) {
        if (cmd.timestamp > 0) {
            Serial.printf("[CMD] Time sync: %lu\n", cmd.timestamp);
            _timeSync.updateEpoch(cmd.timestamp);
            _recalcGrid();
            hasChanges = true;
        }
        _sendMachineStatus();
        return;
    }

    if (cmd.setMode == MODE_AUTO) {
        if (_status.mode != MODE_AUTO) {
            Serial.println("[MODE] Switched to AUTO");
            _status.mode = MODE_AUTO;
            _resetCycle(); 
            hasChanges = true;
        }
        if (cmd.autoMeasureCount > 0) {
            _status.saved_daily_measures = cmd.autoMeasureCount;
            _recalcGrid();
            hasChanges = true;
        }
        if (cmd.startTimeSeconds > 0) {
            _startTimeSeconds = cmd.startTimeSeconds;
            hasChanges = true;
        }
    }
    else if (cmd.setMode == MODE_MANUAL) {
        if (_status.mode != MODE_MANUAL) {
            Serial.println("[MODE] Switched to MANUAL");
            if (_cycleState != STATE_IDLE) {
                _stopRequested = true;
            }
            _status.mode = MODE_MANUAL;
            _resetCycle();
            hasChanges = true;
        }
        if (cmd.manualInterval > 0) {
            _status.saved_manual_cycle = cmd.manualInterval;
            _nextManualRun = millis() + (_status.saved_manual_cycle * 60000UL);
            hasChanges = true;
        }
    }

    if (cmd.hasActions) {
        if (!cmd.chamberStatus.isEmpty()) {
            if (cmd.chamberStatus == "stop" || cmd.chamberStatus == "stop-measurement") {
                Serial.println("[CMD] Stop measurement requested");
                _resetCycle();
                Serial.println("[CMD] Measurement stopped, sending ACK...");
                _sendMachineStatus();
                return;
            } else if (cmd.chamberStatus == "start" || cmd.chamberStatus == "start-measurement") {
                Serial.println("[CMD] Start measurement requested");
                if (_status.mode == MODE_MANUAL) {
                    _performManualMeasurement();
                    return;
                }
                hasChanges = true;
            }
        }
        else {
            if (!cmd.doorStatus.isEmpty()) {
                if (cmd.doorStatus == "open") {
                    _setDoor(true);
                    hasChanges = true;
                } else if (cmd.doorStatus == "close") {
                    _setDoor(false);
                    hasChanges = true;
                }
            }

            if (!cmd.fanStatus.isEmpty()) {
                if (cmd.fanStatus == "on") {
                    _setFan(true);
                    hasChanges = true;
                } else if (cmd.fanStatus == "off") {
                    _setFan(false);
                    hasChanges = true;
                }
            }
        }
    }
    
    Serial.printf("[CMD] Command processed, hasChanges=%d, sending machine_status\n", hasChanges);
    _sendMachineStatus();
}

void StateMachine::_startAutoCycle() {
    _status.isMeasuring = true;
    _cycleState = STATE_PREPARE;
    _cycleStartMs = millis();
    
    _lastCycleStartSeconds = _timeSync.getCurrentTime();
    Serial.printf("[AUTO] Cycle started at epoch: %lu\n", _lastCycleStartSeconds);
    
    _setDoor(false);
    _setFan(true);
    
    _sendMachineStatus();
    _resetMiniData();
}

void StateMachine::_processAutoCycle() {
    unsigned long elapsed = millis() - _cycleStartMs;
    
    if (elapsed > T_TIMEOUT) { _finishAutoCycle(true); return; }

    switch (_cycleState) {
        case STATE_PREPARE:
            if (elapsed >= T_PREPARE) _cycleState = STATE_WAIT_1;
            break;
        case STATE_WAIT_1:
            if (elapsed >= T_MEASURE_1) {
                MeasurementData sample;
                _meas.doFullMeasurement(sample);
                _sendDataPacket(sample, 1);
                _cycleState = STATE_WAIT_2;
            } break;
        case STATE_WAIT_2:
            if (elapsed >= T_MEASURE_2) {
                MeasurementData sample;
                _meas.doFullMeasurement(sample);
                _sendDataPacket(sample, 2);
                _cycleState = STATE_WAIT_3;
            } break;
        case STATE_WAIT_3:
            if (elapsed >= T_MEASURE_3) {
                MeasurementData sample;
                _meas.doFullMeasurement(sample);
                _sendDataPacket(sample, 3);
                _finishAutoCycle(false); 
            } break;
    }
}

void StateMachine::_sendDataPacket(const MeasurementData& data, int sampleNum) {
    String jsonPkt = _jsonFormatter.createDataJson(
        data.ch4, data.co, data.alc, data.nh3, data.h2, data.temp, data.hum
    );
    _sendPacket(jsonPkt);
    Serial.printf("[AUTO] Sample %d sent.\n", sampleNum);
}

void StateMachine::_finishAutoCycle(bool aborted) {
    if (aborted) {
        _resetCycle();
        return;
    }
    
    Serial.println("[AUTO] Mini-cycle finished (3 samples sent).");
    _resetCycle(); 
    _sendMachineStatus();
}

void StateMachine::_performManualMeasurement() {
    if (!_status.isMeasuring) {
        _status.isMeasuring = true; 
        _setDoor(false);
        _setFan(true);
        
        Serial.println("[MANUAL] Status changed, sending machine_status before measure...");
        _sendMachineStatus();
        
        Serial.println("[MANUAL] Waiting for gas stabilization...");
        unsigned long prepareEnd = millis() + T_PREPARE;
        while (millis() < prepareEnd) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            _checkUrgentCommands();
            
            if (_stopRequested) {
                Serial.println("[MANUAL] Aborted during preparation!");
                _resetCycle();
                _sendMachineStatus();
                return;
            }
        }
    }
    
    if (_stopRequested) {
        _resetCycle();
        _sendMachineStatus();
        return;
    }
    
    MeasurementData singleSample;
    auto urgentCheck = [this]() { this->_checkUrgentCommands(); };
    
    if (_meas.doFullMeasurement(singleSample, &_stopRequested, urgentCheck)) {
        if (_stopRequested) {
            Serial.println("[MANUAL] Aborted after measurement!");
            _resetCycle();
            _sendMachineStatus();
            return;
        }
        
        String jsonPkt = _jsonFormatter.createDataJson(
            singleSample.ch4, singleSample.co, singleSample.alc, 
            singleSample.nh3, singleSample.h2, singleSample.temp, singleSample.hum
        );
        _sendPacket(jsonPkt);
    }
}

void StateMachine::_resetCycle() {
    _cycleState = STATE_IDLE;
    _status.isMeasuring = false;
    
    _setFan(false);
    _setDoor(true); 
    
    _lastCycleStartSeconds = 0;
    Serial.println("[CYCLE] Reset complete, cooldown cleared.");
    
    _nextManualRun = millis() + (_status.saved_manual_cycle * 60000UL);
}

void StateMachine::_resetMiniData() {
    for(int i=0; i<3; i++) {
        _miniData[i].ch4 = -1.0f; _miniData[i].co = -1.0f;
        _miniData[i].alc = -1.0f; _miniData[i].nh3 = -1.0f;
        _miniData[i].h2 = -1.0f;  _miniData[i].temp = -1.0f;
        _miniData[i].hum = -1.0f;
    }
}

void StateMachine::_setDoor(bool open) {
    if(open) { 
        _relay.ON_DOOR();
        _status.isDoorOpen = true; 
    }
    else { 
        _relay.OFF_DOOR();
        _status.isDoorOpen = false; 
    }
}

void StateMachine::_setFan(bool on) {
    if(on) { 
        _relay.ON_FAN();
        _status.isFanOn = true; 
    }
    else { 
        _relay.OFF_FAN();
        _status.isFanOn = false; 
    }
}

void StateMachine::_enterDeepSleep() {
    Serial.println("[PWR] Entering Deep Sleep...");
    _resetCycle();
    _sendPacket(_jsonFormatter.createAck("SLEEP"));
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    _timeSync.beforeDeepSleep();
    
    #ifdef PIN_SLEEP_STATUS
    digitalWrite(PIN_SLEEP_STATUS, LOW); 
    #endif
    
    esp_deep_sleep_start();
}

void StateMachine::_handleLightSleep() {
    unsigned long sleepTime = _calcSleepTime();
    if (sleepTime < 5000) return; 

    Serial.printf("[PWR] Light Sleep for %lu ms\n", sleepTime);
    
    esp_sleep_enable_timer_wakeup((sleepTime - 2000) * 1000ULL); 
    esp_sleep_enable_uart_wakeup(UART_NUM_1); 
    
    #ifdef PIN_SLEEP_STATUS
    digitalWrite(PIN_SLEEP_STATUS, LOW);
    #endif
    
    esp_light_sleep_start();
    
    #ifdef PIN_SLEEP_STATUS
    digitalWrite(PIN_SLEEP_STATUS, HIGH);
    #endif
    
    _timeSync.afterWakeup(); 
    
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UART) {
        while(Serial1.available()) Serial1.read();
        _isUartWakeup = true;
        _uartWakeupMs = millis();
    }
}

unsigned long StateMachine::_calcSleepTime() {
    unsigned long now = _timeSync.getCurrentTime();
    unsigned long elapsed = millis() - _cycleStartMs;

    if (_cycleState != STATE_IDLE) {
        if (_cycleState == STATE_PREPARE) return (T_MEASURE_1 > elapsed) ? (T_MEASURE_1 - elapsed) : 0;
        if (_cycleState == STATE_WAIT_1)  return (T_MEASURE_2 > elapsed) ? (T_MEASURE_2 - elapsed) : 0;
        if (_cycleState == STATE_WAIT_2)  return (T_MEASURE_3 > elapsed) ? (T_MEASURE_3 - elapsed) : 0;
        return 0;
    }
    if (now < 100000) return 0;
    
    long minDiff = 86400;
    
    if (_gridInterval > 0) {
        long nextGrid = ((now / _gridInterval) + 1) * _gridInterval;
        long diff = nextGrid - now;
        if (diff < minDiff) minDiff = diff;
    }

    if (_startTimeSeconds > 0) {
        unsigned long secondsInDay = now % 86400;
        long diff = 0;
        if (secondsInDay < _startTimeSeconds) {
            diff = _startTimeSeconds - secondsInDay;
        } else {
            diff = (86400 - secondsInDay) + _startTimeSeconds;
        }
        if (diff < minDiff) minDiff = diff;
    }
    
    return minDiff * 1000UL;
}

void StateMachine::_recalcGrid() {
    if (_status.saved_daily_measures <= 0) _status.saved_daily_measures = 1;
    _gridInterval = 86400 / _status.saved_daily_measures;
}

bool StateMachine::_checkSchedule() {
    unsigned long now = _timeSync.getCurrentTime();
    if (now < 100000 || _gridInterval == 0 || _startTimeSeconds == 0) return false;
    
    if (_lastCycleStartSeconds > 0 && (now - _lastCycleStartSeconds) < CYCLE_COOLDOWN) {
        return false;
    }
    
    unsigned long secondsInDay = now % 86400;
    
    if (secondsInDay >= _startTimeSeconds) {
        unsigned long elapsed = secondsInDay - _startTimeSeconds;
        if (elapsed % _gridInterval < 5) return true;
    } else {
        unsigned long prevDayTime = secondsInDay + 86400 - _startTimeSeconds;
        if (prevDayTime % _gridInterval < 5) return true;
    }
    return false;
}

bool StateMachine::_checkGrid() {
    unsigned long now = _timeSync.getCurrentTime();
    if (now < 100000) return false;
    
    if (_lastCycleStartSeconds > 0 && (now - _lastCycleStartSeconds) < CYCLE_COOLDOWN) {
        return false;
    }
    
    return ((now % _gridInterval) < 5);
}

void StateMachine::_sendMachineStatus() {
    String modeStr = (_status.mode == MODE_AUTO) ? "AUTO" : "MANUAL";
    String stateStr = _status.isMeasuring ? "measuring" : "stop";
    String doorStr = _status.isDoorOpen ? "open" : "close";
    String fanStr = _status.isFanOn ? "on" : "off";
    
    String json = _jsonFormatter.createMachineStatus(
        modeStr, stateStr, doorStr, fanStr, 
        _status.saved_manual_cycle, _status.saved_daily_measures, 0
    );
    _sendPacket(json);
}

void StateMachine::_requestTimeSync() {
    _sendPacket(_jsonFormatter.createTimeSyncRequest());
    _nextTimeSync = millis() + T_SYNC; 
}

void StateMachine::_sendPacket(String json) {
    _cmd.send(json);    
}

void StateMachine::_checkUrgentCommands() {
    if (_cmd.hasCommand()) {
        String rawCmd = _cmd.getCommand();
        Serial.printf("[URGENT] Received command during measurement: %s\n", rawCmd.c_str());
        
        CommandData cmd = _cmdProcessor.parse(rawCmd);
        
        if (cmd.isValid) {
            if (!cmd.chamberStatus.isEmpty() && 
                (cmd.chamberStatus == "stop" || cmd.chamberStatus == "stop-measurement")) {
                Serial.println("[URGENT] STOP command detected!");
                _stopRequested = true;
            }
            else if (cmd.setMode == MODE_SLEEP) {
                Serial.println("[URGENT] SLEEP command detected!");
                _stopRequested = true;
            }
            else {
                bool urgentChanged = false;
                
                if (!cmd.doorStatus.isEmpty()) {
                    if (cmd.doorStatus == "open") _setDoor(true);
                    else if (cmd.doorStatus == "close") _setDoor(false);
                    urgentChanged = true;
                }
                if (!cmd.fanStatus.isEmpty()) {
                    if (cmd.fanStatus == "on") _setFan(true);
                    else if (cmd.fanStatus == "off") _setFan(false);
                    urgentChanged = true;
                }
                
                if (urgentChanged) {
                    _sendMachineStatus();
                }
            }
        }
    }
}
