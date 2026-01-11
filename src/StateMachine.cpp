#include "StateMachine.h"
#include "CRC32.h"
#include "esp_sleep.h"

// --- CẤU HÌNH THỜI GIAN ---
const unsigned long TIME_PREPARE   = 10000;   
const unsigned long TIME_MEASURE_1 = 180000;  
const unsigned long TIME_MEASURE_2 = 480000;  
const unsigned long TIME_MEASURE_3 = 900000;  
const unsigned long TIMEOUT_CYCLE  = 960000;  
const unsigned long DEFAULT_MANUAL_INT = 300000; 
const unsigned long UART_TIMEOUT = 15000;     

StateMachine::StateMachine(Measurement& meas, RelayController& relay, UARTCommander& cmd, TimeSync& timeSync)
    : _meas(meas), _relay(relay), _cmd(cmd), _timeSync(timeSync) {
    _mode = MANUAL; 
    _cycleState = STATE_IDLE;
    _measuresPerDay = DEFAULT_MEASURES_PER_DAY;
    _currentManualInterval = DEFAULT_MANUAL_INT;
    _lastTriggerMinute = -1;
    _measureCount = 0;
    _stopRequested = false;
    _calculateGridInterval(); 
}

void StateMachine::begin() {
    pinMode(PIN_SLEEP_STATUS, OUTPUT);
    _setBusyPin(true); 
    
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKEUP_GPIO, 1);
    
    _stopAndResetCycle(); 
    
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        _sendDetailedAck("WAKEUP_BY_GPIO", "open", "off", "stop");
    }
    
    _sendDetailedAck("SYSTEM_READY", "open", "off", "stop");
    delay(50);
    _sendResponse(_timeSync.getRequestJson()); 
}

void StateMachine::update() {
    if (_stopRequested) {
        _stopAndResetCycle();
        _stopRequested = false;
        // Thông báo đã dừng theo yêu cầu
        _sendDetailedAck("STOPPED_BY_FLAG", "open", "off", "stop");
        return;
    }

    if (_cycleState != STATE_IDLE) {
        _processCycleLogic();
        if (_mode == AUTO && (_cycleState >= STATE_WAIT_MEASURE_1 && _cycleState <= STATE_WAIT_MEASURE_3)) {
            _tryLightSleep();
        }
        return;
    }

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

// --- HÀM GỬI ACK CHI TIẾT (MỚI THÊM) ---
void StateMachine::_sendDetailedAck(const char* cmd, const char* door, const char* fan, const char* state) {
    JsonDocument doc;
    doc["type"] = "ack";
    doc["cmd"] = cmd;
    doc["door"] = door;
    doc["fan"] = fan;
    doc["state"] = state;
    doc["timestamp"] = _timeSync.getCurrentTime(); // Luôn gửi kèm timestamp

    String output;
    serializeJson(doc, output);
    _sendResponse(output);
}

void StateMachine::handleCommand(const String& jsonCmd) {
    processJsonCommand(jsonCmd);
}

void StateMachine::processJsonCommand(const String& jsonStr) {
    if (_isUartWakeupActive) _uartWakeupMillis = millis(); 
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (error) return;

    if (doc["EN"].is<int>() && doc["EN"] == 0) {
        _handleDeepSleepSequence();
        return;
    }
    
    bool cfgChanged = false;
    
    if (doc["timestamp"].is<unsigned long>()) {
        unsigned long ts = doc["timestamp"];
        if (ts > 0) { _timeSync.updateEpoch(ts); _lastTriggerMinute = -1; cfgChanged = true; }
    }
    
    if (doc["measures_per_day"].is<int>()) { 
        _measuresPerDay = doc["measures_per_day"]; 
        _calculateGridInterval(); 
        cfgChanged = true; 
    }
    
    if (doc["set_time"]) { 
        _parseSetTime(doc["set_time"].as<String>()); 
        cfgChanged = true; 
    }

    const char* m = doc["mode"];
    if (m) {
        if (strcmp(m, "manual")==0) { 
            _mode = MANUAL; 
            _stopAndResetCycle(); 
            _sendDetailedAck("MODE_MANUAL", "open", "off", "stop");
        }
        else if (strcmp(m, "auto")==0) { 
            _mode = AUTO; 
            _stopAndResetCycle(); 
            _lastTriggerMinute = -1; 
            _sendDetailedAck("MODE_AUTO", "open", "off", "stop");
        }
    }

    const char* cmd = doc["cmd"];
    const char* st = doc["set_state"];
    bool isStop = (st && strcmp(st,"stop")==0) || (cmd && strcmp(cmd,"stop_measure")==0);
    bool isTrig = (st && strcmp(st,"measure")==0) || (cmd && strcmp(cmd,"trigger_measure")==0);

    if (isStop) { 
        setStopFlag(true); 
    }
    else if (isTrig) {
        if (_cycleState != STATE_IDLE && _cycleState != STATE_MANUAL_LOOP) {
             _sendResponse(_json.createError("BUSY")); // Hoặc dùng hàm _sendDetailedAck gửi Error
        } else { 
            _mode = MANUAL; 
            _startManualLoop(); 
            // Ack khi bắt đầu đo thủ công
            _sendDetailedAck("MEASURE_STARTED", "close", "on", "measure");
        }
    }

    if (_mode == MANUAL) {
        if (doc["set_door"]) _handleDirectCommand(doc["set_door"]);
        if (doc["set_fans"]) _handleDirectCommand(doc["set_fans"]);
    }
    
    if (cfgChanged) _sendDetailedAck("CONFIG_OK", "open", "off", "stop");
}

void StateMachine::_handleDirectCommand(const char* cmd) {
    if (strcmp(cmd,"open")==0) { _relay.ON_DOOR(); _sendDetailedAck("DOOR_OPEN", "open", "off", "stop"); }
    else if (strcmp(cmd,"close")==0) { _relay.OFF_DOOR(); _sendDetailedAck("DOOR_CLOSE", "close", "off", "stop"); }
    else if (strcmp(cmd,"on")==0) { _relay.ON_FAN(); _sendDetailedAck("FAN_ON", "open", "on", "stop"); }
    else if (strcmp(cmd,"off")==0) { _relay.OFF_FAN(); _sendDetailedAck("FAN_OFF", "open", "off", "stop"); }
}

void StateMachine::_startCycle() {
    _cycleState = STATE_PREPARING;
    _cycleStartMillis = millis();
    _measureCount = 0;
    _relay.OFF_DOOR(); 
    _relay.ON_FAN();   
    
    // GỬI ACK: Bắt đầu chu trình đo (Đóng cửa, Bật quạt)
    _sendDetailedAck("MEASURE_STARTED", "close", "on", "measure");

    for(int i=0;i<3;i++) _miniData[i] = {-1,-1,-1,-1,-1,-1,-1};
}

void StateMachine::_processCycleLogic() {
    unsigned long el = millis() - _cycleStartMillis;
    if (el > TIMEOUT_CYCLE && _cycleState != STATE_MANUAL_LOOP) { 
        _finishCycle(); return; 
    }

    switch (_cycleState) {
        case STATE_PREPARING:
            if (el >= TIME_PREPARE) _cycleState = STATE_WAIT_MEASURE_1; 
            break;
        case STATE_WAIT_MEASURE_1:
            if (el >= TIME_MEASURE_1) { _setBusyPin(true); _meas.doFullMeasurement(_miniData[0]); _cycleState = STATE_WAIT_MEASURE_2; } break;
        case STATE_WAIT_MEASURE_2:
            if (el >= TIME_MEASURE_2) { _setBusyPin(true); _meas.doFullMeasurement(_miniData[1]); _cycleState = STATE_WAIT_MEASURE_3; } break;
        case STATE_WAIT_MEASURE_3:
            if (el >= TIME_MEASURE_3) { _setBusyPin(true); _meas.doFullMeasurement(_miniData[2]); _cycleState = STATE_FINISHING; } break;
        case STATE_FINISHING:
            _finishCycle(); 
            break;
        case STATE_MANUAL_LOOP: 
            if (millis() >= _nextManualMeasureMillis) {
                MeasurementData t; _meas.doFullMeasurement(t);
                _sendResponse(_json.createDataJson(t.ch4, t.co, t.alc, t.nh3, t.h2, t.temp, t.hum));
                _nextManualMeasureMillis = millis() + _currentManualInterval;
            } break;
        default: break;
    }
}

void StateMachine::_finishCycle() {
    float s[7]={0}; int c[7]={0};
    for(int i=0; i<3; i++) {
        float v[]={_miniData[i].ch4, _miniData[i].co, _miniData[i].alc, _miniData[i].nh3, _miniData[i].h2, _miniData[i].temp, _miniData[i].hum};
        for(int k=0; k<7; k++) if(v[k] > -90) { s[k]+=v[k]; c[k]++; }
    }
    float avg[7]; for(int k=0;k<7;k++) avg[k] = (c[k]>0) ? (s[k]/c[k]) : -1.0;
    
    // Gửi kết quả cuối cùng
    _sendResponse(_json.createDataJson(avg[0], avg[1], avg[2], avg[3], avg[4], avg[5], avg[6]));
    
    // Kết thúc thì Reset trạng thái
    _stopAndResetCycle();
}

void StateMachine::_stopAndResetCycle() {
    _cycleState = STATE_IDLE; 
    _relay.OFF_FAN(); 
    _relay.ON_DOOR(); 
    _setBusyPin(true); 
    _measureCount = 0;

    // GỬI ACK: Ngưng đo (Mở cửa, Tắt quạt)
    _sendDetailedAck("MEASURE_STOPPED", "open", "off", "stop");
}

void StateMachine::_startManualLoop() {
    _cycleState = STATE_MANUAL_LOOP; 
    _relay.OFF_DOOR(); 
    _relay.ON_FAN();
    _nextManualMeasureMillis = millis() + 1000;
}

void StateMachine::setStopFlag(bool flag) { _stopRequested = flag; }

void StateMachine::_handleDeepSleepSequence() {
    // Trước khi ngủ, đảm bảo thiết bị ở trạng thái an toàn
    _relay.OFF_DOOR(); 
    _relay.OFF_FAN(); 
    delay(1000);
    
    // Gửi xác nhận đi ngủ kèm trạng thái cuối
    _sendDetailedAck("EN:0", "close", "off", "stop"); 
    
    delay(500);
    _setBusyPin(false); 
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
        esp_sleep_enable_uart_wakeup(1); 
        _setBusyPin(false); 
        esp_light_sleep_start();
        _setBusyPin(true); 
        
        if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UART) {
            while(Serial1.available()) Serial1.read(); 
            _sendDetailedAck("UART_WAKEUP", "unknown", "unknown", "stop");
            _isUartWakeupActive = true; 
            _uartWakeupMillis = millis();
        }
    }
}

unsigned long StateMachine::_calculateNextWakeTime() {
    unsigned long now = _timeSync.getCurrentTime();
    if (now < 100000) return 0;
    unsigned long minW = 2147483647; 
    bool f = false;
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

void StateMachine::_setBusyPin(bool isBusy) { 
    digitalWrite(PIN_SLEEP_STATUS, isBusy ? HIGH : LOW); 
}

void StateMachine::_sendResponse(String json) {
    unsigned long crc = CRC32::calculate(json);
    _cmd.pushToQueue(json + "|" + String(crc, HEX));
}

void StateMachine::_calculateGridInterval() {
    if (_measuresPerDay <= 0) _measuresPerDay = 1;
    _gridIntervalSeconds = 86400 / _measuresPerDay;
}

void StateMachine::_parseSetTime(String tStr) {
    int h, m; 
    if (sscanf(tStr.c_str(), "%d:%d", &h, &m)==2) {
        if (h<0||h>23||m<0||m>59) return;
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
        if (t->tm_hour==s.hour && t->tm_min==s.minute) { 
            _lastTriggerMinute = t->tm_min; 
            return true; 
        }
    }
    return false;
}