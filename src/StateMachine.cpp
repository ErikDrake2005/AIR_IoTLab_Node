#include "StateMachine.h"
#include "CRC32.h"
#include "esp_sleep.h"

// --- CẤU HÌNH THỜI GIAN ---
const unsigned long TIME_PREPARE   = 10000;   // 10s chuẩn bị (bật quạt)
const unsigned long TIME_MEASURE_1 = 180000;  // 3 phút đo lần 1
const unsigned long TIME_MEASURE_2 = 480000;  // 8 phút đo lần 2
const unsigned long TIME_MEASURE_3 = 900000;  // 15 phút đo lần 3 (chốt)
const unsigned long TIMEOUT_CYCLE  = 960000;  // Timeout an toàn
const unsigned long DEFAULT_MANUAL_INT = 300000; // 5 phút nếu chạy Manual
const unsigned long UART_TIMEOUT = 15000;     // 15s giữ UART active

// --- CONSTRUCTOR ---
// [FIX] Không cần truyền JsonFormatter nữa
StateMachine::StateMachine(Measurement& meas, RelayController& relay, UARTCommander& cmd, TimeSync& timeSync)
    : _meas(meas), _relay(relay), _cmd(cmd), _timeSync(timeSync) {
    _mode = MANUAL; // Mặc định chạy tay để test
    _cycleState = STATE_IDLE;
    _measuresPerDay = DEFAULT_MEASURES_PER_DAY;
    _currentManualInterval = DEFAULT_MANUAL_INT;
    _lastTriggerMinute = -1;
    _measureCount = 0;
    _stopRequested = false;
    _calculateGridInterval(); // Tính toán chia thời gian trong ngày
}

// --- SETUP BAN ĐẦU ---
void StateMachine::begin() {
    pinMode(PIN_SLEEP_STATUS, OUTPUT);
    _setBusyPin(true); // Báo bận (High)
    
    // Cấu hình đánh thức bằng GPIO (nút bấm hoặc Bridge kích)
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKEUP_GPIO, 1);
    
    _stopAndResetCycle(); // Đưa về trạng thái an toàn
    
    // Nếu tỉnh dậy từ nút bấm/Bridge
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
        JsonDocument doc; doc["WakeUp"] = "Done";
        String s; serializeJson(doc, s); _sendResponse(s);
    }
    
    _sendResponse(_json.createAck("SYSTEM_READY"));
    delay(50);
    _sendResponse(_timeSync.getRequestJson()); // Yêu cầu đồng bộ giờ
}

// --- LOOP CHÍNH (Được gọi liên tục trong Task) ---
void StateMachine::update() {
    // 1. Xử lý cờ dừng khẩn cấp
    if (_stopRequested) {
        _stopAndResetCycle();
        _stopRequested = false;
        _sendResponse(_json.createAck("STOPPED_BY_FLAG"));
        return;
    }

    // 2. Máy trạng thái đang chạy chu trình đo
    if (_cycleState != STATE_IDLE) {
        _processCycleLogic();
        
        // Nếu đang Auto mode và đang chờ giữa các lần đo -> Ngủ nhẹ (Light Sleep) tiết kiệm pin
        if (_mode == AUTO && (_cycleState >= STATE_WAIT_MEASURE_1 && _cycleState <= STATE_WAIT_MEASURE_3)) {
            _tryLightSleep();
        }
        return;
    }

    // 3. Logic AUTO MODE (Chờ đến giờ đo)
    if (_mode == AUTO) {
        // Nếu UART đang active (vừa nhận lệnh), giữ CPU chạy thêm chút
        if (_isUartWakeupActive) {
            if (millis() - _uartWakeupMillis > UART_TIMEOUT) _isUartWakeupActive = false;
            else return;
        }

        unsigned long now = _timeSync.getCurrentTime();
        if (now > 100000) { // Đã có thời gian thực
            if (_isTimeForScheduledMeasure() || _isTimeForGridMeasure()) {
                _startCycle();
                return;
            }
        }
        
        // Không làm gì thì ngủ (Light Sleep chờ timer hoặc Deep Sleep nếu xong hết)
        _tryLightSleep();
    }
}

// --- XỬ LÝ LỆNH TỪ UART ---
void StateMachine::handleCommand(const String& jsonCmd) {
    processJsonCommand(jsonCmd);
}

void StateMachine::processJsonCommand(const String& jsonStr) {
    if (_isUartWakeupActive) _uartWakeupMillis = millis(); // Gia hạn thời gian thức
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (error) return;

    // Lệnh tắt nguồn (Deep Sleep ngay lập tức)
    if (doc["EN"].is<int>() && doc["EN"] == 0) {
        _handleDeepSleepSequence();
        return;
    }
    
    bool cfgChanged = false;
    
    // Đồng bộ thời gian
    if (doc["timestamp"].is<unsigned long>()) {
        unsigned long ts = doc["timestamp"];
        if (ts > 0) { _timeSync.updateEpoch(ts); _lastTriggerMinute = -1; cfgChanged = true; }
    }
    
    // Cấu hình số lần đo
    if (doc["measures_per_day"].is<int>()) { 
        _measuresPerDay = doc["measures_per_day"]; 
        _calculateGridInterval(); 
        cfgChanged = true; 
    }
    
    // Cấu hình lịch đo cụ thể (HH:MM)
    if (doc["set_time"]) { 
        _parseSetTime(doc["set_time"].as<String>()); 
        cfgChanged = true; 
    }

    // Chuyển chế độ
    const char* m = doc["mode"];
    if (m) {
        if (strcmp(m, "manual")==0) { 
            _mode = MANUAL; 
            _stopAndResetCycle(); 
            _sendResponse(_json.createAck("MODE_MANUAL")); 
        }
        else if (strcmp(m, "auto")==0) { 
            _mode = AUTO; 
            _stopAndResetCycle(); 
            _lastTriggerMinute = -1; 
            _sendResponse(_json.createAck("MODE_AUTO")); 
        }
    }

    // Lệnh điều khiển
    const char* cmd = doc["cmd"];
    const char* st = doc["set_state"];
    bool isStop = (st && strcmp(st,"stop")==0) || (cmd && strcmp(cmd,"stop_measure")==0);
    bool isTrig = (st && strcmp(st,"measure")==0) || (cmd && strcmp(cmd,"trigger_measure")==0);

    if (isStop) { 
        setStopFlag(true); 
    }
    else if (isTrig) {
        if (_cycleState != STATE_IDLE && _cycleState != STATE_MANUAL_LOOP) {
            _sendResponse(_json.createError("BUSY"));
        } else { 
            _mode = MANUAL; 
            _startManualLoop(); 
            _sendResponse(_json.createAck("MEASURE_STARTED")); 
        }
    }

    // Lệnh điều khiển trực tiếp (Chỉ Manual)
    if (_mode == MANUAL) {
        if (doc["set_door"]) _handleDirectCommand(doc["set_door"]);
        if (doc["set_fans"]) _handleDirectCommand(doc["set_fans"]);
    }
    
    if (cfgChanged) _sendResponse(_json.createAck("CONFIG_OK"));
}

void StateMachine::_handleDirectCommand(const char* cmd) {
    if (strcmp(cmd,"open")==0) _relay.ON_DOOR();
    else if (strcmp(cmd,"close")==0) _relay.OFF_DOOR();
    else if (strcmp(cmd,"on")==0) _relay.ON_FAN();
    else if (strcmp(cmd,"off")==0) _relay.OFF_FAN();
}

// --- LOGIC CHU TRÌNH ĐO (CORE LOGIC) ---
void StateMachine::_startCycle() {
    _cycleState = STATE_PREPARING;
    _cycleStartMillis = millis();
    _measureCount = 0;
    _relay.OFF_DOOR(); // Đóng cửa buồng
    _relay.ON_FAN();   // Bật quạt hút
    // Reset mảng dữ liệu tạm
    for(int i=0;i<3;i++) _miniData[i] = {-1,-1,-1,-1,-1,-1,-1};
}

void StateMachine::_processCycleLogic() {
    unsigned long el = millis() - _cycleStartMillis;
    
    // Timeout an toàn
    if (el > TIMEOUT_CYCLE && _cycleState != STATE_MANUAL_LOOP) { 
        _finishCycle(); return; 
    }

    switch (_cycleState) {
        case STATE_PREPARING:
            if (el >= TIME_PREPARE) _cycleState = STATE_WAIT_MEASURE_1; 
            break;
            
        case STATE_WAIT_MEASURE_1:
            if (el >= TIME_MEASURE_1) { 
                _setBusyPin(true); // Thức dậy để đo
                _meas.doFullMeasurement(_miniData[0]); 
                _cycleState = STATE_WAIT_MEASURE_2; 
            } break;
            
        case STATE_WAIT_MEASURE_2:
            if (el >= TIME_MEASURE_2) { 
                _setBusyPin(true); 
                _meas.doFullMeasurement(_miniData[1]); 
                _cycleState = STATE_WAIT_MEASURE_3; 
            } break;
            
        case STATE_WAIT_MEASURE_3:
            if (el >= TIME_MEASURE_3) { 
                _setBusyPin(true); 
                _meas.doFullMeasurement(_miniData[2]); 
                _cycleState = STATE_FINISHING; 
            } break;
            
        case STATE_FINISHING:
            _finishCycle(); // Tính trung bình và gửi
            break;
            
        case STATE_MANUAL_LOOP: // Chế độ loop tay liên tục
            if (millis() >= _nextManualMeasureMillis) {
                MeasurementData t; _meas.doFullMeasurement(t);
                _sendResponse(_json.createDataJson(t.ch4, t.co, t.alc, t.nh3, t.h2, t.temp, t.hum));
                _nextManualMeasureMillis = millis() + _currentManualInterval;
            } break;
            
        default: break;
    }
}

// Tính trung bình 3 lần đo và gửi
void StateMachine::_finishCycle() {
    float s[7]={0}; int c[7]={0};
    // Cộng dồn các giá trị hợp lệ (>-90)
    for(int i=0; i<3; i++) {
        float v[]={_miniData[i].ch4, _miniData[i].co, _miniData[i].alc, _miniData[i].nh3, _miniData[i].h2, _miniData[i].temp, _miniData[i].hum};
        for(int k=0; k<7; k++) if(v[k] > -90) { s[k]+=v[k]; c[k]++; }
    }
    // Chia trung bình
    float avg[7]; for(int k=0;k<7;k++) avg[k] = (c[k]>0) ? (s[k]/c[k]) : -1.0;
    
    // Gửi kết quả
    _sendResponse(_json.createDataJson(avg[0], avg[1], avg[2], avg[3], avg[4], avg[5], avg[6]));
    _stopAndResetCycle();
}

void StateMachine::_stopAndResetCycle() {
    _cycleState = STATE_IDLE; 
    _relay.OFF_FAN(); 
    _relay.ON_DOOR(); // Mở cửa xả khí
    _setBusyPin(true); 
    _measureCount = 0;
}

void StateMachine::_startManualLoop() {
    _cycleState = STATE_MANUAL_LOOP; 
    _relay.OFF_DOOR(); 
    _relay.ON_FAN();
    _nextManualMeasureMillis = millis() + 1000;
}

void StateMachine::setStopFlag(bool flag) { _stopRequested = flag; }

// --- LOGIC SLEEP (QUAN TRỌNG) ---
void StateMachine::_handleDeepSleepSequence() {
    _relay.OFF_DOOR(); 
    _relay.OFF_FAN(); 
    delay(2000);
    _sendResponse(_json.createAck("EN:0")); 
    delay(500);
    _setBusyPin(false); // Báo ngủ
    esp_deep_sleep_start();
}

void StateMachine::_tryLightSleep() {
    unsigned long sleepT = 0;
    unsigned long el = millis() - _cycleStartMillis;
    
    // Tính thời gian ngủ dựa trên trạng thái hiện tại
    if (_cycleState == STATE_IDLE) sleepT = _calculateNextWakeTime();
    else if (_cycleState == STATE_WAIT_MEASURE_1) sleepT = TIME_MEASURE_1 - el;
    else if (_cycleState == STATE_WAIT_MEASURE_2) sleepT = TIME_MEASURE_2 - el;
    else if (_cycleState == STATE_WAIT_MEASURE_3) sleepT = TIME_MEASURE_3 - el;

    // Chỉ ngủ nếu thời gian > 5s
    if (sleepT > 5000) {
        esp_sleep_enable_timer_wakeup((sleepT - 2000) * 1000ULL); // Trừ hao 2s
        esp_sleep_enable_uart_wakeup(1); // Cho phép UART đánh thức
        _setBusyPin(false); // Pin 32 LOW
        
        esp_light_sleep_start();
        
        _setBusyPin(true); // Pin 32 HIGH ngay khi dậy
        
        // Nếu bị đánh thức bởi UART
        if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UART) {
            while(Serial1.available()) Serial1.read(); // Xả buffer rác
            JsonDocument d; d["WakeUp"] = "Done"; String s; serializeJson(d,s); _sendResponse(s);
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
    
    // 1. Tính theo chu kỳ đều (Grid Interval)
    if (_gridIntervalSeconds > 0) {
        unsigned long nxt = ((now / _gridIntervalSeconds) + 1) * _gridIntervalSeconds;
        long w = nxt - now;
        if (w > 0 && w < minW) { minW = w; f = true; }
    }
    
    // 2. Tính theo lịch cố định (Schedule)
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

// --- GỬI DỮ LIỆU CÓ CRC ---
void StateMachine::_sendResponse(String json) {
    unsigned long crc = CRC32::calculate(json);
    // Đẩy vào Queue, Task main sẽ gửi đi
    _cmd.pushToQueue(json + "|" + String(crc, HEX));
}

// --- HELPER FUNCTIONS ---
void StateMachine::_calculateGridInterval() {
    if (_measuresPerDay <= 0) _measuresPerDay = 1;
    _gridIntervalSeconds = 86400 / _measuresPerDay;
}

void StateMachine::_parseSetTime(String tStr) {
    int h, m; 
    if (sscanf(tStr.c_str(), "%d:%d", &h, &m)==2) {
        if (h<0||h>23||m<0||m>59) return;
        // Kiểm tra trùng
        for (auto& s : _schedules) if (s.hour==h && s.minute==m) return;
        if (_schedules.size()>=10) _schedules.erase(_schedules.begin());
        _schedules.push_back({h, m, 0});
    }
}

bool StateMachine::_isTimeForGridMeasure() {
    unsigned long now = _timeSync.getCurrentTime();
    // Sai số cho phép 5s
    return (now > 100000) && ((now % _gridIntervalSeconds) < 5);
}

bool StateMachine::_isTimeForScheduledMeasure() {
    unsigned long now = _timeSync.getCurrentTime();
    time_t r = (time_t)now; struct tm* t = localtime(&r);
    
    // Tránh trigger nhiều lần trong 1 phút
    if (t->tm_min == _lastTriggerMinute) return false;
    
    for (auto& s : _schedules) {
        if (t->tm_hour==s.hour && t->tm_min==s.minute) { 
            _lastTriggerMinute = t->tm_min; 
            return true; 
        }
    }
    return false;
}