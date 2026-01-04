#include "StateMachine.h"
#include "Measurement.h"
#include "RelayController.h"
#include "UARTCommander.h"
#include "TimeSync.h"
#include <ArduinoJson.h>
#include "CRC32.h"
#include "Config.h"

// --- CẤU HÌNH THỜI GIAN ---
const unsigned long TIME_STABILIZE = 60000;     
const unsigned long TIME_MEASURE_1 = 180000;    
const unsigned long TIME_MEASURE_2 = 480000;    
const unsigned long TIME_MEASURE_3 = 900000;    
const unsigned long DEFAULT_MANUAL_INTERVAL = 300000; 

StateMachine::StateMachine(Measurement& meas, RelayController& relay, UARTCommander& cmd, TimeSync& timeSync)
    : _meas(meas), _relay(relay), _cmd(cmd), _timeSync(timeSync) {
    
    _mode = MANUAL; 
    _cycleState = STATE_IDLE;
    
    _measuresPerDay = DEFAULT_MEASURES_PER_DAY;
    _currentManualInterval = DEFAULT_MANUAL_INTERVAL;
    
    _lastGridBlock = 0;
    _lastTriggerMinute = -1; 

    _calculateGridInterval();
    _schedules.clear();
}

void StateMachine::begin() {
    Serial.println("[StateMachine] Started. Mode: MANUAL");
    _sendResponse(_json.createTimeSyncRequest());
}

void StateMachine::update() {
    // 1. Ưu tiên xử lý quy trình đo
    if (_cycleState != STATE_IDLE) {
        _processCycleLogic();
        return; 
    }

    // 2. Logic AUTO (Chỉ chạy khi rảnh)
    if (_mode == AUTO) {
        unsigned long currentEpoch = _timeSync.getCurrentTime();
        
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 10000) lastLog = millis();

        if (currentEpoch > 100000) {
            bool trigger = false;
            
            if (_isTimeForScheduledMeasure()) {
                Serial.println("[AUTO] Trigger: Scheduled Time Match!");
                trigger = true;
            } 
            else if (_isTimeForGridMeasure()) {
                Serial.println("[AUTO] Trigger: Grid Cycle Match!");
                trigger = true;
            }

            if (trigger) {
                _startCycle();
            }
        }
    }
}

// =================================================================================
// XỬ LÝ LỆNH: FULL ACK + NODE-RED COMPATIBLE + ARDUINOJSON V7 FIX
// =================================================================================
void StateMachine::handleCommand(const String& cleanJson) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, cleanJson);
    if (error) {
        _sendResponse(_json.createError("JSON_ERR"));
        return;
    }
    Serial.println("[SM] Processing Command...");
    bool configChanged = false;
    bool ackSent = false;
    // ============================================================
    // 1. CẤU HÌNH & THỜI GIAN
    // ============================================================
   // --- XỬ LÝ TIMESTAMP VỚI DEBUG ---
    if (doc["timestamp"].is<unsigned long>()) {
        unsigned long ts = doc["timestamp"];
        
        // [DEBUG] In ra giá trị timestamp nhận được
        Serial.printf("[DEBUG] RX Timestamp: %lu\n", ts);

        if (ts > 0) {
            unsigned long oldTime = _timeSync.getCurrentTime();
            _timeSync.updateEpoch(ts);
            unsigned long newTime = _timeSync.getCurrentTime();

            _lastTriggerMinute = -1; 
            configChanged = true;
            
            Serial.printf("[DEBUG] Time Updated: %lu -> %lu (Diff: %ld ms)\n", oldTime, newTime, (long)(newTime - oldTime));
            Serial.println("[DEBUG] Time Synced OK");
        } else {
            Serial.println("[DEBUG] Error: Timestamp is 0, ignored.");
        }
    }
    else if (doc["timestamp"]) {
        Serial.println("[DEBUG] Error: 'timestamp' found but is NOT unsigned long (Check Gateway sending type).");
    }

    // Cài giờ (Schedule)
    if (doc["set_time"].is<const char*>()) {
        const char* tStr = doc["set_time"];
        _parseSetTime(String(tStr));
        configChanged = true;
    }

    // Cấu hình Auto (Số lần đo/ngày)
    if (doc["measures_per_day"].is<int>()) {
        int n = doc["measures_per_day"];
        if (n != _measuresPerDay) { 
            _measuresPerDay = n;
            _calculateGridInterval();
            Serial.printf("[SET] Auto Grid: %d/day\n", n);
            configChanged = true;
        }
    }
    // Hỗ trợ lệnh cũ
    if (doc["cmd"] == "set_cycle" && doc["val"].is<int>()) {
        _measuresPerDay = doc["val"];
        _calculateGridInterval();
        configChanged = true;
    }

    // Cấu hình Manual (Cycle)
    if (doc["cycle_manual"].is<int>()) {
        int mins = doc["cycle_manual"];
        if (mins < 1) mins = 1; 
        if (mins > 59) mins = 59;
        unsigned long newInterval = (unsigned long)mins * 60000UL;
        
        if (_currentManualInterval != newInterval) {
            _currentManualInterval = newInterval;
            Serial.printf("[SET] Set Manual Cycle: %d min\n", mins);
            configChanged = true;
        }
    }
    else if (doc["cycle"].is<int>()) { // Hỗ trợ cũ
         int mins = doc["cycle"];
         if (mins > 0) {
             _currentManualInterval = (unsigned long)mins * 60000UL;
             configChanged = true;
         }
    }

    // ============================================================
    // 2. CHẾ ĐỘ HOẠT ĐỘNG (MODE)
    // ============================================================
    if (doc["mode"].is<const char*>()) {
        const char* m = doc["mode"];
        if (strcmp(m, "manual") == 0) {
            if (_mode != MANUAL) {
                _mode = MANUAL;
                _stopAndResetCycle(); 
                Serial.println("[SET] Mode: MANUAL");
                configChanged = true;
            }
        } 
        else if (strcmp(m, "auto") == 0) {
            if (_mode != AUTO) {
                _mode = AUTO;
                _stopAndResetCycle();
                _lastTriggerMinute = -1; 
                Serial.println("[SET] Mode: AUTO");
                configChanged = true;
            }
        }
    }

    // ============================================================
    // 3. TRẠNG THÁI HOẠT ĐỘNG (TRIGGER / STOP)
    // ============================================================
    bool isTrigger = false;
    bool isStop = false;

    if (doc["set_state"].is<const char*>()) {
        const char* s = doc["set_state"];
        if (strcmp(s, "measure") == 0) isTrigger = true;
        else if (strcmp(s, "stop") == 0) isStop = true;
    }
    if (doc["cmd"].is<const char*>()) {
        const char* c = doc["cmd"];
        if (strcmp(c, "trigger_measure") == 0) isTrigger = true;
        else if (strcmp(c, "stop_measure") == 0) isStop = true;
    }

    if (isStop) {
        _stopAndResetCycle();
        _sendResponse(_json.createAck("MEASURE_STOPPED"));
        ackSent = true;
    }
    else if (isTrigger) {
        if (_cycleState != STATE_IDLE && _cycleState != STATE_MANUAL_LOOP) {
            _sendResponse(_json.createError("BUSY")); 
            ackSent = true;
        } else {
            if (_mode != MANUAL) { 
                _mode = MANUAL; 
                configChanged = true; 
            }
            _startManualLoop();
            _sendResponse(_json.createAck("MEASURE_STARTED"));
            ackSent = true;
        }
    }

    // ============================================================
    // 4. ĐIỀU KHIỂN PHẦN CỨNG (Chỉ MANUAL)
    // ============================================================
    if (_mode == MANUAL) {
        if (doc["set_door"].is<const char*>()) {
            const char* d = doc["set_door"];
            if (strcmp(d, "open") == 0) { 
                _relay.ON_DOOR(); 
                _sendResponse(_json.createAck("DOOR_OPENED")); ackSent = true; 
            }
            else if (strcmp(d, "close") == 0) { 
                _relay.OFF_DOOR(); 
                _sendResponse(_json.createAck("DOOR_CLOSED")); ackSent = true; 
            }
        }
        if (doc["set_fans"].is<const char*>()) {
            const char* f = doc["set_fans"];
            if (strcmp(f, "on") == 0) { 
                _relay.ON_FAN(); 
                _sendResponse(_json.createAck("FANS_ON")); ackSent = true; 
            }
            else if (strcmp(f, "off") == 0) { 
                _relay.OFF_FAN(); 
                _sendResponse(_json.createAck("FANS_OFF")); ackSent = true; 
            }
        }
        // Lệnh cũ
        if (!ackSent && doc["cmd"].is<const char*>()) {
             const char* c = doc["cmd"];
             if (strcmp(c, "open_door")==0) { _relay.ON_DOOR(); _sendResponse(_json.createAck("DOOR_OPENED")); ackSent=true;}
             else if (strcmp(c, "close_door")==0) { _relay.OFF_DOOR(); _sendResponse(_json.createAck("DOOR_CLOSED")); ackSent=true;}
             else if (strcmp(c, "fans_on")==0) { _relay.ON_FAN(); _sendResponse(_json.createAck("FANS_ON")); ackSent=true;}
             else if (strcmp(c, "fans_off")==0) { _relay.OFF_FAN(); _sendResponse(_json.createAck("FANS_OFF")); ackSent=true;}
        }
    } 
    else {
        if (doc["set_door"].is<const char*>() || doc["set_fans"].is<const char*>()) {
            _sendResponse(_json.createError("ERR_IN_AUTO")); 
            ackSent = true;
        }
    }

    if (configChanged && !ackSent) {
        _sendResponse(_json.createAck("CONFIG_OK"));
    }
}

// =================================================================================
// CÁC HÀM HELPER
// =================================================================================

void StateMachine::_calculateGridInterval() {
    if (_measuresPerDay <= 0) _measuresPerDay = 1;
    _gridIntervalSeconds = SECONDS_IN_DAY / _measuresPerDay;
}

void StateMachine::_parseSetTime(String timeStr) {
    int h, m, s;
    if (sscanf(timeStr.c_str(), "%d:%d:%d", &h, &m, &s) == 3) {
        for (const auto& sch : _schedules) {
            if (sch.hour == h && sch.minute == m) return; 
        }
        if (_schedules.size() >= 10) _schedules.erase(_schedules.begin());
        _schedules.push_back({h, m, s});
        Serial.printf("[SM] Schedule added: %02d:%02d:%02d\n", h, m, s);
    }
}

bool StateMachine::_isTimeForGridMeasure() {
    unsigned long currentEpoch = _timeSync.getCurrentTime();
    if (currentEpoch < 100000) return false;
    return (currentEpoch % _gridIntervalSeconds) < 5;
}

bool StateMachine::_isTimeForScheduledMeasure() {
    unsigned long currentEpoch = _timeSync.getCurrentTime();
    if (currentEpoch < 100000) return false;
    
    time_t raw = (time_t)currentEpoch;
    struct tm* t = localtime(&raw);

    if (t->tm_min == _lastTriggerMinute) return false;

    for (const auto& s : _schedules) {
        if (t->tm_hour == s.hour && t->tm_min == s.minute) {
            _lastTriggerMinute = t->tm_min; 
            Serial.printf("[SM] Match Schedule: %02d:%02d\n", s.hour, s.minute);
            return true;
        }
    }
    return false;
}

void StateMachine::_startCycle() { 
    _cycleState = STATE_PREPARING;
    _cycleStartMillis = millis();
    memset(_miniData, 0, sizeof(_miniData)); 
    _relay.OFF_DOOR(); 
    _relay.ON_FAN();   
    Serial.println("[SM] Auto Cycle Started.");
}

void StateMachine::_startManualLoop() { 
    _cycleState = STATE_MANUAL_LOOP;
    _relay.OFF_DOOR(); 
    _relay.ON_FAN();   
    _nextManualMeasureMillis = millis() + 1000;
    Serial.println("[SM] Manual Loop Started.");
}

// ------------------------------------------------------------------
// QUAN TRỌNG: CẬP NHẬT LOGIC ĐO LƯỜNG VÀ BÁO LỖI
// ------------------------------------------------------------------
void StateMachine::_processCycleLogic() {
    unsigned long elapsed = millis() - _cycleStartMillis;

    // Timeout an toàn cho Auto Cycle
    if (elapsed > 960000 && _cycleState != STATE_MANUAL_LOOP) {
        Serial.println("[SM] Cycle Timeout!");
        _finishCycle();
        return;
    }

    switch (_cycleState) {
        case STATE_PREPARING:
            _cycleState = STATE_STABILIZING;
            Serial.println("[SM] -> STABILIZING (60s)");
            break;
        case STATE_STABILIZING:
            if (elapsed >= TIME_STABILIZE) {
                _cycleState = STATE_WAIT_MEASURE_1;
                Serial.println("[SM] -> WAIT M1");
            }
            break;
        case STATE_WAIT_MEASURE_1:
            if (elapsed >= TIME_MEASURE_1) {
                Serial.println("[SM] Measuring 1/3...");
                // Note: Tạm thời không check lỗi ở bước trung gian này, để dành check ở _finishCycle
                _meas.doFullMeasurement(_miniData[0]);
                _cycleState = STATE_WAIT_MEASURE_2;
            }
            break;
        case STATE_WAIT_MEASURE_2:
            if (elapsed >= TIME_MEASURE_2) {
                Serial.println("[SM] Measuring 2/3...");
                _meas.doFullMeasurement(_miniData[1]);
                _cycleState = STATE_WAIT_MEASURE_3;
            }
            break;
        case STATE_WAIT_MEASURE_3:
            if (elapsed >= TIME_MEASURE_3) {
                Serial.println("[SM] Measuring 3/3...");
                _meas.doFullMeasurement(_miniData[2]);
                _cycleState = STATE_FINISHING;
            }
            break;
        case STATE_FINISHING:
            _finishCycle();
            break;

        // --- XỬ LÝ MANUAL MODE VỚI BÁO LỖI ---
        case STATE_MANUAL_LOOP:
            if (millis() >= _nextManualMeasureMillis) {
                Serial.println("[SM-MANUAL] Measuring...");
                MeasurementData tempData;
                bool result = _meas.doFullMeasurement(tempData); 
                
                if (result) {
                    // ĐO THÀNH CÔNG -> GỬI DỮ LIỆU
                    String jsonStr = _json.createDataJson(
                        tempData.ch4, tempData.co, tempData.alc, 
                        tempData.nh3, tempData.h2, tempData.temp, tempData.hum
                    );
                    _sendResponse(jsonStr); 
                } else {
                    // ĐO THẤT BẠI -> GỬI LỖI (dht, slave, v.v...)
                    String errorName = _meas.getLastError(); // Lấy tên lỗi từ Measurement
                    if (errorName.length() == 0) errorName = "sensor_err"; // Fallback nếu rỗng
                    
                    Serial.print("[SM-MANUAL] Error detected: "); 
                    Serial.println(errorName);
                    
                    _sendResponse(_json.createError(errorName));
                }
                
                _nextManualMeasureMillis = millis() + _currentManualInterval;
                Serial.printf("[SM-MANUAL] Next in: %lu ms\n", _currentManualInterval);
            }
            break;
    }
}

// --- CẬP NHẬT KẾT THÚC CHU TRÌNH AUTO ---
void StateMachine::_finishCycle() {
    float sum[7] = {0}; 
    int count = 0;
    
    // Tính trung bình các lần đo THÀNH CÔNG
    for(int i=0; i<3; i++) {
        // Kiểm tra điều kiện dữ liệu hợp lệ (Temp khác 0 là một dấu hiệu đơn giản)
        // Hoặc check cờ valid nếu struct miniData có
        if(_miniData[i].temp != 0 && !isnan(_miniData[i].temp)) { 
            sum[0]+=_miniData[i].ch4; sum[1]+=_miniData[i].co; sum[2]+=_miniData[i].alc;
            sum[3]+=_miniData[i].nh3; sum[4]+=_miniData[i].h2;
            sum[5]+=_miniData[i].temp; sum[6]+=_miniData[i].hum;
            count++;
        }
    }
    
    if(count > 0) {
        // NẾU CÓ ÍT NHẤT 1 LẦN ĐO THÀNH CÔNG
        for(int k=0; k<7; k++) sum[k] /= count;
        String json = _json.createDataJson(sum[0], sum[1], sum[2], sum[3], sum[4], sum[5], sum[6]);
        _sendResponse(json);
        Serial.printf("[SM] Cycle Done. Valid measures: %d/3\n", count);
    } 
    else {
        // NẾU CẢ 3 LẦN ĐỀU THẤT BẠI
        String errorName = _meas.getLastError(); // Lấy lỗi của lần đo cuối cùng
        if (errorName.length() == 0) errorName = "cycle_fail";
        
        Serial.println("[SM] Cycle Failed! All measurements error.");
        _sendResponse(_json.createError(errorName));
    }

    _stopAndResetCycle();
}

void StateMachine::_stopAndResetCycle() {
    _cycleState = STATE_IDLE;
    _relay.OFF_FAN();  
    _relay.ON_DOOR();
    Serial.println("[SM] Reset -> IDLE");
}

void StateMachine::_sendResponse(String json) {
    unsigned long crc = CRC32::calculate(json);
    String packet = json + "|" + String(crc, HEX);
    _cmd.pushToQueue(packet); 
}

void StateMachine::setStopFlag(bool flag) {
    if (flag) _stopAndResetCycle();
}