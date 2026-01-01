#include "StateMachine.h"
#include "Measurement.h"
#include "RelayController.h"
#include "UARTCommander.h"
#include "TimeSync.h"
#include <ArduinoJson.h>
#include "CRC32.h"

// Cấu hình thời gian (ms)
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
    _lastTriggerMinute = -1; // Reset bộ đếm phút

    _calculateGridInterval();
    _schedules.clear();
}

void StateMachine::begin() {
    Serial.println("[StateMachine] Started. Mode: MANUAL");
    _sendResponse(_json.createTimeSyncRequest());
}

// --- LOGIC LOOP CHÍNH ---
void StateMachine::update() {
    // 1. Nếu đang bận (Đang đo) -> Chạy quy trình đo
    if (_cycleState != STATE_IDLE) {
        _processCycleLogic();
        return; 
    }

    // 2. CHECK AUTO (Chỉ chạy khi Mode là AUTO và Rảnh)
    if (_mode == AUTO) {
        unsigned long currentEpoch = _timeSync.getCurrentTime();
        
        // Debug: In ra trạng thái mỗi 10 giây để biết Auto còn sống không
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 10000) {
            lastLog = millis();
        }

        // Điều kiện 1: Đã đồng bộ giờ (Lấy mốc > 100000 theo code gốc của bạn)
        if (currentEpoch > 100000) {
            bool trigger = false;
            
            // Ưu tiên: Check lịch hẹn giờ
            if (_isTimeForScheduledMeasure()) {
                Serial.println("[AUTO] Trigger: Scheduled Time Match!");
                trigger = true;
            } 
            // Sau đó: Check lịch theo lưới (Grid)
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

// --- XỬ LÝ LỆNH ---
void StateMachine::handleCommand(const String& cleanJson) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, cleanJson);
    
    if (error) {
        _sendResponse(_json.createError("JSON_ERR"));
        return;
    }
    
    if (!doc["type"].is<const char*>()) return;
    const char* type = doc["type"];
    Serial.printf("[INFO] Cmd: %s\n", type);

    // 1. SET MODE
    if (strcmp(type, "set_mode") == 0) {
        if (doc["mode"].is<String>()) {
            String m = doc["mode"];
            if (m == "manual") {
                _mode = MANUAL;
                _stopAndResetCycle(); 
                _sendResponse(_json.createAck("MANUAL_OK"));
            } else {
                _mode = AUTO;
                _stopAndResetCycle();
                // [QUAN TRỌNG] Reset lại bộ đếm để Auto không bị kẹt
                _lastTriggerMinute = -1; 
                _lastGridBlock = 0;
                
                Serial.println("[SM] Mode -> AUTO");
                _sendResponse(_json.createAck("AUTO_OK"));
            }
        }
        return;
    }

    // 2. TRIGGER MEASURE (Tự động chuyển Manual)
    if (strcmp(type, "trigger_measure") == 0) {
        if (_cycleState != STATE_IDLE && _cycleState != STATE_MANUAL_LOOP) {
             _sendResponse(_json.createError("BUSY"));
             return;
        }

        if (_mode != MANUAL) {
            _mode = MANUAL;
            Serial.println("[SM] Auto-switch to MANUAL by Trigger");
        }

        if (doc["cycle"].is<int>()) {
            int mins = doc["cycle"];
            if (mins < 1) mins = 1; if (mins > 59) mins = 59;
            _currentManualInterval = (unsigned long)mins * 60000UL;
            Serial.printf("[SM] Custom Cycle: %d mins\n", mins);
        } else {
             if(_currentManualInterval == 0) _currentManualInterval = DEFAULT_MANUAL_INTERVAL;
        }

        _startManualLoop();
        _sendResponse(_json.createAck("MEASURE_LOOP_STARTED"));
        return;
    }

    // 3. SET TIME (Thêm lịch hẹn)
    if (strcmp(type, "set_time") == 0) {
        if (doc["time"].is<String>()) {
            _parseSetTime(doc["time"].as<String>());
            _sendResponse(_json.createAck("SCHEDULE_ADDED"));
        }
        return;
    }

    // 4. STOP
    if (strcmp(type, "stop_measure") == 0) {
        _stopAndResetCycle();
        _sendResponse(_json.createAck("MEASURE_STOPPED"));
        return;
    }

    // 5. TIME SYNC
    if (strcmp(type, "time_sync") == 0) {
        String timeStr = "";
        if (doc["timestamp"].is<String>()) timeStr = doc["timestamp"].as<String>();
        else if (doc["time"].is<String>()) timeStr = doc["time"].as<String>();

        if (timeStr != "") {
            _timeSync.syncFromTimestamp(timeStr);
            _lastTriggerMinute = -1; // Reset để có thể trigger ngay nếu trùng phút
            _sendResponse(_json.createAck("TIME_SYNCED"));
        }
        return;
    }

    // 6. SET CYCLE (GRID)
    if (strcmp(type, "set_cycle") == 0) {
        if (doc["measures_per_day"].is<int>()) {
            _measuresPerDay = doc["measures_per_day"];
            _calculateGridInterval();
            _sendResponse(_json.createAck("CYCLE_SET_OK"));
        }
        return;
    }

    // Relay Control (Manual only)
    if (_mode == MANUAL) {
        if (strcmp(type, "open_door") == 0)  { _relay.ON_DOOR();  _sendResponse(_json.createAck("DOOR_OPENED")); return; }
        if (strcmp(type, "close_door") == 0) { _relay.OFF_DOOR(); _sendResponse(_json.createAck("DOOR_CLOSED")); return; }
        if (strcmp(type, "fans_on") == 0)    { _relay.ON_FAN();   _sendResponse(_json.createAck("FAN_ON_OK"));   return; }
        if (strcmp(type, "fans_off") == 0)   { _relay.OFF_FAN();  _sendResponse(_json.createAck("FAN_OFF_OK"));  return; }
    } else {
        if (strcmp(type, "open_door")==0 || strcmp(type, "close_door")==0 || strcmp(type, "fans_on")==0) {
            _sendResponse(_json.createError("IN AUTO MODE"));
        }
    }
}

// --- HELPER LOGIC THỜI GIAN (ĐÃ TỐI ƯU) ---

void StateMachine::_calculateGridInterval() {
    if (_measuresPerDay <= 0) _measuresPerDay = 1;
    _gridIntervalSeconds = SECONDS_IN_DAY / _measuresPerDay;
}

void StateMachine::_parseSetTime(String timeStr) {
    int h, m, s;
    if (sscanf(timeStr.c_str(), "%d:%d:%d", &h, &m, &s) == 3) {
        for (const auto& sch : _schedules) {
            if (sch.hour == h && sch.minute == m) return; // Trùng giờ phút là bỏ qua
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

// [QUAN TRỌNG] Logic so sánh lịch hẹn an toàn
bool StateMachine::_isTimeForScheduledMeasure() {
    unsigned long currentEpoch = _timeSync.getCurrentTime();
    if (currentEpoch < 100000) return false;
    
    time_t raw = (time_t)currentEpoch;
    struct tm* t = localtime(&raw);

    // 1. Chống lặp (Debounce): Nếu phút này đã chạy rồi thì không chạy nữa
    if (t->tm_min == _lastTriggerMinute) return false;

    for (const auto& s : _schedules) {
        // 2. So sánh Giờ và Phút (Bỏ qua giây để đảm bảo chắc chắn chạy)
        if (t->tm_hour == s.hour && t->tm_min == s.minute) {
            // Cập nhật phút đã chạy để không lặp lại trong phút này
            _lastTriggerMinute = t->tm_min; 
            Serial.printf("[SM] Match Schedule: %02d:%02d\n", s.hour, s.minute);
            return true;
        }
    }
    return false;
}

// --- QUY TRÌNH ĐO ---

void StateMachine::_startCycle() { // AUTO
    _cycleState = STATE_PREPARING;
    _cycleStartMillis = millis();
    memset(_miniData, 0, sizeof(_miniData)); 
    _relay.OFF_DOOR(); 
    _relay.ON_FAN();   
    Serial.println("[SM] Auto Cycle Started.");
}

void StateMachine::_startManualLoop() { // MANUAL
    _cycleState = STATE_MANUAL_LOOP;
    _relay.OFF_DOOR(); 
    _relay.ON_FAN();   
    _nextManualMeasureMillis = millis() + 1000;
    Serial.println("[SM] Manual Loop Started.");
}

void StateMachine::_processCycleLogic() {
    unsigned long elapsed = millis() - _cycleStartMillis;

    // Timeout logic (Trừ Manual Loop)
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

        case STATE_MANUAL_LOOP:
            if (millis() >= _nextManualMeasureMillis) {
                Serial.println("[SM-MANUAL] Measuring...");
                MeasurementData tempData;
                bool result = _meas.doFullMeasurement(tempData); 

                // Gửi dữ liệu (hoặc 0 nếu lỗi)
                if (result) {
                    String json = _json.createDataJson(tempData.ch4, tempData.co, tempData.alc, tempData.nh3, tempData.h2, tempData.temp, tempData.hum);
                    _sendResponse(json);
                } else {
                    String json = _json.createDataJson(0, 0, 0, 0, 0, 0, 0);
                    _sendResponse(json);
                }
                
                _nextManualMeasureMillis = millis() + _currentManualInterval;
                Serial.printf("[SM-MANUAL] Next: %lu ms\n", _currentManualInterval);
            }
            break;
    }
}

void StateMachine::_finishCycle() {
    float sum[7] = {0}; 
    int count = 0;
    
    for(int i=0; i<3; i++) {
        if(_miniData[i].temp != 0) { 
            sum[0]+=_miniData[i].ch4; sum[1]+=_miniData[i].co; sum[2]+=_miniData[i].alc;
            sum[3]+=_miniData[i].nh3; sum[4]+=_miniData[i].h2;
            sum[5]+=_miniData[i].temp; sum[6]+=_miniData[i].hum;
            count++;
        }
    }
    
    if(count > 0) {
        for(int k=0; k<7; k++) sum[k] /= count;
    }

    String json = _json.createDataJson(sum[0], sum[1], sum[2], sum[3], sum[4], sum[5], sum[6]);
    _sendResponse(json);
    Serial.println("[SM] Cycle Done.");
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
    _cmd.send(packet); 
}

void StateMachine::setStopFlag(bool flag) {
    if (flag) _stopAndResetCycle();
}