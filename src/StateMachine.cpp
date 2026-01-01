#include "StateMachine.h"
#include "Measurement.h"
#include "RelayController.h"
#include "UARTCommander.h"
#include "TimeSync.h"
#include <ArduinoJson.h>
#include "CRC32.h"
#include "Config.h"

// --- CẤU HÌNH THỜI GIAN (Giữ nguyên như code cũ) ---
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

// --- LOGIC LOOP CHÍNH (Giữ nguyên logic cũ) ---
void StateMachine::update() {
    // 1. Ưu tiên xử lý quy trình đo
    if (_cycleState != STATE_IDLE) {
        _processCycleLogic();
        return; 
    }

    // 2. Logic AUTO (Chỉ chạy khi rảnh)
    if (_mode == AUTO) {
        unsigned long currentEpoch = _timeSync.getCurrentTime();
        
        // Heartbeat log
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 10000) lastLog = millis();

        if (currentEpoch > 100000) {
            bool trigger = false;
            
            // Logic cũ: Ưu tiên Lịch hẹn -> Sau đó đến Grid
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
// XỬ LÝ LỆNH TUẦN TỰ (BATCH PROCESSING) - LOGIC CŨ TRÊN CẤU TRÚC MỚI
// =================================================================================
void StateMachine::handleCommand(const String& cleanJson) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, cleanJson);
    
    if (error) {
        _sendResponse(_json.createError("JSON_ERR"));
        return;
    }

    Serial.println("[SM] Processing Batch...");

    // 1. TIME SYNC: Kiểm tra nếu trường timestamp tồn tại và là số
    // [FIX V7] Thay doc.containsKey("timestamp") bằng doc["timestamp"].is<unsigned long>()
    if (doc["timestamp"].is<unsigned long>()) { 
        unsigned long ts = doc["timestamp"];
        if (ts > 0) {
            _timeSync.updateEpoch(ts);
            _lastTriggerMinute = -1; 
            Serial.println("-> Time Synced");
        }
    }

    // 2. SET MODE: Kiểm tra nếu trường mode là chuỗi
    // [FIX V7] Thay doc.containsKey("mode") bằng doc["mode"].is<const char*>()
    if (doc["mode"].is<const char*>()) {
        const char* m = doc["mode"];
        if (strcmp(m, "manual") == 0) {
            if (_mode != MANUAL) {
                _mode = MANUAL;
                _stopAndResetCycle(); 
                Serial.println("-> Mode: MANUAL");
            }
        } 
        else if (strcmp(m, "auto") == 0) {
            if (_mode != AUTO) {
                _mode = AUTO;
                _stopAndResetCycle();
                _lastTriggerMinute = -1; 
                _lastGridBlock = 0;
                Serial.println("-> Mode: AUTO");
            }
        }
    }

    // 3. XỬ LÝ LỆNH "cmd"
    // [FIX V7] Thay doc.containsKey("cmd") bằng doc["cmd"].is<const char*>()
    if (doc["cmd"].is<const char*>()) {
        const char* cmd = doc["cmd"];
        Serial.printf("-> Cmd: %s\n", cmd);

        if (strcmp(cmd, "stop_measure") == 0) {
            _stopAndResetCycle();
            _sendResponse(_json.createAck("MEASURE_STOPPED"));
        }
        else if (strcmp(cmd, "trigger_measure") == 0) {
            if (_cycleState != STATE_IDLE && _cycleState != STATE_MANUAL_LOOP) {
                _sendResponse(_json.createError("BUSY"));
            } else {
                if (_mode != MANUAL) { 
                    _mode = MANUAL; 
                    Serial.println("-> Trigger Force MANUAL");
                }
                
                // [FIX V7] Thay doc.containsKey("cycle") -> doc["cycle"].is<int>()
                if (doc["cycle"].is<int>()) {
                    int mins = doc["cycle"];
                    if (mins < 1) mins = 1; if (mins > 59) mins = 59;
                    _currentManualInterval = (unsigned long)mins * 60000UL;
                    Serial.printf("-> Manual Cycle: %d min\n", mins);
                }
                
                _startManualLoop();
                _sendResponse(_json.createAck("MEASURE_LOOP_STARTED"));
            }
        }
        else { // Lệnh phần cứng (chỉ chạy ở Manual)
            if (_mode == MANUAL) {
                if (strcmp(cmd, "open_door") == 0)       { _relay.ON_DOOR(); _sendResponse(_json.createAck("DOOR_OPENED")); }
                else if (strcmp(cmd, "close_door") == 0) { _relay.OFF_DOOR(); _sendResponse(_json.createAck("DOOR_CLOSED")); }
                else if (strcmp(cmd, "fans_on") == 0)    { _relay.ON_FAN();  _sendResponse(_json.createAck("FAN_ON_OK")); }
                else if (strcmp(cmd, "fans_off") == 0)   { _relay.OFF_FAN(); _sendResponse(_json.createAck("FAN_OFF_OK")); }
            } else {
                _sendResponse(_json.createError("IN AUTO MODE"));
            }
        }
    }

    // 4. CẤU HÌNH AUTO
    // [FIX V7] Thay doc.containsKey("measures_per_day") -> doc["measures_per_day"].is<int>()
    if (doc["measures_per_day"].is<int>()) {
        int n = doc["measures_per_day"];
        _measuresPerDay = n;
        _calculateGridInterval();
        Serial.printf("-> Auto Grid: %d/day\n", n);
    }

    // [FIX V7] Thay doc.containsKey("set_time") -> doc["set_time"].is<const char*>()
    if (doc["set_time"].is<const char*>()) {
        String tStr = doc["set_time"].as<String>();
        _parseSetTime(tStr);
    }
}

// =================================================================================
// CÁC HÀM HELPER & PROCESS (GIỮ NGUYÊN CODE CŨ 100%)
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

    // Timeout logic
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
    _cmd.pushToQueue(packet); 
}

void StateMachine::setStopFlag(bool flag) {
    if (flag) _stopAndResetCycle();
}