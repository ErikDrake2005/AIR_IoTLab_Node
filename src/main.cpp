#include "StateMachine.h"
#include "Measurement.h"
#include "RelayController.h"
#include "UARTCommander.h"
#include "TimeSync.h"
#include <ArduinoJson.h>
#include "CRC32.h"
#include "Config.h"

const unsigned long TIME_STABILIZE = 60000;
const unsigned long TIME_MEASURE_1 = 180000;
const unsigned long TIME_MEASURE_2 = 480000;
const unsigned long TIME_MEASURE_3 = 900000;
const unsigned long TIMEOUT_CYCLE = 960000;
const unsigned long DEFAULT_MANUAL_INTERVAL = 300000;

StateMachine::StateMachine(Measurement& meas, RelayController& relay, UARTCommander& cmd, TimeSync& timeSync)
    : _meas(meas), _relay(relay), _cmd(cmd), _timeSync(timeSync) {
    _mode = MANUAL;
    _cycleState = STATE_IDLE;
    _measuresPerDay = DEFAULT_MEASURES_PER_DAY;
    _currentManualInterval = DEFAULT_MANUAL_INTERVAL;
    _lastTriggerMinute = -1;
    _calculateGridInterval();
    _schedules.clear();
}

void StateMachine::begin() {
    Serial.println("[SM] Started");
    _sendResponse(_json.createTimeSyncRequest());
}

void StateMachine::update() {
    if (_cycleState != STATE_IDLE) {
        _processCycleLogic();
        return;
    }
    if (_mode == AUTO) {
        unsigned long currentEpoch = _timeSync.getCurrentTime();
        if (currentEpoch > 100000) {
            bool trigger = false;
            if (_isTimeForScheduledMeasure()) trigger = true;
            else if (_isTimeForGridMeasure()) trigger = true;
            
            if (trigger) _startCycle();
        }
    }
}

void StateMachine::handleCommand(const String& cleanJson) {
    JsonDocument doc;
    if (deserializeJson(doc, cleanJson)) {
        _sendResponse(_json.createError("JSON_ERR"));
        return;
    }

    bool configChanged = false;
    bool ackSent = false;

    if (doc["timestamp"].is<unsigned long>()) {
        unsigned long ts = doc["timestamp"];
        if (ts > 0) {
            _timeSync.updateEpoch(ts);
            _lastTriggerMinute = -1;
            configChanged = true;
        }
    }
    if (doc["set_time"].is<const char*>()) {
        _parseSetTime(String((const char*)doc["set_time"]));
        configChanged = true;
    }
    if (doc["measures_per_day"].is<int>()) {
        _measuresPerDay = doc["measures_per_day"];
        _calculateGridInterval();
        configChanged = true;
    }
    // Giữ lại để tương thích ngược
    if (doc["cmd"] == "set_cycle" && doc["val"].is<int>()) {
        _measuresPerDay = doc["val"];
        _calculateGridInterval();
        configChanged = true;
    }
    if (doc["cycle_manual"].is<int>()) {
        int rawVal = doc["cycle_manual"];
        int mins = constrain(rawVal, 1, 60);
        unsigned long newInterval = (unsigned long)mins * 60000UL;
        if (_currentManualInterval != newInterval) {
            _currentManualInterval = newInterval;
            configChanged = true;
        }
    }

    if (doc["mode"].is<const char*>()) {
        const char* m = doc["mode"];
        if (strcmp(m, "manual") == 0 && _mode != MANUAL) {
            _mode = MANUAL;
            _stopAndResetCycle();
            configChanged = true;
        } else if (strcmp(m, "auto") == 0 && _mode != AUTO) {
            _mode = AUTO;
            _stopAndResetCycle();
            _lastTriggerMinute = -1;
            configChanged = true;
        }
    }

    const char* set_state = doc["set_state"];
    const char* cmd = doc["cmd"];
    bool isTrigger = (set_state && strcmp(set_state, "measure") == 0) || (cmd && strcmp(cmd, "trigger_measure") == 0);
    bool isStop = (set_state && strcmp(set_state, "stop") == 0) || (cmd && strcmp(cmd, "stop_measure") == 0);

    if (isStop) {
        _stopAndResetCycle();
        _sendResponse(_json.createAck("MEASURE_STOPPED"));
        ackSent = true;
    } else if (isTrigger) {
        if (_cycleState != STATE_IDLE && _cycleState != STATE_MANUAL_LOOP) {
            _sendResponse(_json.createError("BUSY"));
        } else {
            if (_mode != MANUAL) { _mode = MANUAL; configChanged = true; }
            _startManualLoop();
            _sendResponse(_json.createAck("MEASURE_STARTED"));
        }
        ackSent = true;
    }

    if (_mode == MANUAL) {
        const char* d = doc["set_door"];
        const char* f = doc["set_fans"];
        if (d) {
            if (strcmp(d, "open") == 0) { _relay.ON_DOOR(); _sendResponse(_json.createAck("DOOR_OPENED")); ackSent = true; }
            else if (strcmp(d, "close") == 0) { _relay.OFF_DOOR(); _sendResponse(_json.createAck("DOOR_CLOSED")); ackSent = true; }
        }
        if (f) {
            if (strcmp(f, "on") == 0) { _relay.ON_FAN(); _sendResponse(_json.createAck("FANS_ON")); ackSent = true; }
            else if (strcmp(f, "off") == 0) { _relay.OFF_FAN(); _sendResponse(_json.createAck("FANS_OFF")); ackSent = true; }
        }
    } else {
        if (!doc["set_door"].isNull() || !doc["set_fans"].isNull()) {
            _sendResponse(_json.createError("ERR_IN_AUTO"));
            ackSent = true;
        }
    }

    if (configChanged && !ackSent) {
        _sendResponse(_json.createAck("CONFIG_OK"));
    }
}

void StateMachine::_processCycleLogic() {
    unsigned long elapsed = millis() - _cycleStartMillis;

    if (elapsed > TIMEOUT_CYCLE && _cycleState != STATE_MANUAL_LOOP) {
        _finishCycle();
        return;
    }

    switch (_cycleState) {
        case STATE_PREPARING:
            _cycleState = STATE_STABILIZING;
            break;
        case STATE_STABILIZING:
            if (elapsed >= TIME_STABILIZE) _cycleState = STATE_WAIT_MEASURE_1;
            break;
        case STATE_WAIT_MEASURE_1:
            if (elapsed >= TIME_MEASURE_1) {
                _meas.doFullMeasurement(_miniData[0]);
                _cycleState = STATE_WAIT_MEASURE_2;
            }
            break;
        case STATE_WAIT_MEASURE_2:
            if (elapsed >= TIME_MEASURE_2) {
                _meas.doFullMeasurement(_miniData[1]);
                _cycleState = STATE_WAIT_MEASURE_3;
            }
            break;
        case STATE_WAIT_MEASURE_3:
            if (elapsed >= TIME_MEASURE_3) {
                _meas.doFullMeasurement(_miniData[2]);
                _cycleState = STATE_FINISHING;
            }
            break;
        case STATE_FINISHING:
            _finishCycle();
            break;
        case STATE_MANUAL_LOOP:
            if (millis() >= _nextManualMeasureMillis) {
                MeasurementData t;
                _meas.doFullMeasurement(t);
                _sendResponse(_json.createDataJson(t.ch4, t.co, t.alc, t.nh3, t.h2, t.temp, t.hum));
                _nextManualMeasureMillis = millis() + _currentManualInterval;
            }
            break;
    }
}

void StateMachine::_finishCycle() {
    float sums[7] = {0};
    int counts[7] = {0};
    for (int i = 0; i < 3; i++) {
        float vals[] = {_miniData[i].ch4, _miniData[i].co, _miniData[i].alc, _miniData[i].nh3, _miniData[i].h2, _miniData[i].temp, _miniData[i].hum};
        for (int k = 0; k < 7; k++) {
            if (vals[k] != -1) {
                sums[k] += vals[k];
                counts[k]++;
            }
        }
    }
    float avgs[7];
    for (int k = 0; k < 7; k++) avgs[k] = (counts[k] > 0) ? (sums[k] / counts[k]) : -1.0;
    
    // [DEBUG] In kết quả trung bình ra Serial trước khi gửi
    Serial.println("\n--- [SM] CYCLE RESULT ---");
    Serial.printf("CH4: %.2f | CO: %.2f | ALC: %.2f\n", avgs[0], avgs[1], avgs[2]);
    Serial.printf("NH3: %.2f | H2: %.2f | T/H: %.1f/%.1f\n", avgs[3], avgs[4], avgs[5], avgs[6]);
    Serial.println("-------------------------");

    _sendResponse(_json.createDataJson(avgs[0], avgs[1], avgs[2], avgs[3], avgs[4], avgs[5], avgs[6]));
    _stopAndResetCycle();
}

void StateMachine::_startCycle() {
    _cycleState = STATE_PREPARING;
    _cycleStartMillis = millis();
    
    // Khởi tạo -1 cho tất cả
    for(int i=0; i<3; i++) {
        _miniData[i].ch4 = -1;
        _miniData[i].co  = -1;
        _miniData[i].alc = -1;
        _miniData[i].nh3 = -1;
        _miniData[i].h2  = -1;
        _miniData[i].temp = -1;
        _miniData[i].hum  = -1;
    }
    _relay.OFF_DOOR();
    _relay.ON_FAN();
    Serial.println("[SM] Auto Cycle Started");
}

void StateMachine::_startManualLoop() {
    _cycleState = STATE_MANUAL_LOOP;
    _relay.OFF_DOOR();
    _relay.ON_FAN();
    _nextManualMeasureMillis = millis() + 1000;
}

void StateMachine::_stopAndResetCycle() {
    _cycleState = STATE_IDLE;
    _relay.OFF_FAN();
    _relay.ON_DOOR();
}

void StateMachine::_sendResponse(String json) {
    Serial.print("[SM] Queue: ");
    Serial.println(json); // Debug JSON string
    unsigned long crc = CRC32::calculate(json);
    _cmd.pushToQueue(json + "|" + String(crc, HEX));
}

void StateMachine::setStopFlag(bool flag) {
    if (flag) _stopAndResetCycle();
}

void StateMachine::_calculateGridInterval() {
    if (_measuresPerDay <= 0) _measuresPerDay = 1;
    _gridIntervalSeconds = SECONDS_IN_DAY / _measuresPerDay;
}

void StateMachine::_parseSetTime(String timeStr) {
    int h, m;
    if (sscanf(timeStr.c_str(), "%d:%d", &h, &m) == 2) {
        if (h < 0 || h > 23 || m < 0 || m > 59) return;
        
        // Kiểm tra trùng
        for (const auto& sch : _schedules) {
            if (sch.hour == h && sch.minute == m) return;
        }
        
        // Giới hạn 10 để an toàn RAM
        if (_schedules.size() >= 10) _schedules.erase(_schedules.begin());
        
        _schedules.push_back({h, m, 0});
        Serial.printf("[SM] Added schedule: %02d:%02d\n", h, m);
    }
}

bool StateMachine::_isTimeForGridMeasure() {
    unsigned long currentEpoch = _timeSync.getCurrentTime();
    return (currentEpoch > 100000) && ((currentEpoch % _gridIntervalSeconds) < 5);
}

bool StateMachine::_isTimeForScheduledMeasure() {
    unsigned long currentEpoch = _timeSync.getCurrentTime();
    time_t raw = (time_t)currentEpoch;
    struct tm* t = localtime(&raw);
    if (t->tm_min == _lastTriggerMinute) return false;
    for (const auto& s : _schedules) {
        if (t->tm_hour == s.hour && t->tm_min == s.minute) {
            _lastTriggerMinute = t->tm_min;
            return true;
        }
    }
    return false;
}
