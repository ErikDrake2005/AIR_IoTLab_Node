#include "StateMachine.h"
#include "Measurement.h"
#include "RelayController.h"
#include "UARTCommander.h"
#include "TimeSync.h"
#include <ArduinoJson.h>
#include "CRC32.h"
#include "Config.h"
#include "esp_sleep.h"

const unsigned long TIME_STABILIZE = 60000;
const unsigned long TIME_MEASURE_1 = 180000;
const unsigned long TIME_MEASURE_2 = 480000;
const unsigned long TIME_MEASURE_3 = 900000;
const unsigned long TIMEOUT_CYCLE = 960000;
const unsigned long DEFAULT_MANUAL_INTERVAL = 300000;
const unsigned long UART_WAIT_TIMEOUT = 15000;

StateMachine::StateMachine(Measurement& meas, RelayController& relay, UARTCommander& cmd, TimeSync& timeSync)
    : _meas(meas), _relay(relay), _cmd(cmd), _timeSync(timeSync) {
    _mode = MANUAL;
    _cycleState = STATE_IDLE;
    _measuresPerDay = DEFAULT_MEASURES_PER_DAY;
    _currentManualInterval = DEFAULT_MANUAL_INTERVAL;
    _lastTriggerMinute = -1;
    _calculateGridInterval();
    _schedules.clear();
    _uartWakeupMillis = 0;
    _isUartWakeupActive = false;
}

void StateMachine::begin() {
    pinMode(PIN_SLEEP_STATUS, OUTPUT);
    _setBusyPin(true);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKEUP_GPIO, 1);

    Serial.println("[SM] Started");
    _stopAndResetCycle();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("[BOOT] Wakeup DeepSleep GPIO33");
        JsonDocument doc; doc["WakeUp"] = "Done";
        String out; serializeJson(doc, out);
        _sendResponse(out);
    } else if (cause == ESP_SLEEP_WAKEUP_UART) {
        Serial.println("[BOOT] Wakeup LightSleep UART");
    }

    JsonDocument ackDoc;
    ackDoc["type"] = "ack";
    ackDoc["cmd"] = "SYSTEM_READY";
    ackDoc["timestamp"] = _timeSync.getCurrentTime(); 
    String ackJson; serializeJson(ackDoc, ackJson);
    _sendResponse(ackJson);
    delay(50); 
    _sendResponse(_json.createTimeSyncRequest());
}

void StateMachine::update() {
    // 1. Xử lý Cycle Logic
    if (_cycleState != STATE_IDLE) {
        _processCycleLogic();
        // Nếu đang trong Auto Cycle và ở các bước chờ (Wait), thử ngủ
        if (_mode == AUTO && (_cycleState == STATE_WAIT_MEASURE_1 || 
                              _cycleState == STATE_WAIT_MEASURE_2 || 
                              _cycleState == STATE_WAIT_MEASURE_3)) {
            _tryLightSleep();
        }
        return;
    }

    // 2. Nếu IDLE (Rảnh rỗi)
    if (_mode == AUTO) {
        // Nếu vừa bị đánh thức bởi UART, chờ 15s lệnh
        if (_isUartWakeupActive) {
            if (millis() - _uartWakeupMillis > UART_WAIT_TIMEOUT) {
                _isUartWakeupActive = false; // Hết 15s, cho phép ngủ lại
            } else {
                return; // Đang chờ lệnh, không ngủ, không check schedule
            }
        }

        // Check lịch đo
        unsigned long currentEpoch = _timeSync.getCurrentTime();
        if (currentEpoch > 100000) {
            bool trigger = false;
            if (_isTimeForScheduledMeasure()) trigger = true;
            else if (_isTimeForGridMeasure()) trigger = true;
            
            if (trigger) {
                _startCycle();
                return;
            }
        }
        
        // Không có việc gì -> Ngủ
        _tryLightSleep();
    }
}

void StateMachine::handleCommand(const String& cleanJson) {
    // Reset timer chờ ngủ nếu có lệnh bất kỳ
    if (_isUartWakeupActive) _uartWakeupMillis = millis();

    JsonDocument doc;
    if (deserializeJson(doc, cleanJson)) {
        _sendResponse(_json.createError("JSON_ERR"));
        return;
    }

    // [REQ 1] DEEP SLEEP COMMAND (Đã sửa lỗi containsKey cho ArduinoJson v7)
    if (doc["EN"].is<int>()) {
        int en = doc["EN"];
        if (en == 0) {
            _handleDeepSleepSequence();
            return;
        }
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
        JsonDocument ackDoc;
        ackDoc["type"] = "ack";
        ackDoc["timestamp"] = _timeSync.getCurrentTime();
        
        if (strcmp(m, "manual") == 0) {
            _mode = MANUAL; 
            _stopAndResetCycle();
            ackDoc["cmd"] = "MODE_MANUAL";
        } 
        else if (strcmp(m, "auto") == 0) {
            _mode = AUTO;
            _stopAndResetCycle();
            _lastTriggerMinute = -1;
            ackDoc["cmd"] = "MODE_AUTO";
        }
        String out; serializeJson(ackDoc, out);
        _sendResponse(out);
        ackSent = true;
    }
    const char* set_state = doc["set_state"];
    const char* cmd = doc["cmd"];
    bool isTrigger = (set_state && strcmp(set_state, "measure") == 0) || (cmd && strcmp(cmd, "trigger_measure") == 0);
    bool isStop = (set_state && strcmp(set_state, "stop") == 0) || (cmd && strcmp(cmd, "stop_measure") == 0);

    if (isStop) {
        _stopAndResetCycle();
        JsonDocument ackDoc;
        ackDoc["type"] = "ack";
        ackDoc["cmd"] = "MEASURE_STOPPED";
        ackDoc["timestamp"] = _timeSync.getCurrentTime();
        String out; serializeJson(ackDoc, out);
        _sendResponse(out);
        ackSent = true;
    } 
    else if (isTrigger) {
        if (_cycleState != STATE_IDLE && _cycleState != STATE_MANUAL_LOOP) {
            _sendResponse(_json.createError("BUSY"));
        } else {
            // Manual Trigger -> Force Manual Mode
            _mode = MANUAL; 
            _startManualLoop();
            JsonDocument ackDoc;
            ackDoc["type"] = "ack";
            ackDoc["cmd"] = "MEASURE_STARTED";
            ackDoc["timestamp"] = _timeSync.getCurrentTime();
            String out; serializeJson(ackDoc, out);
            _sendResponse(out);
        }
        ackSent = true;
    }
    if (_mode == MANUAL) {
        const char* d = doc["set_door"];
        const char* f = doc["set_fans"];
        if (d || f) {
            JsonDocument ackDoc;
            ackDoc["type"] = "ack";
            ackDoc["timestamp"] = _timeSync.getCurrentTime();
            bool actionTaken = false;

            if (d) {
                if (strcmp(d, "open") == 0) { 
                    _relay.ON_DOOR(); ackDoc["cmd"] = "DOOR_OPENED"; actionTaken = true; 
                } else if (strcmp(d, "close") == 0) { 
                    _relay.OFF_DOOR(); ackDoc["cmd"] = "DOOR_CLOSED"; actionTaken = true; 
                }
            }
            if (f) {
                if (strcmp(f, "on") == 0) { 
                    _relay.ON_FAN(); ackDoc["cmd"] = "FANS_ON"; actionTaken = true; 
                } else if (strcmp(f, "off") == 0) { 
                    _relay.OFF_FAN(); ackDoc["cmd"] = "FANS_OFF"; actionTaken = true; 
                }
            }
            if (actionTaken) {
                String out; serializeJson(ackDoc, out);
                _sendResponse(out);
                ackSent = true;
            }
        }
    } 
    if (configChanged && !ackSent) {
        JsonDocument ackDoc;
        ackDoc["type"] = "ack";
        ackDoc["cmd"] = "CONFIG_OK";
        ackDoc["timestamp"] = _timeSync.getCurrentTime();
        String out; serializeJson(ackDoc, out);
        _sendResponse(out);
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
             // Đã delay 10s ở startCycle, giờ chuyển luôn
            _cycleState = STATE_WAIT_MEASURE_1;
            break;
        case STATE_STABILIZING:
            // Giữ lại case này để tương thích enum, nhưng logic 15p dùng WAIT_MEASURE
            if (elapsed >= TIME_STABILIZE) _cycleState = STATE_WAIT_MEASURE_1;
            break;
        case STATE_WAIT_MEASURE_1:
            if (elapsed >= TIME_MEASURE_1) {
                _setBusyPin(true); // Đảm bảo thức để đo
                _meas.doFullMeasurement(_miniData[0]);
                _cycleState = STATE_WAIT_MEASURE_2;
            }
            break;
        case STATE_WAIT_MEASURE_2:
            if (elapsed >= TIME_MEASURE_2) {
                _setBusyPin(true);
                _meas.doFullMeasurement(_miniData[1]);
                _cycleState = STATE_WAIT_MEASURE_3;
            }
            break;
        case STATE_WAIT_MEASURE_3:
            if (elapsed >= TIME_MEASURE_3) {
                _setBusyPin(true);
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

void StateMachine::_startCycle() {
    _cycleState = STATE_PREPARING;
    
    // [REQ] 10s delay logic
    _relay.OFF_DOOR();
    _relay.ON_FAN();
    Serial.println("[SM] Closing door & Waiting 10s...");
    delay(10000); // Ổn định cơ khí
    
    // Reset thời gian SAU KHI delay xong
    _cycleStartMillis = millis();
    
    for(int i=0; i<3; i++) {
        _miniData[i].ch4 = -1; _miniData[i].co = -1; _miniData[i].alc = -1;
        _miniData[i].nh3 = -1; _miniData[i].h2 = -1; _miniData[i].temp = -1; _miniData[i].hum = -1;
    }
    Serial.println("[SM] Auto Cycle Started (Mini-Cycle)");
}

void StateMachine::_tryLightSleep() {
    unsigned long timeToSleep = 0;
    unsigned long now = millis();
    unsigned long elapsed = now - _cycleStartMillis;

    // 1. Tính toán thời gian ngủ dự kiến
    if (_cycleState == STATE_IDLE) {
        timeToSleep = _calculateNextWakeTime();
    } 
    else if (_cycleState == STATE_WAIT_MEASURE_1) {
        if (TIME_MEASURE_1 > elapsed) timeToSleep = TIME_MEASURE_1 - elapsed;
    }
    else if (_cycleState == STATE_WAIT_MEASURE_2) {
        if (TIME_MEASURE_2 > elapsed) timeToSleep = TIME_MEASURE_2 - elapsed;
    }
    else if (_cycleState == STATE_WAIT_MEASURE_3) {
        if (TIME_MEASURE_3 > elapsed) timeToSleep = TIME_MEASURE_3 - elapsed;
    }

    // 2. Kiểm tra điều kiện ngủ
    // Trừ hao 2s để thức dậy sớm chuẩn bị
    if (timeToSleep > 5000) {
        unsigned long sleepDuration = timeToSleep - 2000;
        
        Serial.printf("[PWR] Sleeping for %lu ms\n", sleepDuration);
        Serial.flush();

        // Config Wakeup
        esp_sleep_enable_timer_wakeup(sleepDuration * 1000ULL);
        esp_sleep_enable_uart_wakeup(1); // UART1 (Serial1)

        // Báo trạng thái ngủ cho Bridge
        _setBusyPin(false);
        
        // --- NGỦ ---
        esp_light_sleep_start();
        
        // --- TỈNH DẬY ---
        _setBusyPin(true);

        esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
        if (reason == ESP_SLEEP_WAKEUP_UART) {
            Serial.println("[PWR] Woke by UART");
            while(Serial1.available()) Serial1.read(); // Xóa dummy byte
            
            JsonDocument doc; doc["WakeUp"] = "Done";
            String out; serializeJson(doc, out);
            _sendResponse(out);

            _isUartWakeupActive = true;
            _uartWakeupMillis = millis();
        }
    }
}

void StateMachine::_handleDeepSleepSequence() {
    Serial.println("[PWR] DeepSleep Seq Start");
    _relay.OFF_DOOR();
    delay(10000); // Wait 10s
    
    JsonDocument ackDoc;
    ackDoc["type"] = "ack"; ackDoc["cmd"] = "EN:0";
    String out; serializeJson(ackDoc, out);
    _sendResponse(out);
    Serial.flush();
    
    _setBusyPin(false);
    esp_deep_sleep_start();
}

unsigned long StateMachine::_calculateNextWakeTime() {
    unsigned long nowEpoch = _timeSync.getCurrentTime();
    if (nowEpoch < 100000) return 0;

    unsigned long minWait = 2147483647;
    bool found = false;

    // Check Grid
    if (_gridIntervalSeconds > 0) {
        unsigned long currentBlock = nowEpoch / _gridIntervalSeconds;
        unsigned long nextGrid = (currentBlock + 1) * _gridIntervalSeconds;
        long wait = nextGrid - nowEpoch;
        if (wait > 0 && wait < minWait) { minWait = wait; found = true; }
    }

    // Check Schedule
    long secondsToday = nowEpoch % 86400;
    for (const auto& sch : _schedules) {
        long schSeconds = sch.hour * 3600 + sch.minute * 60;
        long wait = 0;
        if (schSeconds > secondsToday) wait = schSeconds - secondsToday;
        else wait = (86400 - secondsToday) + schSeconds;
        
        if (wait > 0 && wait < minWait) { minWait = wait; found = true; }
    }

    if (found) return minWait * 1000UL;
    return 0;
}

void StateMachine::_setBusyPin(bool isBusy) {
    digitalWrite(PIN_SLEEP_STATUS, isBusy ? HIGH : LOW);
}

void StateMachine::_finishCycle() {
    float sums[7] = {0};
    int counts[7] = {0};
    for (int i = 0; i < 3; i++) {
        float vals[] = {_miniData[i].ch4, _miniData[i].co, _miniData[i].alc, _miniData[i].nh3, _miniData[i].h2, _miniData[i].temp, _miniData[i].hum};
        for (int k = 0; k < 7; k++) {
            if (vals[k] > -99) { // Check valid logic (-1 or >0)
                sums[k] += vals[k];
                counts[k]++;
            }
        }
    }
    float avgs[7];
    for (int k = 0; k < 7; k++) avgs[k] = (counts[k] > 0) ? (sums[k] / counts[k]) : -1.0;
    
    Serial.println("\n--- [SM] CYCLE RESULT ---");
    _sendResponse(_json.createDataJson(avgs[0], avgs[1], avgs[2], avgs[3], avgs[4], avgs[5], avgs[6]));
    _stopAndResetCycle();
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
    _setBusyPin(true); // Luôn thức khi IDLE (trừ khi Auto mode quyết định ngủ)
}

void StateMachine::_sendResponse(String json) {
    Serial.print("[SM] Queue: "); Serial.println(json); 
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
        for (const auto& sch : _schedules) {
            if (sch.hour == h && sch.minute == m) return;
        }
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