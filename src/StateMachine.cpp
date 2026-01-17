#include "StateMachine.h"
#include "esp_sleep.h"
#include "CRC32.h"
#include "Config.h" // [Quan trọng] Để lấy định nghĩa PIN

// --- CẤU HÌNH THỜI GIAN (ms) ---
const unsigned long T_PREPARE   = 10000;      // 10s chờ ổn định khí
const unsigned long T_MEASURE_1 = 180000;     // 3 phút (Auto lần 1)
const unsigned long T_MEASURE_2 = 480000;     // 8 phút (Auto lần 2)
const unsigned long T_MEASURE_3 = 900000;     // 15 phút (Auto lần 3)
const unsigned long T_TIMEOUT   = 960000;     // Timeout an toàn
const unsigned long T_SYNC      = 3600000;    // 1 giờ sync 1 lần
const unsigned long T_UART_WAIT = 15000;      // 15s chờ sau khi UART đánh thức

StateMachine::StateMachine(Measurement& meas, RelayController& relay, UARTCommander& cmd, TimeSync& timeSync)
    : _meas(meas), _relay(relay), _cmd(cmd), _timeSync(timeSync) 
{
    // Khởi tạo trạng thái mặc định
    _status.mode = MODE_MANUAL;
    _status.isMeasuring = false;
    _status.isDoorOpen = true; // Mặc định mở (SAFE)
    _status.isFanOn = false;   // Mặc định tắt
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
    
    _resetMiniData();
}

void StateMachine::begin() {
    #ifdef PIN_SLEEP_STATUS
    pinMode(PIN_SLEEP_STATUS, OUTPUT);
    digitalWrite(PIN_SLEEP_STATUS, HIGH); // HIGH = Đang thức (Báo cho Bridge)
    #endif
    
    #ifdef PIN_WAKEUP_GPIO
    // Cấu hình đánh thức bằng chân GPIO 33 (khi Bridge kích HIGH)
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKEUP_GPIO, 1);
    #endif

    _recalcGrid();
    _resetCycle(); // Đưa máy về trạng thái an toàn (Mở cửa, Tắt quạt)
    _sendMachineStatus();
    _requestTimeSync(); // Hỏi giờ Gateway ngay khi khởi động
}

void StateMachine::update() {
    // 1. Kiểm tra đồng bộ giờ
    if (millis() > _nextTimeSync && _nextTimeSync != 0) {
        _requestTimeSync();
    }

    // 2. Kiểm tra tín hiệu dừng khẩn cấp (từ lệnh STOP)
    if (_stopRequested) {
        Serial.println("[STOP] Forced Stop Requested!");
        _resetCycle();
        _sendMachineStatus(); // Báo cáo ngay trạng thái dừng
        _stopRequested = false;
        return;
    }

    // 3A. AUTO MODE LOGIC
    if (_status.mode == MODE_AUTO && _cycleState != STATE_IDLE) {
        _processAutoCycle();
        _handleLightSleep(); // Ngủ nhẹ giữa các pha chờ của Auto
        return;
    }

    // 3B. MANUAL MODE & IDLE LOGIC
    unsigned long now = millis();

    if (_status.mode == MODE_MANUAL) {
        // Nếu đang trong phiên đo Manual (đã bật máy)
        if (_status.isMeasuring) {
            // Kiểm tra chu kỳ để tự lấy mẫu thêm (nếu user cài đặt)
            if (_status.saved_manual_cycle > 0 && now >= _nextManualRun) {
                _performManualMeasurement(); 
                _nextManualRun = now + (_status.saved_manual_cycle * 60000UL);
            }
        }
    }
    else if (_status.mode == MODE_AUTO) {
        // IDLE LOOP của Auto Mode: Chờ đến giờ hẹn
        
        // Reset cờ UART Wakeup sau 15s để hệ thống ổn định
        if (_isUartWakeup && now - _uartWakeupMs > T_UART_WAIT) {
            _isUartWakeup = false;
        }
        
        // Kiểm tra lịch hẹn (Start Time hoặc Grid)
        if ((_startTimeSeconds > 0 && _checkSchedule()) || (_checkGrid())) {
            _startAutoCycle();
            return;
        }
        
        _handleLightSleep(); // Ngủ tiết kiệm pin khi chờ
    }
}

// ================= XỬ LÝ LỆNH TỪ BRIDGE =================

void StateMachine::processRawCommand(String rawLine) {
    // 1. Parse lệnh
    CommandData cmd = _cmdProcessor.parse(rawLine);
    
    if (!cmd.isValid) {
        Serial.println("[CMD] Ignored (Invalid CRC/JSON)");
        return;
    }

    Serial.println("[CMD] Valid. Processing...");

    // 2. XỬ LÝ LỆNH NGỦ (ƯU TIÊN 1)
    if (cmd.setMode == MODE_SLEEP) {
        Serial.println("[CMD] Received SLEEP command.");
        _enterDeepSleep(); 
        return; 
    }

    // 3. XỬ LÝ ĐỒNG BỘ GIỜ (ƯU TIÊN 2)
    if (cmd.setMode == MODE_TIMESTAMP) {
        if (cmd.timestamp > 0) {
            _timeSync.updateEpoch(cmd.timestamp);
            _recalcGrid();
        }
    }

    // 4. XỬ LÝ CHUYỂN CHẾ ĐỘ (AUTO / MANUAL)
    if (cmd.setMode == MODE_AUTO) {
        if (_status.mode != MODE_AUTO) {
            Serial.println("[MODE] Switched to AUTO");
            _status.mode = MODE_AUTO;
            _resetCycle(); 
        }
        if (cmd.autoMeasureCount > 0) {
            _status.saved_daily_measures = cmd.autoMeasureCount;
            _recalcGrid();
        }
        if (cmd.startTimeSeconds > 0) _startTimeSeconds = cmd.startTimeSeconds;
    }
    else if (cmd.setMode == MODE_MANUAL) {
        if (_status.mode != MODE_MANUAL) {
            Serial.println("[MODE] Switched to MANUAL");
            _status.mode = MODE_MANUAL;
            _resetCycle();
        }
        if (cmd.manualInterval > 0) {
            _status.saved_manual_cycle = cmd.manualInterval;
            _nextManualRun = millis() + (_status.saved_manual_cycle * 60000UL);
        }
    }

    // 5. THỰC THI HÀNH ĐỘNG (Chỉ MANUAL Mode mới điều khiển Relay trực tiếp)
    // [CHECK QUAN TRỌNG]: Code này CÓ điều khiển phần cứng
    if (_status.mode == MODE_MANUAL && cmd.hasActions) {
        // Điều khiển Cửa
        if (cmd.doorStatus == "open")       _setDoor(true);
        else if (cmd.doorStatus == "close") _setDoor(false);

        // Điều khiển Quạt
        if (cmd.fanStatus == "on")          _setFan(true);
        else if (cmd.fanStatus == "off")    _setFan(false);

        // Điều khiển Đo
        if (!cmd.chamberStatus.isEmpty()) {
            if (cmd.chamberStatus == "stop" || cmd.chamberStatus == "stop-measurement") {
                _stopRequested = true; // Sẽ được xử lý ở update()
            } else {
                // start, trigger, measuring...
                _performManualMeasurement();
            }
        }
    }
    
    // Gửi phản hồi trạng thái mới nhất về Server
    _sendMachineStatus();
    Serial.println("[CMD] Done & Status Sent.");
}

// ================= LOGIC AUTO CYCLE (15 PHÚT) =================

void StateMachine::_startAutoCycle() {
    Serial.println("[AUTO] START 15m Cycle");
    _status.isMeasuring = true;
    _cycleState = STATE_PREPARE;
    _cycleStartMs = millis();
    
    _setDoor(false); // Đóng cửa (RELAY OFF_DOOR)
    _setFan(true);   // Bật quạt (RELAY ON_FAN)
    
    _sendMachineStatus(); // Báo trạng thái đang đo
    _resetMiniData();
}

void StateMachine::_processAutoCycle() {
    unsigned long elapsed = millis() - _cycleStartMs;
    
    if (elapsed > T_TIMEOUT) { _finishAutoCycle(true); return; }

    switch (_cycleState) {
        case STATE_PREPARE: // 10s đầu
            if (elapsed >= T_PREPARE) _cycleState = STATE_WAIT_1;
            break;
        case STATE_WAIT_1: // Phút thứ 3
            if (elapsed >= T_MEASURE_1) {
                Serial.println("[AUTO] Mini Measure 1");
                _meas.doFullMeasurement(_miniData[0]); 
                _cycleState = STATE_WAIT_2;
            } break;
        case STATE_WAIT_2: // Phút thứ 8
            if (elapsed >= T_MEASURE_2) {
                Serial.println("[AUTO] Mini Measure 2");
                _meas.doFullMeasurement(_miniData[1]); 
                _cycleState = STATE_WAIT_3;
            } break;
        case STATE_WAIT_3: // Phút thứ 15
            if (elapsed >= T_MEASURE_3) {
                Serial.println("[AUTO] Mini Measure 3");
                _meas.doFullMeasurement(_miniData[2]); 
                _finishAutoCycle(false); 
            } break;
    }
}

void StateMachine::_finishAutoCycle(bool aborted) {
    if (aborted) {
        Serial.println("[AUTO] Aborted!");
        _resetCycle();
        return;
    }

    Serial.println("[AUTO] Finished. Calc Average...");
    
    // Tính trung bình 3 lần đo
    float sumCH4 = 0, sumCO = 0, sumAlc = 0, sumNH3 = 0, sumH2 = 0, sumTemp = 0, sumHum = 0;
    int cntCH4 = 0, cntCO = 0, cntAlc = 0, cntNH3 = 0, cntH2 = 0, cntTemp = 0, cntHum = 0;

    for (int i = 0; i < 3; i++) {
        if (_miniData[i].ch4 != -1.0f) { sumCH4 += _miniData[i].ch4; cntCH4++; }
        if (_miniData[i].co  != -1.0f) { sumCO  += _miniData[i].co;  cntCO++; }
        if (_miniData[i].alc != -1.0f) { sumAlc += _miniData[i].alc; cntAlc++; }
        if (_miniData[i].nh3 != -1.0f) { sumNH3 += _miniData[i].nh3; cntNH3++; }
        if (_miniData[i].h2  != -1.0f) { sumH2  += _miniData[i].h2;  cntH2++; }
        if (_miniData[i].temp!= -1.0f) { sumTemp+= _miniData[i].temp;cntTemp++; }
        if (_miniData[i].hum != -1.0f) { sumHum += _miniData[i].hum; cntHum++; }
    }

    float avgCH4 = (cntCH4 > 0) ? (sumCH4 / cntCH4) : -1.0f;
    float avgCO  = (cntCO  > 0) ? (sumCO  / cntCO)  : -1.0f;
    float avgAlc = (cntAlc > 0) ? (sumAlc / cntAlc) : -1.0f;
    float avgNH3 = (cntNH3 > 0) ? (sumNH3 / cntNH3) : -1.0f;
    float avgH2  = (cntH2  > 0) ? (sumH2  / cntH2)  : -1.0f;
    float avgTemp= (cntTemp> 0) ? (sumTemp/ cntTemp): -1.0f;
    float avgHum = (cntHum > 0) ? (sumHum / cntHum) : -1.0f;

    // Gửi gói tin DATA
    String jsonPkt = _jsonFormatter.createDataJson(avgCH4, avgCO, avgAlc, avgNH3, avgH2, avgTemp, avgHum);
    _sendPacket(jsonPkt);

    // Xong cycle thì về nghỉ và NGỦ DEEP SLEEP để tiết kiệm pin tối đa
    _resetCycle(); 
    _enterDeepSleep();
}

// ================= LOGIC MANUAL (SNAPSHOT) =================

void StateMachine::_performManualMeasurement() {
    // Nếu chưa bật máy thì bật lên (Cửa đóng, Quạt mở)
    if (!_status.isMeasuring) {
        Serial.println("[MANUAL] Start Session: Close Door, Fan ON...");
        _status.isMeasuring = true; 
        
        _setDoor(false); // Đóng cửa
        _setFan(true);   // Bật quạt
        
        // Dùng vTaskDelay thay cho delay() để không treo hệ thống
        vTaskDelay(T_PREPARE / portTICK_PERIOD_MS); 
    } 

    Serial.println("[MANUAL] Measuring Snapshot...");
    
    // Đo 1 mẫu
    MeasurementData singleSample;
    if (_meas.doFullMeasurement(singleSample)) {
        String jsonPkt = _jsonFormatter.createDataJson(
            singleSample.ch4, singleSample.co, singleSample.alc, 
            singleSample.nh3, singleSample.h2, singleSample.temp, singleSample.hum
        );
        _sendPacket(jsonPkt);
        Serial.println("[MANUAL] Data Sent.");
    }

    // Lưu ý: Manual mode không tự tắt (Close/Off), user phải gửi lệnh STOP
}

// ================= RESET & UTILS =================

void StateMachine::_resetCycle() {
    _cycleState = STATE_IDLE;
    _status.isMeasuring = false;
    
    // Trạng thái nghỉ an toàn: Mở cửa, Tắt quạt
    _setFan(false);
    _setDoor(true); 
    
    // Reset thời gian chạy thủ công
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

// ================= HÀM ĐIỀU KHIỂN RELAY (WRAPPER) =================
// Đây là nơi kết nối giữa Logic và Phần Cứng

void StateMachine::_setDoor(bool open) {
    if(open) { 
        _relay.ON_DOOR();       // Gọi RelayController: Mở cửa
        _status.isDoorOpen = true; 
    }
    else { 
        _relay.OFF_DOOR();      // Gọi RelayController: Đóng cửa
        _status.isDoorOpen = false; 
    }
}

void StateMachine::_setFan(bool on) {
    if(on) { 
        _relay.ON_FAN();        // Gọi RelayController: Bật quạt
        _status.isFanOn = true; 
    }
    else { 
        _relay.OFF_FAN();       // Gọi RelayController: Tắt quạt
        _status.isFanOn = false; 
    }
}

// ================= SLEEP & POWER =================

void StateMachine::_enterDeepSleep() {
    Serial.println("[PWR] Entering Deep Sleep...");
    _resetCycle(); // Đảm bảo cửa mở, quạt tắt trước khi ngủ
    
    _sendPacket(_jsonFormatter.createAck("SLEEP"));
    vTaskDelay(500 / portTICK_PERIOD_MS); // Chờ gửi xong UART
    
    _timeSync.beforeDeepSleep();
    
    #ifdef PIN_SLEEP_STATUS
    digitalWrite(PIN_SLEEP_STATUS, LOW); // Hạ cờ Status -> Bridge biết Node ngủ
    #endif
    
    esp_deep_sleep_start();
}

void StateMachine::_handleLightSleep() {
    unsigned long sleepTime = _calcSleepTime();
    if (sleepTime < 5000) return; 

    Serial.printf("[PWR] Light Sleep for %lu ms\n", sleepTime);
    
    // Config đánh thức
    esp_sleep_enable_timer_wakeup((sleepTime - 2000) * 1000ULL); 
    esp_sleep_enable_uart_wakeup(UART_NUM_1); 
    
    #ifdef PIN_SLEEP_STATUS
    digitalWrite(PIN_SLEEP_STATUS, LOW);
    #endif
    
    esp_light_sleep_start();
    
    #ifdef PIN_SLEEP_STATUS
    digitalWrite(PIN_SLEEP_STATUS, HIGH); // Tỉnh dậy
    #endif
    
    _timeSync.afterWakeup(); 
    
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UART) {
        while(Serial1.available()) Serial1.read(); // Xả rác UART
        _isUartWakeup = true;
        _uartWakeupMs = millis();
    }
}

unsigned long StateMachine::_calcSleepTime() {
    unsigned long now = _timeSync.getCurrentTime();
    unsigned long elapsed = millis() - _cycleStartMs;

    // 1. Nếu đang trong chu trình đo Auto: Ngủ theo pha
    if (_cycleState != STATE_IDLE) {
        if (_cycleState == STATE_PREPARE) return (T_MEASURE_1 > elapsed) ? (T_MEASURE_1 - elapsed) : 0;
        if (_cycleState == STATE_WAIT_1)  return (T_MEASURE_2 > elapsed) ? (T_MEASURE_2 - elapsed) : 0;
        if (_cycleState == STATE_WAIT_2)  return (T_MEASURE_3 > elapsed) ? (T_MEASURE_3 - elapsed) : 0;
        return 0;
    }
    
    // 2. Nếu đang IDLE (chưa có thời gian thực): Không ngủ
    if (now < 100000) return 0;
    
    long minDiff = 86400; // Mặc định 1 ngày
    
    // 3. Tính thời gian đến GRID tiếp theo
    if (_gridInterval > 0) {
        long nextGrid = ((now / _gridInterval) + 1) * _gridInterval;
        long diff = nextGrid - now;
        if (diff < minDiff) minDiff = diff;
    }

    // 4. Tính thời gian đến START TIME (Hẹn giờ)
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
    unsigned long secondsInDay = now % 86400;
    
    // Nếu trùng giờ hẹn (sai số < 5s)
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
    // Kiểm tra nếu thời gian hiện tại chia hết cho Grid (sai số < 5s)
    return (now > 100000) && ((now % _gridInterval) < 5);
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