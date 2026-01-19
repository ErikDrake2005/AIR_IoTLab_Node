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
    _lastCycleStartSeconds = 0;  // Chưa có cycle nào
    
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
    
    // Đưa máy về trạng thái an toàn (Mở cửa, Tắt quạt)
    Serial.println("[BOOT] Resetting to safe state...");
    _resetCycle();
    
    // Chờ một chút để UART Commander Task sẵn sàng trước khi gửi
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    // Gửi machine_status sau khi khởi động
    Serial.println("[BOOT] Sending initial machine_status...");
    _sendMachineStatus();
    
    // Hỏi giờ Gateway
    Serial.println("[BOOT] Requesting time sync...");
    _requestTimeSync();
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
        // _handleLightSleep(); // [DISABLED] Không ngủ nhẹ nữa
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
        
        // Delay nhỏ để không busy-wait (có thể điều chỉnh 10-100ms)
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ================= XỬ LÝ LỆNH TỪ BRIDGE =================

void StateMachine::processRawCommand(String rawLine) {
    CommandData cmd = _cmdProcessor.parse(rawLine);
    
    // Nếu gói tin không hợp lệ -> bỏ qua
    if (!cmd.isValid) {
        Serial.println("[CMD] Invalid command received, ignoring...");
        return;
    }

    // Biến cờ để theo dõi có thay đổi gì không (debug)
    bool hasChanges = false;

    // 1. XỬ LÝ LỆNH NGỦ (SLEEP)
    if (cmd.setMode == MODE_SLEEP) {
        Serial.println("[CMD] SLEEP command received");
        _sendMachineStatus(); // Gửi trạng thái trước khi ngủ
        _enterDeepSleep(); 
        return; 
    }

    // 2. XỬ LÝ ĐỒNG BỘ THỜI GIAN (TIMESTAMP)
    if (cmd.setMode == MODE_TIMESTAMP) {
        if (cmd.timestamp > 0) {
            Serial.printf("[CMD] Time sync: %lu\n", cmd.timestamp);
            _timeSync.updateEpoch(cmd.timestamp);
            _recalcGrid();
            hasChanges = true;
        }
        // Gửi machine_status phản hồi
        _sendMachineStatus();
        return;
    }

    // 3. XỬ LÝ CHUYỂN CHẾ ĐỘ (AUTO / MANUAL)
    if (cmd.setMode == MODE_AUTO) {
        if (_status.mode != MODE_AUTO) {
            Serial.println("[MODE] Switched to AUTO");
            _status.mode = MODE_AUTO;
            _resetCycle(); 
            hasChanges = true;
        }
        // Cập nhật measurementCount nếu có (không null và > 0)
        if (cmd.autoMeasureCount > 0) {
            _status.saved_daily_measures = cmd.autoMeasureCount;
            _recalcGrid();
            hasChanges = true;
        }
        // Cập nhật startTime nếu có
        if (cmd.startTimeSeconds > 0) {
            _startTimeSeconds = cmd.startTimeSeconds;
            hasChanges = true;
        }
    }
    else if (cmd.setMode == MODE_MANUAL) {
        if (_status.mode != MODE_MANUAL) {
            Serial.println("[MODE] Switched to MANUAL");
            // Nếu đang trong auto cycle, dừng lại
            if (_cycleState != STATE_IDLE) {
                _stopRequested = true;
            }
            _status.mode = MODE_MANUAL;
            _resetCycle();
            hasChanges = true;
        }
        // Cập nhật transmissionIntervalMinutes nếu có (không null và > 0)
        if (cmd.manualInterval > 0) {
            _status.saved_manual_cycle = cmd.manualInterval;
            _nextManualRun = millis() + (_status.saved_manual_cycle * 60000UL);
            hasChanges = true;
        }
    }

    // 4. XỬ LÝ CÁC HÀNH ĐỘNG (trong khóa "do")
    if (cmd.hasActions) {
        // [QUAN TRỌNG] Xử lý chamberStatus TRƯỚC - vì nó đã điều khiển cả cửa và quạt
        // Nếu có chamberStatus thì bỏ qua doorStatus và fanStatus
        if (!cmd.chamberStatus.isEmpty()) {
            if (cmd.chamberStatus == "stop" || cmd.chamberStatus == "stop-measurement") {
                Serial.println("[CMD] Stop measurement requested");
                // Thực hiện dừng ngay lập tức, không đợi update() loop
                _resetCycle();  // Đã bao gồm: Mở cửa, Tắt quạt
                Serial.println("[CMD] Measurement stopped, sending ACK...");
                _sendMachineStatus();
                return; // Đã gửi status, không cần gửi lại ở cuối
            } else if (cmd.chamberStatus == "start" || cmd.chamberStatus == "start-measurement") {
                Serial.println("[CMD] Start measurement requested");
                if (_status.mode == MODE_MANUAL) {
                    _performManualMeasurement();  // Đã bao gồm: Đóng cửa, Bật quạt
                    // Đã gửi machine_status bên trong, không cần gửi lại ở cuối
                    return;
                }
                hasChanges = true;
            }
        }
        else {
            // CHỈ xử lý doorStatus và fanStatus khi KHÔNG CÓ chamberStatus
            // Xử lý doorStatus
            if (!cmd.doorStatus.isEmpty()) {
                if (cmd.doorStatus == "open") {
                    _setDoor(true);
                    hasChanges = true;
                } else if (cmd.doorStatus == "close") {
                    _setDoor(false);
                    hasChanges = true;
                }
            }

            // Xử lý fanStatus
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
    
    // 5. LUÔN GỬI MACHINE_STATUS SAU KHI XỬ LÝ LỆNH (kể cả không có thay đổi)
    Serial.printf("[CMD] Command processed, hasChanges=%d, sending machine_status\n", hasChanges);
    _sendMachineStatus();
}

// ================= LOGIC AUTO CYCLE (15 PHÚT) =================

void StateMachine::_startAutoCycle() {
    _status.isMeasuring = true;
    _cycleState = STATE_PREPARE;
    _cycleStartMs = millis();
    
    // [QUAN TRỌNG] Lưu thời điểm bắt đầu để tránh đụng lịch
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
                _meas.doFullMeasurement(_miniData[0]); 
                _cycleState = STATE_WAIT_2;
            } break;
        case STATE_WAIT_2:
            if (elapsed >= T_MEASURE_2) {
                _meas.doFullMeasurement(_miniData[1]); 
                _cycleState = STATE_WAIT_3;
            } break;
        case STATE_WAIT_3:
            if (elapsed >= T_MEASURE_3) {
                _meas.doFullMeasurement(_miniData[2]); 
                _finishAutoCycle(false); 
            } break;
    }
}

void StateMachine::_finishAutoCycle(bool aborted) {
    if (aborted) {
        _resetCycle();
        return;
    }
    
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

    // Gửi dữ liệu trung bình sau khi đo xong 3 lần
    String jsonPkt = _jsonFormatter.createDataJson(avgCH4, avgCO, avgAlc, avgNH3, avgH2, avgTemp, avgHum);
    _sendPacket(jsonPkt);
    Serial.println("[AUTO] Mini-cycle finished, average data sent.");

    // Reset và gửi machine_status - KHÔNG deep sleep
    _resetCycle(); 
    _sendMachineStatus();
}

void StateMachine::_performManualMeasurement() {
    if (!_status.isMeasuring) {
        _status.isMeasuring = true; 
        _setDoor(false);
        _setFan(true);
        
        // GỬI MACHINE_STATUS NGAY SAU KHI ĐỔI TRẠNG THÁI
        Serial.println("[MANUAL] Status changed, sending machine_status before measure...");
        _sendMachineStatus();
        
        // Chờ khí ổn định (10s) - chia nhỏ để check abort và lệnh khẩn cấp
        Serial.println("[MANUAL] Waiting for gas stabilization...");
        unsigned long prepareEnd = millis() + T_PREPARE;
        while (millis() < prepareEnd) {
            // Check lệnh khẩn cấp từ UART mỗi 500ms
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
    
    // Kiểm tra abort trước khi đo
    if (_stopRequested) {
        _resetCycle();
        _sendMachineStatus();
        return;
    }
    
    MeasurementData singleSample;
    // Truyền _stopRequested và callback để measurement có thể abort giữa chừng
    // Lambda capture this để có thể gọi _checkUrgentCommands()
    auto urgentCheck = [this]() { this->_checkUrgentCommands(); };
    
    if (_meas.doFullMeasurement(singleSample, &_stopRequested, urgentCheck)) {
        // Kiểm tra lại sau khi đo xong
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
    
    // [COOLDOWN] Nếu vừa mới đo xong trong 15 phút -> bỏ qua
    if (_lastCycleStartSeconds > 0 && (now - _lastCycleStartSeconds) < CYCLE_COOLDOWN) {
        return false;  // Vẫn trong cooldown, không trigger
    }
    
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
    if (now < 100000) return false;
    
    // [COOLDOWN] Nếu vừa mới đo xong trong 15 phút -> bỏ qua
    if (_lastCycleStartSeconds > 0 && (now - _lastCycleStartSeconds) < CYCLE_COOLDOWN) {
        return false;  // Vẫn trong cooldown, không trigger
    }
    
    // Kiểm tra nếu thời gian hiện tại chia hết cho Grid (sai số < 5s)
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

// ================= CHECK LỆNH KHẨN CẤP (TRONG QUÁ TRÌNH ĐO) =================
void StateMachine::_checkUrgentCommands() {
    // Kiểm tra xem có lệnh nào trong queue không
    if (_cmd.hasCommand()) {
        String rawCmd = _cmd.getCommand();
        Serial.printf("[URGENT] Received command during measurement: %s\n", rawCmd.c_str());
        
        // Parse nhanh để check có phải lệnh stop không
        CommandData cmd = _cmdProcessor.parse(rawCmd);
        
        if (cmd.isValid) {
            // Check lệnh stop
            if (!cmd.chamberStatus.isEmpty() && 
                (cmd.chamberStatus == "stop" || cmd.chamberStatus == "stop-measurement")) {
                Serial.println("[URGENT] STOP command detected!");
                _stopRequested = true;
            }
            // Check lệnh sleep
            else if (cmd.setMode == MODE_SLEEP) {
                Serial.println("[URGENT] SLEEP command detected!");
                _stopRequested = true;
            }
            // Các lệnh khác (door, fan, mode change) - xử lý ngay
            else {
                // Xử lý door/fan ngay
                if (!cmd.doorStatus.isEmpty()) {
                    if (cmd.doorStatus == "open") _setDoor(true);
                    else if (cmd.doorStatus == "close") _setDoor(false);
                    _sendMachineStatus();
                }
                if (!cmd.fanStatus.isEmpty()) {
                    if (cmd.fanStatus == "on") _setFan(true);
                    else if (cmd.fanStatus == "off") _setFan(false);
                    _sendMachineStatus();
                }
            }
        }
    }
}