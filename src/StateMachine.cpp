#include "StateMachine.h"
#include "esp_sleep.h"
#include "CRC32.h"

// --- CẤU HÌNH THỜI GIAN (ms) ---
const unsigned long T_PREPARE   = 10000;      // 10s chờ ổn định
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
    _status.isDoorOpen = true; // Mặc định mở
    _status.isFanOn = false;   // Mặc định tắt
    _status.saved_manual_cycle = 5; 
    _status.saved_daily_measures = 4;
    
    _cycleState = STATE_IDLE;
    _nextTimeSync = 0;
    _stopRequested = false;
    _lastTriggerMin = -1;
    
    _resetMiniData();
}

void StateMachine::begin() {
    #ifdef PIN_SLEEP_STATUS
    pinMode(PIN_SLEEP_STATUS, OUTPUT);
    digitalWrite(PIN_SLEEP_STATUS, HIGH); // HIGH = Đang thức
    #endif
    
    #ifdef PIN_WAKEUP_GPIO
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKEUP_GPIO, 1);
    #endif

    _recalcGrid();
    
    // Khởi động: Reset trạng thái về nghỉ (Mở cửa, Tắt quạt)
    _resetCycle();
    
    _sendMachineStatus();
    _requestTimeSync();
}

void StateMachine::update() {
    // 1. Kiểm tra đồng bộ giờ
    if (millis() > _nextTimeSync && _nextTimeSync != 0) {
        _requestTimeSync();
    }

    // 2. Kiểm tra tín hiệu dừng khẩn cấp
    if (_stopRequested) {
        _resetCycle();
        _stopRequested = false;
        return;
    }

    // 3. LOGIC ĐO: Phân chia rõ rệt AUTO và MANUAL
    
    // 3A. AUTO MODE: Chạy theo CycleState (Prepare -> Wait1 -> Wait2 -> Wait3)
    if (_status.mode == MODE_AUTO && _cycleState != STATE_IDLE) {
        _processAutoCycle();
        _handleLightSleep(); // Ngủ giữa các pha chờ
        return;
    }

    // 3B. MANUAL MODE: Trạng thái giữ nguyên (Persistent), không tự động chuyển State
    // Logic Manual được xử lý trực tiếp khi nhận lệnh hoặc theo Timer bên dưới

    // 4. LOGIC CHỜ (IDLE LOOP)
    unsigned long now = millis();

    if (_status.mode == MODE_MANUAL) {
        // Nếu đang bật máy (isMeasuring = true), kiểm tra chu kỳ để tự lấy mẫu thêm
        if (_status.isMeasuring) {
            if (now >= _nextManualRun) {
                _performManualMeasurement(); // Đo tiếp mà không thay đổi trạng thái cửa/quạt
                _nextManualRun = now + (_status.saved_manual_cycle * 60000UL);
            }
        }
    }
    else if (_status.mode == MODE_AUTO) {
        // Reset cờ UART Wakeup
        if (_isUartWakeup && now - _uartWakeupMs > T_UART_WAIT) {
            _isUartWakeup = false;
        }
        
        // Kiểm tra lịch/lưới để kích hoạt Auto Cycle
        if (_checkSchedule() || _checkGrid()) {
            _startAutoCycle();
            return;
        }
        
        _handleLightSleep();
    }
}

// ================= XỬ LÝ LỆNH (COMMAND PROCESSING) =================

void StateMachine::processRawCommand(String rawStr) {
    if (_isUartWakeup) _uartWakeupMs = millis(); 

    CommandData cmd = _cmdProcessor.parse(rawStr);
    
    if (cmd.isValid) {
        Serial.println("[SM] Valid CMD received");
        _applyCommand(cmd);
    }
}

void StateMachine::_applyCommand(CommandData& cmd) {
    bool changed = false;

    // --- SLEEP ---
    if (!cmd.enable && cmd.setMode == MODE_SLEEP) {
        _enterDeepSleep();
        return; 
    }

    // --- TIME SYNC ---
    if (cmd.setMode == MODE_TIMESTAMP) {
        if (cmd.timestamp > 0) {
            _timeSync.updateEpoch(cmd.timestamp);
            _nextTimeSync = millis() + T_SYNC; 
            _recalcGrid();
            changed = true;
        }
    }

    // --- CHANGE MODE ---
    if (cmd.setMode == MODE_AUTO) {
        if (_status.mode != MODE_AUTO) { 
            _status.mode = MODE_AUTO; 
            _resetCycle(); // Chuyển mode thì reset hết
            changed = true; 
        }
        if (cmd.autoMeasureCount > 0) {
            _status.saved_daily_measures = cmd.autoMeasureCount;
            _recalcGrid();
            changed = true;
        }
    } 
    else if (cmd.setMode == MODE_MANUAL) {
        if (_status.mode != MODE_MANUAL) { 
            _status.mode = MODE_MANUAL; 
            _resetCycle(); // Chuyển mode thì reset hết
            changed = true; 
        }
        if (cmd.manualInterval > 0) {
            _status.saved_manual_cycle = cmd.manualInterval;
            _nextManualRun = millis() + (_status.saved_manual_cycle * 60000UL);
            changed = true;
        }

        // --- MANUAL ACTIONS ---
        if (cmd.hasActions) {
            if (cmd.doorStatus == "open") _setDoor(true);
            else if (cmd.doorStatus == "close") _setDoor(false);

            if (cmd.fanStatus == "on") _setFan(true);
            else if (cmd.fanStatus == "off") _setFan(false);

            // [LOGIC MANUAL MỚI]
            // start/measurement -> Bật máy, Giữ nguyên, Đo
            if (cmd.chamberStatus == "measuring" || cmd.chamberStatus == "trigger_measure" || cmd.chamberStatus == "start") {
                _performManualMeasurement(); 
            }
            // stop -> Tắt máy, Về IDLE
            else if (cmd.chamberStatus == "stop" || cmd.chamberStatus == "stop_measure") {
                _stopRequested = true; // Sẽ kích hoạt _resetCycle ở vòng loop tới
            }
            
            changed = true;
        }
    }

    _sendMachineStatus();
}

// ================= LOGIC AUTO CYCLE (15 PHÚT) =================

void StateMachine::_startAutoCycle() {
    Serial.println("[AUTO] START 15m Cycle");
    _status.isMeasuring = true;
    _cycleState = STATE_PREPARE;
    _cycleStartMs = millis();
    
    _setDoor(false); // Đóng cửa
    _setFan(true);   // Bật quạt
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
        Serial.println("[AUTO] Aborted!");
        _resetCycle();
        return;
    }

    Serial.println("[AUTO] Finished. Calculating Average...");
    
    // Tính trung bình (Logic giữ nguyên)
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

    String jsonPkt = _jsonFormatter.createDataJson(avgTemp, avgHum, avgCH4, avgCO, avgAlc, avgNH3, avgH2);
    _sendPacket(jsonPkt);

    _resetCycle(); // Auto xong thì tắt máy
    _enterDeepSleep();
}

// ================= LOGIC MANUAL (SESSION BASED) =================

void StateMachine::_performManualMeasurement() {
    // 1. Kiểm tra nếu hệ thống chưa bật (Lần đo đầu tiên của phiên)
    if (!_status.isMeasuring) {
        Serial.println("[MANUAL] Starting Session: Close Door, Fan ON...");
        
        _status.isMeasuring = true; // Đánh dấu đang trong phiên
        _setDoor(false); // Đóng cửa
        _setFan(true);   // Bật quạt
        
        // Chờ khí ổn định
        delay(T_PREPARE); 
    } else {
        Serial.println("[MANUAL] System Active. Taking Snapshot...");
    }

    // 2. Đo 1 lần
    MeasurementData singleSample;
    if (_meas.doFullMeasurement(singleSample)) {
        String jsonPkt = _jsonFormatter.createDataJson(
            singleSample.temp, singleSample.hum, singleSample.ch4, 
            singleSample.co, singleSample.alc, singleSample.nh3, singleSample.h2
        );
        _sendPacket(jsonPkt);
        Serial.println("[MANUAL] Data Sent.");
    } else {
        Serial.println("[MANUAL] Measure Failed.");
    }

    // 3. QUAN TRỌNG: KHÔNG GỌI _resetCycle() Ở ĐÂY
    // Trạng thái (Đóng cửa, Bật quạt) sẽ giữ nguyên cho đến khi nhận lệnh STOP.
}

// ================= TRẠNG THÁI & RESET =================

void StateMachine::_resetCycle() {
    _cycleState = STATE_IDLE;
    _status.isMeasuring = false;
    
    // Trạng thái nghỉ: Mở cửa (thoáng khí), Tắt quạt
    _setFan(false);
    _setDoor(true); 
    
    _sendMachineStatus();
}

void StateMachine::_resetMiniData() {
    for(int i=0; i<3; i++) {
        _miniData[i].ch4 = -1.0f; _miniData[i].co = -1.0f;
        _miniData[i].alc = -1.0f; _miniData[i].nh3 = -1.0f;
        _miniData[i].h2 = -1.0f;  _miniData[i].temp = -1.0f;
        _miniData[i].hum = -1.0f;
    }
}

// ================= QUẢN LÝ NĂNG LƯỢNG =================

void StateMachine::_enterDeepSleep() {
    Serial.println("[PWR] Deep Sleep");
    _setDoor(true); // Mở cửa khi ngủ cho an toàn
    _setFan(false);
    
    _sendPacket(_jsonFormatter.createAck("SLEEP"));
    delay(500); 
    
    digitalWrite(PIN_SLEEP_STATUS, LOW);
    esp_deep_sleep_start();
}

void StateMachine::_handleLightSleep() {
    unsigned long sleepTime = _calcSleepTime();
    if (sleepTime < 5000) return;

    esp_sleep_enable_timer_wakeup((sleepTime - 2000) * 1000ULL); 
    esp_sleep_enable_uart_wakeup(UART_NUM_1); 
    
    digitalWrite(PIN_SLEEP_STATUS, LOW);
    esp_light_sleep_start();
    digitalWrite(PIN_SLEEP_STATUS, HIGH); 
    
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UART) {
        while(Serial1.available()) Serial1.read(); 
        _sendPacket(_jsonFormatter.createWakeupAck());
        _isUartWakeup = true;
        _uartWakeupMs = millis();
    }
}

unsigned long StateMachine::_calcSleepTime() {
    unsigned long now = _timeSync.getCurrentTime();
    unsigned long elapsed = millis() - _cycleStartMs;

    if (_cycleState != STATE_IDLE) {
        if (_cycleState == STATE_PREPARE) return T_MEASURE_1 - elapsed;
        if (_cycleState == STATE_WAIT_1)  return T_MEASURE_2 - elapsed;
        if (_cycleState == STATE_WAIT_2)  return T_MEASURE_3 - elapsed;
        return 0;
    }
    
    if (now < 100000) return 0;
    
    long minDiff = 86400; 
    long secToday = now % 86400;

    if (_gridInterval > 0) {
        long nextGrid = ((now / _gridInterval) + 1) * _gridInterval;
        long diff = nextGrid - now;
        if (diff < minDiff) minDiff = diff;
    }
    return minDiff * 1000UL;
}

// ================= HELPERS & UPLINK =================

void StateMachine::_recalcGrid() {
    if (_status.saved_daily_measures <= 0) _status.saved_daily_measures = 1;
    _gridInterval = 86400 / _status.saved_daily_measures;
}

bool StateMachine::_checkSchedule() { return false; } 

bool StateMachine::_checkGrid() {
    unsigned long now = _timeSync.getCurrentTime();
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
    unsigned long crc = CRC32::calculate(json);
    _cmd.pushToQueue(json + "|" + String(crc, HEX));
}

void StateMachine::_setDoor(bool open) {
    if(open) { _relay.ON_DOOR(); _status.isDoorOpen = true; }
    else     { _relay.OFF_DOOR(); _status.isDoorOpen = false; }
}
void StateMachine::_setFan(bool on) {
    if(on) { _relay.ON_FAN(); _status.isFanOn = true; }
    else   { _relay.OFF_FAN(); _status.isFanOn = false; }
}