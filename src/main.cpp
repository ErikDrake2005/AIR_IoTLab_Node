#include <Arduino.h>
#include "StateMachine.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h" 
#include "Config.h"
#include "JsonFormatter.h"
#include "TimeSync.h"
#include "RelayController.h"
#include "Measurement.h"
#include "UARTCommander.h"
#include "RS485Master.h"
#include "SHT31Sensor.h"
#include "CRC32.h" 

// --- KHỞI TẠO ĐỐI TƯỢNG TOÀN CỤC ---
SemaphoreHandle_t sysMutex;
HardwareSerial rs485Serial(2);
HardwareSerial commandSerial(1);

RS485Master rs485(rs485Serial);
SHT31Sensor sht31;
RelayController relay;
JsonFormatter jsonFormatter;
Measurement measurement(rs485, sht31, jsonFormatter);

UARTCommander uartCommander; 
TimeSync timeSync(jsonFormatter);
StateMachine stateMachine(measurement, relay, uartCommander, timeSync);

// --- TASK 1: NHẬN LỆNH TỪ UART (Ưu tiên cao hơn Logic) ---
void uartRxTask(void* pvParameters) {
    // Tăng buffer cục bộ để xử lý chuỗi
    static char rxBuffer[2048]; 
    static int rxIndex = 0;

    // Xóa rác buffer UART phần cứng khi khởi động
    while (commandSerial.available()) commandSerial.read();

    Serial.println("[UART Task] Ready to receive commands...");

    for (;;) {
        // Đọc dữ liệu đến
        while (commandSerial.available()) {
            char c = commandSerial.read();
            
            // Tìm ký tự kết thúc dòng (xuống dòng \n)
            if (c == '\n') {
                rxBuffer[rxIndex] = 0; // Kết thúc chuỗi NULL
                String jsonCmd = String(rxBuffer);
                jsonCmd.trim(); // Xóa khoảng trắng thừa /r
                
                if (jsonCmd.length() > 0) {
                    Serial.print("[RX] "); Serial.println(jsonCmd);

                    // --- 1. XỬ LÝ FAST STOP (AN TOÀN HƠN) ---
                    // Nếu gặp stop, ta cố gắng lấy Mutex trong 100ms.
                    // Nếu không lấy được (do MachineTask bị treo), ta VẪN GỌI STOP
                    // nhưng chấp nhận rủi ro nhỏ để cứu hệ thống.
                    if (jsonCmd.indexOf("stop_measure") >= 0) {
                        Serial.println("[CMD] !!! FAST STOP TRIGGERED !!!");
                        // Thử lấy mutex nhanh
                        if (xSemaphoreTake(sysMutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
                            stateMachine.setStopFlag(true); 
                            xSemaphoreGive(sysMutex);
                        } else {
                            // Cưỡng chế dừng nếu không lấy được Mutex
                            Serial.println("[WARN] Force Stop without Mutex!");
                            stateMachine.setStopFlag(true); 
                        }
                    }

                    // --- 2. XỬ LÝ LỆNH BÌNH THƯỜNG ---
                    // Chờ Mutex tối đa 3 giây. 
                    if (xSemaphoreTake(sysMutex, 3000 / portTICK_PERIOD_MS) == pdTRUE) {
                        stateMachine.handleCommand(jsonCmd);
                        xSemaphoreGive(sysMutex);
                    } else {
                        // Nếu đợi quá lâu mà không lấy được Mutex -> Hệ thống quá bận
                        Serial.println("[CMD ERR] System Busy -> Dropped Cmd");
                        // Gửi lỗi về Bridge để biết đường gửi lại
                        String errJson = jsonFormatter.createError("SYSTEM_BUSY_TIMEOUT");
                        uartCommander.pushToQueue(errJson);
                    }
                }
                
                // Reset buffer cho lệnh tiếp theo
                rxIndex = 0; 
            } else {
                // Chỉ nhận nếu còn chỗ trong buffer, chừa 1 byte cho ký tự NULL
                if (rxIndex < 2047) {
                    rxBuffer[rxIndex++] = c;
                } else {
                    // Buffer tràn -> Reset để tránh lỗi bộ nhớ
                    rxIndex = 0;
                    Serial.println("[UART ERR] Buffer Overflow -> Reset");
                }
            }
        }
        // Delay nhẹ để nhường CPU cho các ngắt
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// --- TASK 2: MÁY TRẠNG THÁI (CORE LOGIC) ---
void machineTask(void* pvParameters) {
    unsigned long lastDebugTime = 0;

    Serial.println("[Machine Task] Started...");

    for (;;) {
        // 1. Giữ Mutex để cập nhật trạng thái
        // Dùng portMAX_DELAY: Task này sẽ ngủ đông cho đến khi lấy được Mutex (không tốn CPU)
        if (xSemaphoreTake(sysMutex, portMAX_DELAY) == pdTRUE) {
            stateMachine.update();
            xSemaphoreGive(sysMutex);
        }

        // ====================================================================
        // [FIX QUAN TRỌNG] KIỂM TRA VÀ GỬI DỮ LIỆU ĐI (TX)
        // Nếu không có đoạn này, gói tin chỉ nằm trong Queue mà không bao giờ gửi
        // ====================================================================
        if (uartCommander.hasCommand()) {
            String packet = uartCommander.getCommand();
            if (packet.length() > 0) {
                // Gửi ra UART nối với Bridge
                commandSerial.print(packet);
                // BẮT BUỘC phải có xuống dòng để Bridge cắt chuỗi
                commandSerial.println(); 
                
                // Debug log
                Serial.print("[UART_TX] Sent: ");
                Serial.println(packet);
            }
        }
        // ====================================================================

        // 2. [DEBUG] Heartbeat
        if (millis() - lastDebugTime > 10000) {
            lastDebugTime = millis();
            // unsigned long now = timeSync.getCurrentTime();
            // Serial.printf("[MAIN] Heartbeat. Heap: %d bytes\n", ESP.getFreeHeap());
        }

        // Delay 20ms: Đủ mượt cho logic, nhường thời gian cho UART nhận lệnh
        // Nếu delay quá lâu (50ms+), phản hồi UART có thể bị chậm một chút
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void setup() {
    // 1. Cấu hình hệ thống
    setCpuFrequencyMhz(240); 
    Serial.begin(115200);
    Serial.println("\n\n=== [BOOT] AIR_VL_01 SYSTEM STARTING ===");

    // -----------------------------------------------------------
    // [FIX QUAN TRỌNG] KHỞI ĐỘNG I2C TRƯỚC KHI GỌI CẢM BIẾN
    // -----------------------------------------------------------
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); 
    Serial.printf("[I2C] Bus Started on SDA=%d, SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);

    // [DEBUG] Quét thử xem SHT31 có online không ngay lập tức
    Serial.print("[I2C] Scanning SHT31... ");
    Wire.beginTransmission(0x44);
    if (Wire.endTransmission() == 0) Serial.println("FOUND at 0x44 (OK)");
    else {
        Wire.beginTransmission(0x45);
        if (Wire.endTransmission() == 0) Serial.println("FOUND at 0x45 (OK)");
        else Serial.println("ERROR: NOT FOUND! Check wiring.");
    }
    // -----------------------------------------------------------

    // 2. Khởi động các module phần cứng
    relay.begin();
    rs485.begin();
    
    // Bây giờ mới gọi SHT31 begin (nó sẽ dùng bus I2C đã start ở trên)
    sht31.begin(); 
    
    // UART cho lệnh
    commandSerial.setRxBufferSize(4096); 
    commandSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    uartCommander.begin(commandSerial);

    // 3. Tạo Mutex
    sysMutex = xSemaphoreCreateMutex();
    if (sysMutex == NULL) {
        Serial.println("!!! ERROR: Failed to create Mutex !!!");
        while(1);
    }

    // 4. Khởi động StateMachine
    stateMachine.begin();
    xTaskCreatePinnedToCore(uartRxTask, "UART_RX", 8192, NULL, 2, NULL, 0);

    // Task Machine: Priority 1, chạy Core 1 (Core ứng dụng chính)
    xTaskCreatePinnedToCore(machineTask, "MACHINE", 8192, NULL, 1, NULL, 1);
    
    Serial.println("[BOOT] Tasks Created. System Running.");
}

void loop() {
    // Xóa Task loop mặc định để giải phóng RAM cho Stack
    vTaskDelete(NULL);
}