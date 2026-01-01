#include <Arduino.h>
#include "StateMachine.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h" 
#include "config.h"
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

// --- TASK 1: NHẬN LỆNH TỪ UART ---
void uartRxTask(void* pvParameters) {
    // Tăng buffer để chứa trọn vẹn bản tin JSON
    static char rxBuffer[2048]; 
    static int rxIndex = 0;

    // Xóa rác buffer UART khi khởi động
    while (commandSerial.available()) commandSerial.read();

    Serial.println("[UART Task] Ready to receive commands...");

    for (;;) {
        // Đọc dữ liệu đến
        while (commandSerial.available()) {
            char c = commandSerial.read();
            
            // Tìm ký tự kết thúc dòng (xuống dòng)
            if (c == '\n') {
                rxBuffer[rxIndex] = 0; // Kết thúc chuỗi
                String jsonCmd = String(rxBuffer);
                jsonCmd.trim(); // Xóa khoảng trắng thừa
                
                if (jsonCmd.length() > 0) {
                    Serial.print("[RX] "); Serial.println(jsonCmd);

                    // --- 1. ƯU TIÊN CAO: XỬ LÝ STOP ---
                    // Kiểm tra chuỗi thô ngay lập tức, không cần parse JSON hay đợi Mutex
                    if (jsonCmd.indexOf("stop_measure") >= 0) {
                        stateMachine.setStopFlag(true); 
                        Serial.println("[CMD] !!! FAST STOP TRIGGERED !!!");
                        // Vẫn để code chạy xuống dưới để gửi phản hồi (ACK)
                    }

                    // --- 2. XỬ LÝ LỆNH ---
                    // Chờ Mutex tối đa 3 giây. Nếu máy đang bận đo quá lâu thì có thể bị drop,
                    // nhưng lệnh STOP ở trên đã đảm bảo dừng được máy.
                    if (xSemaphoreTake(sysMutex, 5000 / portTICK_PERIOD_MS) == pdTRUE) {
                        stateMachine.handleCommand(jsonCmd);
                        xSemaphoreGive(sysMutex);
                    } else {
                        Serial.println("[CMD ERR] System Busy (Mutex Timeout) -> Command Dropped");
                        // Mẹo: Nếu lệnh set_mode bị drop do đang đo, hãy gửi STOP trước, sau đó gửi lại set_mode
                    }
                }
                
                // Reset buffer cho lệnh tiếp theo
                rxIndex = 0; 
            } else {
                // Chỉ nhận nếu còn chỗ trong buffer
                if (rxIndex < 2047) {
                    rxBuffer[rxIndex++] = c;
                }
            }
        }
        // Delay nhẹ để tránh chiếm dụng CPU (Watchdog)
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// --- TASK 2: MÁY TRẠNG THÁI (CORE LOGIC) ---
void machineTask(void* pvParameters) {
    unsigned long lastDebugTime = 0;

    Serial.println("[Machine Task] Started...");

    for (;;) {
        // 1. Giữ Mutex để cập nhật trạng thái
        // Dùng portMAX_DELAY để đảm bảo update luôn được gọi khi rảnh
        if (xSemaphoreTake(sysMutex, portMAX_DELAY) == pdTRUE) {
            stateMachine.update();
            xSemaphoreGive(sysMutex);
        }

        // 2. [DEBUG] Heartbeat mỗi 10 giây để biết Auto Mode có đang sống không
        if (millis() - lastDebugTime > 10000) {
            lastDebugTime = millis();
            // Lấy giờ hệ thống kiểm tra
            unsigned long now = timeSync.getCurrentTime();
            if (now > 1600000000) {
                 // In ra dấu chấm để biết Task vẫn đang chạy
                //  Serial.printf("[MAIN] Alive. Time: %lu\n", now);
            } else {
                 Serial.println("[MAIN] Waiting for TimeSync...");
            }
        }

        // Delay 50ms: Đủ nhanh để phản hồi, đủ chậm để UART Task chen vào được
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// --- SETUP ---
void setup() {
    // 1. Cấu hình hệ thống
    setCpuFrequencyMhz(240); // Chạy max tốc độ
    Serial.begin(115200);
    Serial.println("\n\n=== [BOOT] AIR_VL_01 SYSTEM STARTING ===");

    // 2. Khởi động các module phần cứng
    relay.begin();
    rs485.begin();
    sht31.begin();
    
    // UART cho lệnh
    commandSerial.setRxBufferSize(4096); // Buffer phần cứng lớn
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
    // Stack size = 8192 bytes (8KB) để đảm bảo ArduinoJson không làm tràn stack
    xTaskCreatePinnedToCore(uartRxTask,  "UART_RX",8192, NULL, 1,  NULL, 0 );

    xTaskCreatePinnedToCore(machineTask, "MACHINE",8192,NULL,1,NULL,1 );
    
    Serial.println("[BOOT] Tasks Created. System Running.");
}

void loop() {
    // Loop để trống vì chúng ta dùng FreeRTOS Tasks
    // Xóa Task loop mặc định để tiết kiệm RAM  
    vTaskDelete(NULL);
}