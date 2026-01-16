#include <Arduino.h>
#include "StateMachine.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" 
#include "Config.h"
#include "CommandProcessor.h"
#include "JsonFormatter.h"
#include "TimeSync.h"
#include "RelayController.h"
#include "Measurement.h"
#include "UARTCommander.h"
#include "RS485Master.h"
#include "SHT31Sensor.h"
#include "CRC32.h"

// --- GLOBAL OBJECTS ---
SemaphoreHandle_t sysMutex;
HardwareSerial rs485Serial(2);
HardwareSerial commandSerial(1);

RS485Master rs485(rs485Serial, PIN_RS485_DE);
SHT31Sensor sht31;
RelayController relay;
JsonFormatter jsonFormatter;
Measurement measurement(rs485, sht31, jsonFormatter);
UARTCommander uartCommander; 
TimeSync timeSync(jsonFormatter);

StateMachine stateMachine(measurement, relay, uartCommander, timeSync);

// --- TASK 1: UART RX (Core 0) ---
// Nhiệm vụ: Nhận chuỗi JSON|CRC từ Bridge và đẩy vào StateMachine
void uartRxTask(void* pvParameters) {
    // Buffer tĩnh để tiết kiệm cấp phát động liên tục
    static char rxBuffer[4096]; 
    static int rxIdx = 0;

    Serial.println("[UART RX] Listening (921600) - JSON Text Mode...");

    for (;;) {
        // Xử lý dữ liệu đến
        while (commandSerial.available()) {
            // [HANDSHAKE] Quan trọng: Có dữ liệu -> Báo BẬN/THỨC ngay lập tức
            digitalWrite(PIN_SLEEP_STATUS, HIGH); 
            
            char c = commandSerial.read();

            // Ký tự kết thúc lệnh (Newline)
            if (c == '\n') {
                rxBuffer[rxIdx] = 0; // Null-terminate
                String rawLine = String(rxBuffer);
                rawLine.trim();
                
                // Reset buffer cho lệnh tiếp theo
                rxIdx = 0;

                if (rawLine.length() > 0) {
                    Serial.print("[RX] "); Serial.println(rawLine);

                    // Đẩy vào StateMachine xử lý (Bảo vệ bằng Mutex)
                    if (xSemaphoreTake(sysMutex, 1000) == pdTRUE) {
                        stateMachine.processRawCommand(rawLine);
                        xSemaphoreGive(sysMutex);
                    }
                }
            } 
            else {
                // Tích lũy ký tự vào buffer
                if (rxIdx < 4095 && c != '\r') {
                    rxBuffer[rxIdx++] = c;
                } else if (rxIdx >= 4095) {
                    // Tràn buffer -> Reset để tránh lỗi
                    Serial.println("[ERR] UART Buffer Overflow!");
                    rxIdx = 0;
                }
            }
        }
        
        // Delay nhẹ để nhường CPU, tránh WDT Reset Core 0
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

// --- TASK 2: MAIN LOGIC (Core 1) ---
// Nhiệm vụ: Chạy StateMachine loop và gửi dữ liệu đi
void machineTask(void* pvParameters) {
    for (;;) {
        // 1. Cập nhật Logic Máy Trạng Thái
        if (xSemaphoreTake(sysMutex, portMAX_DELAY) == pdTRUE) {
            stateMachine.update();
            xSemaphoreGive(sysMutex);
        }

        // 2. Gửi dữ liệu hàng đợi (Uplink) ra UART
        if (uartCommander.hasCommand()) {
            String pkt = uartCommander.getCommand();
            if (pkt.length() > 0) {
                // Tính CRC gửi đi: JSON|CRC_HEX
                unsigned long crc = CRC32::calculate(pkt);
                
                commandSerial.print(pkt);
                commandSerial.print("|");
                commandSerial.println(String(crc, HEX));
                
                Serial.print("[TX] "); Serial.println(pkt);
            }
        }
        
        // Chu kỳ update chính (20ms)
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void setup() {
    // 1. CPU & Debug
    setCpuFrequencyMhz(240); // Max Speed cho xử lý JSON/CRC nhanh nhất
    Serial.begin(115200);
    Serial.println("\n=== [BOOT] NODE AIR_VL_01 (Nested JSON) ===");

    // 2. Cấu hình GPIO Handshake (Phải làm sớm)
    pinMode(PIN_SLEEP_STATUS, OUTPUT);
    digitalWrite(PIN_SLEEP_STATUS, HIGH); // Khởi động lên là THỨC
    pinMode(PIN_WAKEUP_GPIO, INPUT_PULLDOWN);

    // 3. Init Cảm biến & Relay
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); 
    if(!sht31.begin()) Serial.println("[ERR] SHT31 Init Fail");
    
    relay.begin(); 
    
    // Trạng thái Relay an toàn khi khởi động
    Serial.println("[BOOT] Safety: Door OPEN, Fan OFF");
    relay.ON_DOOR(); 
    delay(50);
    relay.OFF_FAN(); 
    relay.OFF(); 
    
    rs485.begin();
    
    // 4. Init UART Giao tiếp Bridge
    commandSerial.setRxBufferSize(4096); // Buffer lớn để chứa JSON lồng nhau
    commandSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    uartCommander.begin(commandSerial);
    
    // 5. Init System
    sysMutex = xSemaphoreCreateMutex();
    stateMachine.begin();
    
    // 6. Khởi tạo Tasks
    // RX_TASK: Core 0, Stack 12KB (để parse JSON)
    xTaskCreatePinnedToCore(uartRxTask, "RX_TASK", 12288, NULL, 2, NULL, 0); 
    
    // MAIN_TASK: Core 1, Stack 16KB (Logic chính)
    xTaskCreatePinnedToCore(machineTask, "MAIN_TASK", 16384, NULL, 1, NULL, 1);
}

void loop() { 
    // Loop trống vì dùng FreeRTOS Tasks
    vTaskDelete(NULL); 
}