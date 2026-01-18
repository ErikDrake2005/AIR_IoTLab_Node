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
#include "driver/rtc_io.h" // Thư viện để config chân Wakeup

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

// --- MAIN TASK ---
// Task này chịu trách nhiệm:
// 1. Chạy vòng lặp StateMachine (Logic chính)
// 2. Lấy lệnh từ hàng đợi UART (Do UARTCommander nhận về) để xử lý
void machineTask(void* pvParameters) {
    Serial.println("[MAIN] Machine Task Started");
    
    for (;;) {
        // 1. Cập nhật State Machine (Logic đo đạc, relay...)
        if (xSemaphoreTake(sysMutex, portMAX_DELAY) == pdTRUE) {
            stateMachine.update();
            xSemaphoreGive(sysMutex);
        }

        // 2. Kiểm tra có lệnh mới từ UARTCommander không?
        // (Lưu ý: UARTCommander đã có Task riêng để đọc Serial và đẩy vào Queue)
        if (uartCommander.hasCommand()) {
            String rawCmd = uartCommander.getCommand();
            
            if (rawCmd.length() > 0) {
                Serial.print("[RX CMD] "); 
                Serial.println(rawCmd);

                // Xử lý lệnh trong vùng an toàn Mutex
                if (xSemaphoreTake(sysMutex, 1000) == pdTRUE) {
                    stateMachine.processRawCommand(rawCmd);
                    xSemaphoreGive(sysMutex);
                }
            }
        }
        
        // Delay nhẹ để nhường CPU
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void setup() {
    setCpuFrequencyMhz(240);
    
    // --- 1. HANDSHAKE (QUAN TRỌNG NHẤT) ---
    // Bật ngay lập tức để Bridge biết Node đã thức
    // PIN_SLEEP_STATUS thường là GPIO 32
    pinMode(PIN_SLEEP_STATUS, OUTPUT);
    digitalWrite(PIN_SLEEP_STATUS, HIGH); 

    // --- 2. CẤU HÌNH WAKEUP (GPIO 33) ---
    // PIN_WAKEUP_GPIO thường là GPIO 33
    pinMode(PIN_WAKEUP_GPIO, INPUT_PULLDOWN); 
    // Cho phép đánh thức khi chân này lên mức HIGH (1)
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKEUP_GPIO, 1);

    // --- 3. Debug & Init ---
    Serial.begin(115200);
    Serial.println("\n=== [BOOT] NODE AIR_VL_01 (Final V2) ===");
    
    // Kiểm tra nguyên nhân thức dậy
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("[BOOT] Woke up by BRIDGE Trigger!");
    } else {
        Serial.println("[BOOT] Woke up by POWER/TIMER");
    }

    // --- 4. Init Hardware ---
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); 
    if(!sht31.begin()) Serial.println("[ERR] SHT31 Init Fail");
    
    relay.begin(); 
    Serial.println("[BOOT] Safety: Door OPEN, Fan OFF");
    relay.ON_DOOR(); 
    delay(50);
    relay.OFF_FAN(); 
    relay.OFF(); 
    
    rs485.begin();
    
    // --- 5. Init UART Commander ---
    // Khởi động Serial 1 cho giao tiếp với Bridge
    commandSerial.setRxBufferSize(4096);
    commandSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    
    // Flush bất kỳ dữ liệu rác từ bootloader
    delay(100);
    while(commandSerial.available()) commandSerial.read();
    commandSerial.flush();
    
    // UARTCommander sẽ tự tạo 2 Task (TX và RX) chạy ngầm
    uartCommander.begin(commandSerial);
    
    sysMutex = xSemaphoreCreateMutex();
    stateMachine.begin();
    
    // --- 6. Create Tasks ---
    // Không cần tạo uartRxTask thủ công nữa vì UARTCommander đã làm rồi
    xTaskCreatePinnedToCore(machineTask, "MAIN_TASK", 16384, NULL, 1, NULL, 1);
}

void loop() { 
    vTaskDelete(NULL); 
}