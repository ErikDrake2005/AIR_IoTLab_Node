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

SemaphoreHandle_t sysMutex;
HardwareSerial rs485Serial(2);
HardwareSerial commandSerial(1);

// Khởi tạo Modules
RS485Master rs485(rs485Serial, PIN_RS485_DE);
SHT31Sensor sht31;
RelayController relay;
JsonFormatter jsonFormatter; // Vẫn cần cho Measurement dùng
Measurement measurement(rs485, sht31, jsonFormatter);
UARTCommander uartCommander; 
TimeSync timeSync(jsonFormatter);

// [FIX] Constructor này KHÔNG truyền jsonFormatter nữa (nó tự tạo bên trong)
StateMachine stateMachine(measurement, relay, uartCommander, timeSync);

// TASK NHẬN UART VÀ CHECK CRC
void uartRxTask(void* pvParameters) {
    static char rxBuffer[2048]; 
    static int rxIndex = 0;
    while (commandSerial.available()) commandSerial.read();
    Serial.println("[UART RX] Listening...");

    for (;;) {
        while (commandSerial.available()) {
            char c = commandSerial.read();
            if (c == '\n') {
                rxBuffer[rxIndex] = 0;
                String raw = String(rxBuffer); raw.trim(); rxIndex = 0;
                
                if (raw.length() > 0) {
                    // Check CRC: {"cmd":...}|ABC12345
                    int sep = raw.lastIndexOf('|');
                    if (sep > 0) {
                        String json = raw.substring(0, sep);
                        String crcS = raw.substring(sep + 1);
                        unsigned long cal = CRC32::calculate(json);
                        unsigned long rec = strtoul(crcS.c_str(), NULL, 16);
                        
                        if (cal == rec) {
                            Serial.print("[RX OK] "); Serial.println(json);
                            // Ưu tiên xử lý lệnh Stop
                            if (json.indexOf("stop_measure") >= 0 || json.indexOf("\"EN\":0") >= 0) {
                                stateMachine.setStopFlag(true);
                            }
                            // Đẩy vào Logic
                            if (xSemaphoreTake(sysMutex, 2000) == pdTRUE) {
                                stateMachine.handleCommand(json);
                                xSemaphoreGive(sysMutex);
                            }
                        } else {
                            Serial.printf("[RX ERR] CRC Fail. Cal:%X Rec:%X\n", cal, rec);
                        }
                    }
                }
            } else {
                if (rxIndex < 2047) rxBuffer[rxIndex++] = c; else rxIndex = 0;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// TASK LOGIC CHÍNH
void machineTask(void* pvParameters) {
    Serial.println("[Logic Task] Started");
    for (;;) {
        // 1. Chạy state machine
        if (xSemaphoreTake(sysMutex, portMAX_DELAY) == pdTRUE) {
            stateMachine.update();
            xSemaphoreGive(sysMutex);
        }
        // 2. Gửi dữ liệu UART (Queue)
        if (uartCommander.hasCommand()) {
            String pkt = uartCommander.getCommand();
            if (pkt.length() > 0) {
                commandSerial.println(pkt);
                Serial.print("[TX] "); Serial.println(pkt);
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void setup() {
    setCpuFrequencyMhz(80); 
    Serial.begin(115200);
    Serial.println("\n=== [BOOT] AIR_VL_01 ===");
    
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); 
    if(!sht31.begin()) Serial.println("[SHT31] Error/Custom Addr");
    
    relay.begin(); 
    rs485.begin();
    
    commandSerial.setRxBufferSize(2048); 
    commandSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    uartCommander.begin(commandSerial);
    
    sysMutex = xSemaphoreCreateMutex();
    stateMachine.begin();
    
    xTaskCreatePinnedToCore(uartRxTask, "UART_RX", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(machineTask, "LOGIC", 8192, NULL, 1, NULL, 1);
}

void loop() { vTaskDelete(NULL); }