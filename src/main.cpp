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
#include "PacketDef.h" 

SemaphoreHandle_t sysMutex;
HardwareSerial rs485Serial(2);
HardwareSerial commandSerial(1);

// Module Instantiation
RS485Master rs485(rs485Serial, PIN_RS485_DE);
SHT31Sensor sht31;
RelayController relay;
JsonFormatter jsonFormatter;
Measurement measurement(rs485, sht31, jsonFormatter);
UARTCommander uartCommander; 
TimeSync timeSync(jsonFormatter);

StateMachine stateMachine(measurement, relay, uartCommander, timeSync);

// --- TASK 1: UART RX TỐC ĐỘ CAO (HIGH SPEED HANDLER) ---
// Support cả Binary (PacketDef) và JSON format
void uartRxTask(void* pvParameters) {
    static uint8_t binBuf[256];  // Buffer cho binary
    static char jsonBuf[4096];   // Buffer cho JSON
    static int binIdx = 0, jsonIdx = 0;
    static bool isBinaryMode = false;
    static int expectedBinLen = 0;
    
    Serial.println("[UART RX] High Speed Listening (921600) - Binary+JSON support...");

    for (;;) {
        while (commandSerial.available()) {
            uint8_t byte = commandSerial.read();
            
            // --- AUTO-DETECT Mode ---
            // Nếu byte đầu không phải '{', assume binary
            if (binIdx == 0 && jsonIdx == 0) {
                isBinaryMode = (byte != '{');
                if (isBinaryMode) Serial.println("[UART] Binary mode detected");
            }

            if (isBinaryMode) {
                // === BINARY MODE (PacketDef) ===
                binBuf[binIdx++] = byte;
                
                // Heuristic: Binary messages thường < 256 bytes, look for DT_END
                if (binIdx > 2 && byte == DT_END && binIdx < 256) {
                    Serial.printf("[BIN RX] %d bytes received\n", binIdx);
                    
                    // Decode binary -> JSON
                    JsonDocument doc;
                    PacketUtils::decodeBinaryToJson(binBuf, binIdx, doc);
                    
                    String jsonStr;
                    serializeJson(doc, jsonStr);
                    Serial.print("[BIN DECODED] "); Serial.println(jsonStr);
                    
                    // Priority: STOP command
                    if (jsonStr.indexOf("stop") >= 0 || doc["EN"] == 0) {
                        stateMachine.setStopFlag(true);
                    }
                    
                    // Process command
                    if (xSemaphoreTake(sysMutex, 1000) == pdTRUE) {
                        stateMachine.handleCommand(jsonStr);
                        xSemaphoreGive(sysMutex);
                    }
                    
                    binIdx = 0;
                    jsonIdx = 0;
                    isBinaryMode = false;
                } 
                else if (binIdx >= 256) {
                    // Buffer overflow - reset
                    Serial.println("[ERR] Binary buffer overflow!");
                    binIdx = 0;
                    jsonIdx = 0;
                }
            } 
            else {
                // === JSON MODE ===
                if (byte == '\n') {
                    jsonBuf[jsonIdx] = 0;
                    String raw(jsonBuf);
                    raw.trim();
                    jsonIdx = 0;

                    if (raw.length() > 0) {
                        String finalJson = "";
                        int sep = raw.lastIndexOf('|');

                        // Format: JSON|CRC
                        if (sep > 0) {
                            String jsonPart = raw.substring(0, sep);
                            String crcPart = raw.substring(sep + 1);
                            unsigned long cal = CRC32::calculate(jsonPart);
                            unsigned long rec = strtoul(crcPart.c_str(), NULL, 16);
                            
                            if (cal == rec) {
                                finalJson = jsonPart;
                            } else {
                                Serial.printf("[ERR] CRC Fail. Cal:%lX Rec:%lX\n", cal, rec);
                            }
                        } 
                        // Pure JSON
                        else if (raw.startsWith("{") && raw.endsWith("}")) {
                            finalJson = raw;
                        }

                        // Process
                        if (finalJson.length() > 0) {
                            Serial.print("[JSON RX] "); Serial.println(finalJson);
                            
                            // Priority: STOP
                            if (finalJson.indexOf("stop") >= 0 || finalJson.indexOf("\"EN\":0") >= 0) {
                                stateMachine.setStopFlag(true);
                            }

                            if (xSemaphoreTake(sysMutex, 1000) == pdTRUE) {
                                stateMachine.handleCommand(finalJson);
                                xSemaphoreGive(sysMutex);
                            }
                        }
                    }
                    binIdx = 0;
                    isBinaryMode = false;
                } 
                else {
                    // Accumulate JSON
                    if (jsonIdx < 4095 && byte != '\r') {
                        jsonBuf[jsonIdx++] = byte;
                    } else if (jsonIdx >= 4095) {
                        Serial.println("[ERR] JSON buffer overflow!");
                        jsonIdx = 0;
                    }
                }
            }
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

// --- TASK 2: MAIN LOGIC ---
void machineTask(void* pvParameters) {
    for (;;) {
        // 1. Update State Machine
        if (xSemaphoreTake(sysMutex, portMAX_DELAY) == pdTRUE) {
            stateMachine.update();
            xSemaphoreGive(sysMutex);
        }

        // 2. Gửi dữ liệu đi (Outbound)
        if (uartCommander.hasCommand()) {
            String pkt = uartCommander.getCommand();
            if (pkt.length() > 0) {
                // Tính CRC gửi đi
                unsigned long crc = CRC32::calculate(pkt);
                
                // Gửi nguyên khối (Atomic) để tránh bị cắt giữa chừng
                commandSerial.print(pkt);
                commandSerial.print("|");
                commandSerial.println(String(crc, HEX));
                
                Serial.print("[TX] "); Serial.println(pkt);
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void setup() {
    setCpuFrequencyMhz(160); // [BOOST] Tăng tốc CPU lên tối đa để xử lý UART nhanh
    Serial.begin(115200);    // Cổng Debug
    Serial.println("\n=== [BOOT] AIR_VL_01 (HighSpeed) ===");
    
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); 
    if(!sht31.begin()) Serial.println("[SHT31] Error/Custom Addr");
    
    relay.begin(); 
    
    // ===== [INIT ON BOOT] Mở cửa, tắt quạt, ngưng đo =====
    Serial.println("[BOOT] Initialize relays: DOOR ON, FAN OFF, MEASURE OFF");
    relay.ON_DOOR();    // Mở cửa
    delay(10);
    relay.OFF_FAN();    // Tắt quạt
    delay(10);
    relay.OFF();        // Ngưng đo (turn off measurement)
    Serial.println("[BOOT] Relay init complete");
    
    rs485.begin();
    
    // [QUAN TRỌNG] Tăng buffer cho UART 1 lên 4KB để chịu tải 921600
    commandSerial.setRxBufferSize(4096); 
    commandSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    uartCommander.begin(commandSerial);
    
    sysMutex = xSemaphoreCreateMutex();
    stateMachine.begin();
    
    // [FIX] Tăng Stack để tránh overflow khi xử lý JsonDocument
    // RX_FAST: 12KB (từ 6KB) - xử lý parse + handleCommand + JsonDocument
    // MAIN: 16KB (từ 8KB) - xử lý _sendMachineStatus() với JsonDocument
    xTaskCreatePinnedToCore(uartRxTask, "RX_FAST", 12288, NULL, 2, NULL, 0); 
    xTaskCreatePinnedToCore(machineTask, "MAIN", 16384, NULL, 1, NULL, 1);
}

void loop() { vTaskDelete(NULL); }