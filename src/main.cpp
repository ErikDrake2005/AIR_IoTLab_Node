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
RS485Master rs485(rs485Serial);
SHT31Sensor sht31;
RelayController relay;
JsonFormatter jsonFormatter;
Measurement measurement(rs485, sht31, jsonFormatter);
UARTCommander uartCommander; 
TimeSync timeSync(jsonFormatter);
StateMachine stateMachine(measurement, relay, uartCommander, timeSync);
void uartRxTask(void* pvParameters) {
    static char rxBuffer[2048]; 
    static int rxIndex = 0;
    while (commandSerial.available()) commandSerial.read();
    Serial.println("[UART Task] Ready to receive commands...");
    for (;;) {
        // Đọc dữ liệu đến
        while (commandSerial.available()) {
            char c = commandSerial.read();
            if (c == '\n') {
                rxBuffer[rxIndex] = 0;
                String jsonCmd = String(rxBuffer);
                jsonCmd.trim();
                if (jsonCmd.length() > 0) {
                    Serial.print("[RX] "); Serial.println(jsonCmd);
                    if (jsonCmd.indexOf("stop_measure") >= 0) {
                        Serial.println("[CMD] !!! FAST STOP TRIGGERED !!!");
                        if (xSemaphoreTake(sysMutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
                            stateMachine.setStopFlag(true); 
                            xSemaphoreGive(sysMutex);
                        } else {
                            Serial.println("[WARN] Force Stop without Mutex!");
                            stateMachine.setStopFlag(true); 
                        }
                    }
                    if (xSemaphoreTake(sysMutex, 3000 / portTICK_PERIOD_MS) == pdTRUE) {
                        stateMachine.handleCommand(jsonCmd);
                        xSemaphoreGive(sysMutex);
                    } else {
                        Serial.println("[CMD ERR] System Busy -> Dropped Cmd");
                        String errJson = jsonFormatter.createError("SYSTEM_BUSY_TIMEOUT");
                        uartCommander.pushToQueue(errJson);
                    }
                }
                rxIndex = 0; 
            } else {
                if (rxIndex < 2047) {
                    rxBuffer[rxIndex++] = c;
                } else {
                    rxIndex = 0;
                    Serial.println("[UART ERR] Buffer Overflow -> Reset");
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// --- TASK 2: MÁY TRẠNG THÁI (CORE LOGIC) ---
void machineTask(void* pvParameters) {
    unsigned long lastDebugTime = 0;

    Serial.println("[Machine Task] Started...");

    for (;;) {
        if (xSemaphoreTake(sysMutex, portMAX_DELAY) == pdTRUE) {
            stateMachine.update();
            xSemaphoreGive(sysMutex);
        }
        if (uartCommander.hasCommand()) {
            String packet = uartCommander.getCommand();
            if (packet.length() > 0) {
                commandSerial.print(packet);
                commandSerial.println(); 
                Serial.print("[UART_TX] Sent: ");
                Serial.println(packet);
            }
        }

        // 2. [DEBUG] Heartbeat
        if (millis() - lastDebugTime > 10000) {
            lastDebugTime = millis();
            Serial.printf("[MAIN] Heartbeat. Heap: %d bytes\n", ESP.getFreeHeap());
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void setup() {
    // 1. Cấu hình hệ thống
    setCpuFrequencyMhz(80); 
    Serial.begin(115200);
    Serial.println("\n\n=== [BOOT] AIR_VL_01 SYSTEM STARTING ===");

    // -----------------------------------------------------------
    // [FIX QUAN TRỌNG] KHỞI ĐỘNG I2C TRƯỚC KHI GỌI CẢM BIẾN
    // -----------------------------------------------------------
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); 
    Serial.printf("[I2C] Bus Started on SDA=%d, SCL=%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
    Serial.print("[I2C] Scanning SHT31... ");
    Wire.beginTransmission(0x44);
    if (Wire.endTransmission() == 0) Serial.println("FOUND at 0x44 (OK)");
    else {
        Wire.beginTransmission(0x45);
        if (Wire.endTransmission() == 0) Serial.println("FOUND at 0x45 (OK)");
        else Serial.println("ERROR: NOT FOUND! Check wiring.");
    }
    relay.begin();
    rs485.begin();
    sht31.begin(); 
    commandSerial.setRxBufferSize(4096); 
    commandSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    uartCommander.begin(commandSerial);

    // 3. Tạo Mutex
    sysMutex = xSemaphoreCreateMutex();
    if (sysMutex == NULL) {
        Serial.println("!!! ERROR: Failed to create Mutex !!!");
        while(1);
    }
    stateMachine.begin();
    xTaskCreatePinnedToCore(uartRxTask, "UART_RX", 8192, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(machineTask, "MACHINE", 8192, NULL, 1, NULL, 1);
    Serial.println("[BOOT] Tasks Created. System Running.");
}

void loop() {
    vTaskDelete(NULL);
}
