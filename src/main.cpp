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
#include "driver/uart.h" // [MỚI] Thư viện điều khiển UART Wakeup

SemaphoreHandle_t sysMutex;
HardwareSerial rs485Serial(2);
HardwareSerial commandSerial(1); // Serial1
RS485Master rs485(rs485Serial);
SHT31Sensor sht31;
RelayController relay;
JsonFormatter jsonFormatter;
Measurement measurement(rs485, sht31, jsonFormatter);
UARTCommander uartCommander; 
TimeSync timeSync(jsonFormatter);
StateMachine stateMachine(measurement, relay, uartCommander, timeSync);

// --- HÀM HỖ TRỢ: ĐỌC PIN ---
float readBatteryVoltage() {
    analogReadResolution(12);
    long sum = 0;
    for(int i=0; i<20; i++) {
        sum += analogRead(PIN_BATTERY);
        delay(2);
    }
    return (sum / 20.0) * (3.3 / 4095.0) * BAT_VOLTAGE_DIVIDER;
}

// --- HÀM HỖ TRỢ: ĐI VÀO NGỦ SÂU (SHUTDOWN) ---
void enterDeepSleep(const char* reasonMsg) {
    Serial.printf("[PWR] CRITICAL: %s -> ENTERING DEEP SLEEP\n", reasonMsg);
    
    // 1. Gửi lời trăn trối
    HardwareSerial cmdSerial(1);
    cmdSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    cmdSerial.println("{\"pw\":\"low\"}");
    cmdSerial.flush();
    delay(50); 

    // 2. Tắt mắt (Báo Node chết)
    pinMode(PIN_EYE, OUTPUT); 
    digitalWrite(PIN_EYE, LOW);
    relay.OFF_FAN();
    relay.OFF_DOOR();
    vTaskDelay(5000 / portTICK_PERIOD_MS); 

    // 4. Ngủ
    esp_sleep_enable_timer_wakeup(BAT_CHECK_INTERVAL * 60 * 1000000ULL);
    esp_deep_sleep_start();
}

// --- HÀM SETUP: KIỂM TRA PIN LÚC KHỞI ĐỘNG ---
void checkBatteryAtBoot() {
    pinMode(PIN_BATTERY, INPUT);
    // Tắt mắt trước
    pinMode(PIN_EYE, OUTPUT); digitalWrite(PIN_EYE, LOW); 

    float voltage = readBatteryVoltage();
    Serial.printf("[PWR] Boot Voltage: %.2fV\n", voltage);
    
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    float threshold = (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) ? BAT_RECOVERY_VOLTAGE : BAT_MIN_VOLTAGE;

    if (voltage < threshold) {
        enterDeepSleep("Boot Low Voltage");
    }
    
    // Nếu tỉnh dậy từ Deep Sleep -> Báo cáo
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
         HardwareSerial cmdSerial(1);
         cmdSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
         cmdSerial.println("{\"pw\":\"OK\"}");
         cmdSerial.flush(); 
         delay(50);
    }
}

void uartRxTask(void* pvParameters) {
    static char rxBuffer[2048]; 
    static int rxIndex = 0;
    while (commandSerial.available()) commandSerial.read();
    Serial.println("[UART Task] Ready...");
    
    for (;;) {
        while (commandSerial.available()) {
            char c = commandSerial.read();
            if (c == '\n') {
                rxBuffer[rxIndex] = 0;
                String jsonCmd = String(rxBuffer);
                jsonCmd.trim();
                if (jsonCmd.length() > 0) {
                    Serial.print("[RX] "); Serial.println(jsonCmd);
                    // Fast Path: Stop Measure
                    if (jsonCmd.indexOf("stop_measure") >= 0) {
                        if (xSemaphoreTake(sysMutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
                            stateMachine.setStopFlag(true); 
                            xSemaphoreGive(sysMutex);
                        } else stateMachine.setStopFlag(true); 
                    }
                    // Normal Path
                    if (xSemaphoreTake(sysMutex, 3000 / portTICK_PERIOD_MS) == pdTRUE) {
                        stateMachine.handleCommand(jsonCmd);
                        xSemaphoreGive(sysMutex);
                    } else {
                        uartCommander.pushToQueue(jsonFormatter.createError("BUSY"));
                    }
                }
                rxIndex = 0; 
            } else {
                if (rxIndex < 2047) rxBuffer[rxIndex++] = c;
                else rxIndex = 0;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// --- TASK CHÍNH: LOGIC + POWER MANAGEMENT ---
void machineTask(void* pvParameters) {
    unsigned long lastBatCheck = 0;
    unsigned long lastDebug = 0;

    Serial.println("[Machine Task] Started with LIGHT SLEEP Logic...");

    for (;;) {
        // 1. WATCHDOG PIN (10s/lần)
        if (millis() - lastBatCheck > 10000) { 
            lastBatCheck = millis();
            float vBat = readBatteryVoltage();
            if (vBat < BAT_MIN_VOLTAGE) {
                enterDeepSleep("Runtime Drop");
            }
        }

        // 2. LOGIC STATE MACHINE
        if (xSemaphoreTake(sysMutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
            stateMachine.update();
            xSemaphoreGive(sysMutex);
        }

        // 3. GỬI DATA UART
        if (uartCommander.hasCommand()) {
            String packet = uartCommander.getCommand();
            if (packet.length() > 0) {
                commandSerial.println(packet);
                Serial.print("[TX] "); Serial.println(packet);
                vTaskDelay(50 / portTICK_PERIOD_MS); 
            }
        }
        
        // [DEBUG]
        if (millis() - lastDebug > 10000) {
            lastDebug = millis();
            Serial.printf("[MAIN] Heap: %d | Bat: %.2fV\n", ESP.getFreeHeap(), readBatteryVoltage());
        }

        // 4. LIGHT SLEEP LOGIC (Chỉ chạy khi IDLE)
        uint32_t sleepMs = stateMachine.getSleepIntervalMs();
        
        // Chỉ ngủ khi rảnh > 5 giây
        if (sleepMs > 5000) {
            Serial.printf("[PWR] Sleeping %d ms...\n", sleepMs);
            Serial.flush();

            // A. HANDSHAKE: Tắt Mắt (Báo bận/ngủ)
            digitalWrite(PIN_EYE, LOW);

            // B. Cấu hình Wakeup
            esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);
            uart_set_wakeup_threshold(UART_NUM_1, 3); // Wakeup khi có cạnh xung UART
            esp_sleep_enable_uart_wakeup(UART_NUM_1);

            // C. Ngủ Light Sleep
            esp_light_sleep_start();

            // --- TỈNH DẬY ---
            
            // D. HANDSHAKE: Bật Mắt (Báo sẵn sàng nhận lệnh)
            digitalWrite(PIN_EYE, HIGH);
            Serial.println("[PWR] Woke up! EYE=HIGH");

            esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
            if (cause == ESP_SLEEP_WAKEUP_UART) {
                Serial.println("[PWR] Woke by UART -> Waiting for CMD...");
                // Không cần làm gì, uartRxTask sẽ tự hứng dữ liệu tiếp theo
                // Chỉ cần delay nhỏ để hệ thống ổn định
                delay(10);
            }
        }
        
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n\n=== [BOOT] AIR_VL_01 SYSTEM STARTING ===");
    checkBatteryAtBoot(); 

    // 2. BẬT MẮT (Handshake)
    pinMode(PIN_EYE, OUTPUT);
    digitalWrite(PIN_EYE, HIGH); 
    Serial.println("[BOOT] EYE PIN set HIGH (Ready)");

    // 3. KHỞI TẠO PHẦN CỨNG
    setCpuFrequencyMhz(80); 
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); 
    relay.begin();
    rs485.begin();
    sht31.begin(); 
    
    commandSerial.setRxBufferSize(4096); 
    commandSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    uartCommander.begin(commandSerial);

    sysMutex = xSemaphoreCreateMutex();
    stateMachine.begin();
    
    xTaskCreatePinnedToCore(uartRxTask, "UART_RX", 8192, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(machineTask, "MACHINE", 8192, NULL, 1, NULL, 1);
    
    Serial.println("[BOOT] System Running.");
}

void loop() {
    vTaskDelete(NULL);
}