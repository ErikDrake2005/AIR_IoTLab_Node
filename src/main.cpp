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
#include "ModbusContext.h"
#include "Sensor.h"
#include "CRC32.h"
#include "driver/rtc_io.h"

// --- GLOBAL OBJECTS ---
SemaphoreHandle_t sysMutex;
HardwareSerial rs485Serial(2);
HardwareSerial commandSerial(1);
ModbusContext modbus(rs485Serial, PIN_RS485_DE);
SensorRegistry sensorRegistry;
RelayController relay;
JsonFormatter jsonFormatter;
Measurement measurement(sensorRegistry, jsonFormatter);
UARTCommander uartCommander; 
TimeSync timeSync(jsonFormatter);
StateMachine stateMachine(measurement, relay, uartCommander, timeSync);
// --- MAIN TASK ---
void machineTask(void* pvParameters) {
    Serial.println("[MAIN] Machine Task Started");
    for (;;) {
        // 1. Cập nhật State Machine (Logic đo đạc, relay...)
        if (xSemaphoreTake(sysMutex, portMAX_DELAY) == pdTRUE) {
            stateMachine.update();
            xSemaphoreGive(sysMutex);
        }
        // 2. Kiểm tra có lệnh mới từ UARTCommander không?
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
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void setup() {
    setCpuFrequencyMhz(80);
    pinMode(PIN_SLEEP_STATUS, OUTPUT);
    digitalWrite(PIN_SLEEP_STATUS, HIGH);
    pinMode(PIN_WAKEUP_GPIO, INPUT_PULLDOWN); 
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKEUP_GPIO, 1);
    Serial.begin(115200);
    Serial.println("\n=== [BOOT] NODE AIR_VL_01 (Final V2) ===");
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println("[BOOT] Woke up by BRIDGE Trigger!");
    } else {
        Serial.println("[BOOT] Woke up by POWER/TIMER");
    }
    relay.begin(); 
    Serial.println("[BOOT] Safety: Door OPEN, Fan OFF");
    relay.ON_DOOR(); 
    delay(50);
    relay.OFF_FAN(); 
    relay.OFF(); 
    modbus.begin();
    auto co2Sensor = SensorFactory::create(SensorType::Co2, RS485_ADDR_CO2, modbus);
    auto ch4Sensor = SensorFactory::create(SensorType::Ch4, RS485_ADDR_CH4, modbus);
    auto tempHumSensor = SensorFactory::create(SensorType::TempHum, RS485_ADDR_TEMP_HUM, modbus);
    if (co2Sensor) sensorRegistry.addOrUpdate(std::move(co2Sensor));
    if (ch4Sensor) sensorRegistry.addOrUpdate(std::move(ch4Sensor));
    if (tempHumSensor) sensorRegistry.addOrUpdate(std::move(tempHumSensor));
    commandSerial.setRxBufferSize(4096);
    commandSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    delay(100);
    while(commandSerial.available()) commandSerial.read();
    commandSerial.flush();
    uartCommander.begin(commandSerial);
    sysMutex = xSemaphoreCreateMutex();
    stateMachine.begin();
    xTaskCreatePinnedToCore(machineTask, "MAIN_TASK", 16384, NULL, 1, NULL, 1);
}

void loop() { 
    vTaskDelete(NULL); 
}