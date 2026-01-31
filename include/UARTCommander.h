#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

struct UartTxMessage {
    uint8_t buffer[256];
    size_t length;
};
struct UartRxMessage {
    char cmd[512];
};

class UARTCommander {
private:
    HardwareSerial* _serial;
    QueueHandle_t _txQueue;
    QueueHandle_t _rxQueue;
    static UARTCommander* instance;
    static void txTask(void* pvParameters);
    static void rxTask(void* pvParameters);

public:
    UARTCommander();
    void begin(HardwareSerial& serial);
    void send(const String& jsonStr);
    void sendBinary(JsonDocument& doc);
    void enqueueTx(const uint8_t* data, size_t len, const char* debugTag);
    bool hasCommand();
    String getCommand();
};