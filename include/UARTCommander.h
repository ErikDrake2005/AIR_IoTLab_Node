#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <ArduinoJson.h>

class UARTCommander {
public:
    UARTCommander();
    static UARTCommander* instance;
    void begin(HardwareSerial& serial);
    
    // JSON format (backward compatible)
    void send(const String& data);
    
    // Binary format (PacketDef - more efficient)
    void sendBinary(JsonDocument& doc);
    
    bool hasCommand(); 
    String getCommand(); 
    void clearCommand();
    void uartISR(); 
    void pushToQueue(String data); 

private:
    HardwareSerial* _serial;
    QueueHandle_t _commandQueue;
    String _rxBuffer;
};
