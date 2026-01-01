#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

class UARTCommander {
public:
    UARTCommander();
    static UARTCommander* instance;
    void begin(HardwareSerial& serial);
    void send(const String& data);
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
