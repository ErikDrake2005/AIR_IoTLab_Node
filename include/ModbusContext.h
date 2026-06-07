#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include <ModbusMaster.h>
#include "Config.h"

class ModbusContext {
public:
    ModbusContext(HardwareSerial& serial, uint8_t dePin = PIN_RS485_DE);
    void begin(uint32_t baud = RS485_BAUD);
    void attach(ModbusMaster& node, uint8_t address);

private:
    HardwareSerial& _serial;
    uint8_t _dePin;
    bool _initialized;
    bool _hasRequested;
    unsigned long _lastRequestMs;

    static ModbusContext* s_active;
    static void preTransmission();
    static void postTransmission();
    void waitForRequestInterval();
    void setTransmit(bool enable);
};
