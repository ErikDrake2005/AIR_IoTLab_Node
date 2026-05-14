#include "ModbusContext.h"

ModbusContext* ModbusContext::s_active = nullptr;

ModbusContext::ModbusContext(HardwareSerial& serial, uint8_t dePin)
    : _serial(serial), _dePin(dePin), _initialized(false) {}

void ModbusContext::begin(uint32_t baud) {
    if (_initialized) return;

    pinMode(_dePin, OUTPUT);
    digitalWrite(_dePin, LOW);
    _serial.begin(baud, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    _initialized = true;
    s_active = this;
}

void ModbusContext::attach(ModbusMaster& node, uint8_t address) {
    node.begin(address, _serial);
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);
}

void ModbusContext::preTransmission() {
    if (s_active) s_active->setTransmit(true);
}

void ModbusContext::postTransmission() {
    if (s_active) s_active->setTransmit(false);
}

void ModbusContext::setTransmit(bool enable) {
    digitalWrite(_dePin, enable ? HIGH : LOW);
}
