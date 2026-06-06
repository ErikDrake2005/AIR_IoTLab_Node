#include "RS485Master.h"

const uint16_t RS485_CMD_TIMEOUT_MS = 2000;    // Timeout cho lệnh gửi đi
const uint16_t RS485_RESPONSE_TIMEOUT_MS = 2500; // Timeout cho phản hồi

namespace {
uint16_t crc16Modbus(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
}

RS485Master::RS485Master(HardwareSerial& serial, uint8_t dePin)
    : _serial(serial), _dePin(dePin) {}
void RS485Master::begin() {
    _serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    _serial.setTimeout(300);
    pinMode(_dePin, OUTPUT);
    _receiveMode();
    _initialized = true;
    Serial.println("[RS485] Khoi dong thanh cong");
}
void RS485Master::_transmitMode() { digitalWrite(_dePin, HIGH); }
void RS485Master::_receiveMode()  { digitalWrite(_dePin, LOW); }

bool RS485Master::readHoldingRegister(uint8_t slaveId, uint16_t reg, uint16_t& value, uint16_t timeoutMs) {
    if (!_initialized) return false;

    while (_serial.available()) _serial.read();

    uint8_t request[8];
    request[0] = slaveId;
    request[1] = 0x03;
    request[2] = static_cast<uint8_t>((reg >> 8) & 0xFF);
    request[3] = static_cast<uint8_t>(reg & 0xFF);
    request[4] = 0x00;
    request[5] = 0x01;
    uint16_t crc = crc16Modbus(request, 6);
    request[6] = static_cast<uint8_t>(crc & 0xFF);
    request[7] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    _transmitMode();
    delay(5);
    _serial.write(request, sizeof(request));
    _serial.flush();
    delay(5);
    _receiveMode();

    uint8_t response[7];
    size_t received = 0;
    unsigned long start = millis();
    while (received < sizeof(response) && (millis() - start < timeoutMs)) {
        if (_serial.available()) {
            response[received++] = static_cast<uint8_t>(_serial.read());
        } else {
            delay(2);
        }
    }

    if (received != sizeof(response)) {
        Serial.printf("[RS485] TIMEOUT waiting response from S%d\n", slaveId);
        return false;
    }

    if (response[0] != slaveId || response[1] != 0x03 || response[2] != 0x02) {
        Serial.printf("[RS485] INVALID response header from S%d\n", slaveId);
        return false;
    }

    uint16_t respCrc = static_cast<uint16_t>(response[5]) | (static_cast<uint16_t>(response[6]) << 8);
    uint16_t calcCrc = crc16Modbus(response, 5);
    if (respCrc != calcCrc) {
        Serial.printf("[RS485] CRC error from S%d (calc=0x%04X recv=0x%04X)\n", slaveId, calcCrc, respCrc);
        return false;
    }

    value = static_cast<uint16_t>(response[3] << 8) | response[4];
    return true;
}