// src/RS485Master.cpp
#include "RS485Master.h"
RS485Master::RS485Master(HardwareSerial& serial, uint8_t dePin)
    : _serial(serial), _dePin(dePin) {}
void RS485Master::begin() {
    _serial.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    _serial.setTimeout(200);
    pinMode(_dePin, OUTPUT);
    _receiveMode();
    _initialized = true;
    Serial.println("[RS485] Khoi dong thanh cong");
}
void RS485Master::_transmitMode() { digitalWrite(_dePin, HIGH); }
void RS485Master::_receiveMode()  { digitalWrite(_dePin, LOW); }
bool RS485Master::sendCommand(uint8_t slaveId, uint8_t cmd, uint16_t timeoutMs) {
    while (_serial.available()) _serial.read(); 
    _transmitMode();
    vTaskDelay(2 / portTICK_PERIOD_MS); 
    _serial.printf(":S%d,CMD%d\n", slaveId, cmd);
    _serial.flush();
    vTaskDelay(2 / portTICK_PERIOD_MS); 
    _receiveMode();
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (_serial.available()) {
            String resp = _serial.readStringUntil('\n');
            resp.trim();
            if (resp.indexOf(",ACK,") != -1) return true;
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
    return false;
}

String RS485Master::readResponse(uint16_t timeoutMs) {
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (_serial.available()) {
            String line = _serial.readStringUntil('\n');
            line.trim();
            if (line.startsWith(":S")) return line;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    return "";
}
bool RS485Master::parseCH4(const String& line, float& ch4) {
    if (!_initialized) { ch4 = 0; return false; }
    if (!line.startsWith(":S1,DATA,")) return false;
    ch4 = line.substring(9).toFloat();
    return true;
}

bool RS485Master::parseMICS(const String& line, float& co, float& alc, float& nh3, float& h2) {
    if (!_initialized) { co = alc = nh3 = h2 = 0; return false; }
    if (!line.startsWith(":S2,DATA,")) return false;
    
    int p1 = line.indexOf(',', 9);
    int p2 = line.indexOf(',', p1 + 1);
    int p3 = line.indexOf(',', p2 + 1);
    
    if (p1 == -1 || p2 == -1 || p3 == -1) return false;
    
    co  = line.substring(9, p1).toFloat();
    alc = line.substring(p1 + 1, p2).toFloat();
    nh3 = line.substring(p2 + 1, p3).toFloat();
    h2  = line.substring(p3 + 1).toFloat();
    return true;
}