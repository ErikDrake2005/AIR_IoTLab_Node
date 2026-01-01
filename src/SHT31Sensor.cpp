#include "SHT31Sensor.h"
SHT31Sensor::SHT31Sensor() : _sht31() {}
bool SHT31Sensor::begin() {
    Wire.begin(SHT31_SDA_PIN, SHT31_SCL_PIN);
    _initialized = _sht31.begin(SHT31_I2C_ADDR);
    if (!_initialized) {
        Serial.println("[SHT31] Khong tim thay cam bien SHT31 – mặc định 0");
    } else {
        Serial.println("[SHT31] Da khoi dong cam bien SHT31");
    }
    return _initialized;
}

bool SHT31Sensor::readData(float &temperature, float &humidity) {
    if (!_initialized) {
        temperature = 0;
        humidity = 0;
        Serial.println("[SHT31] Cam bien fail – mặc định 0");
        return false;
    }
    temperature = _sht31.readTemperature();
    humidity = _sht31.readHumidity();
    if (isnan(temperature) || isnan(humidity)) {
        Serial.println("[SHT31] Loi doc du lieu cam bien SHT31");
        return false;
    }
    return true;
}