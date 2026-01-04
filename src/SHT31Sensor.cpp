#include "SHT31Sensor.h"

SHT31Sensor::SHT31Sensor() : _sht31() {
    _initialized = false;
}

// [QUAN TRỌNG] Phải có "SHT31Sensor::" ở phía trước
bool SHT31Sensor::begin() {   // <--- KIỂM TRA DÒNG NÀY
    // Thử địa chỉ 0x44
    if (_sht31.begin(0x44)) {
        _initialized = true;
        Serial.println("[SHT31] Init Success at 0x44");
        return true;
    }
    
    // Thử địa chỉ 0x45
    if (_sht31.begin(0x45)) {
        _initialized = true;
        Serial.println("[SHT31] Init Success at 0x45");
        return true;
    }

    Serial.println("[SHT31] ERROR: Sensor not found! Check wiring.");
    _initialized = false;
    return false;
}

bool SHT31Sensor::readData(float &temperature, float &humidity) {
    if (!_initialized) {
        // Thử kết nối lại nếu chưa init được
        if (!begin()) {
            temperature = 0;
            humidity = 0;
            return false;
        }
    }

    float t = _sht31.readTemperature();
    float h = _sht31.readHumidity();

    if (isnan(t) || isnan(h)) {
        Serial.println("[SHT31] Read Error: NaN");
        return false;
    }

    temperature = t;
    humidity = h;
    return true;
}