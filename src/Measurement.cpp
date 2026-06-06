#include "Measurement.h"
#include "JsonFormatter.h"
#include "Config.h"

const int SENSOR_MAX_RETRIES = 3;
const unsigned long SENSOR_RETRY_DELAY_MS = 3000;

Measurement::Measurement(SensorRegistry& sensors, JsonFormatter& json)
    : _sensors(sensors), _json(json) {
    _lastError = "";
}

String Measurement::getLastError() {
    return _lastError;
}

bool Measurement::doFullMeasurement(String& outputJson) {
    MeasurementData data;
    doFullMeasurement(data);
    outputJson = _json.createDataJson(data.co2, data.ch4, data.temp, data.hum);
    return true;
}

static void applyReading(const SensorReading& reading, MeasurementData& data) {
    switch (reading.type) {
        case SensorType::Co2:
            data.co2 = reading.value1;
            break;
        case SensorType::Ch4:
            data.ch4 = reading.value1;
            break;
        case SensorType::TempHum:
            data.temp = reading.value1;
            data.hum = reading.value2;
            break;
    }
}

bool Measurement::doFullMeasurement(MeasurementData& data, bool* abortFlag, UrgentCheckCallback checkCallback) {
    _lastError = "";
    data.co2 = -1.0f;
    data.ch4 = -1.0f;
    data.temp = -1.0f;
    data.hum = -1.0f;

    auto delayWithCheck = [&](unsigned long ms) {
        unsigned long end = millis() + ms;
        while (millis() < end) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (checkCallback) checkCallback();
            if (abortFlag && *abortFlag) return;
        }
    };

    const auto& sensors = _sensors.list();
    for (const auto& sensorPtr : sensors) {
        if (abortFlag && *abortFlag) return false;
        if (!sensorPtr) continue;

        ISensor* sensor = sensorPtr.get();
        bool success = false;
        SensorReading reading;

        Serial.printf("[MEAS] Reading %s (addr %u)...\n", sensor->name(), sensor->address());
        for (int attempt = 1; attempt <= SENSOR_MAX_RETRIES; attempt++) {
            if (abortFlag && *abortFlag) break;

            if (sensor->read(reading)) {
                Serial.printf("[MEAS] %s OK (attempt %d)\n", sensor->name(), attempt);
                success = true;
                break;
            }

            Serial.printf("[MEAS] %s Failed (attempt %d/%d)\n", sensor->name(), attempt, SENSOR_MAX_RETRIES);
            if (attempt < SENSOR_MAX_RETRIES) {
                delayWithCheck(SENSOR_RETRY_DELAY_MS);
                if (abortFlag && *abortFlag) break;
            }
        }

        if (!success) {
            if (_lastError == "") _lastError = sensor->name();
            continue;
        }

        applyReading(reading, data);
    }

    return true;
}