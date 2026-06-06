#include "Measurement.h"
#include "RS485Master.h"
#include "JsonFormatter.h"
#include "Config.h"

const int SENSOR_MAX_RETRIES = 2; //Retry Count
const unsigned long SENSOR_RETRY_DELAY_MS = 3000;

Measurement::Measurement(RS485Master& rs485, JsonFormatter& json)
    : _rs485(rs485), _json(json) {
    _lastError = "";
}

String Measurement::getLastError() {
    return _lastError;
}

// Wrapper: Luôn tạo JSON DATA thay vì JSON ERROR
bool Measurement::doFullMeasurement(String& outputJson) {
    MeasurementData data;
    doFullMeasurement(data); 
    outputJson = _json.createDataJson(data.co2, data.ch4, data.temp, data.hum);
    return true; 
}

bool Measurement::doFullMeasurement(MeasurementData& data, bool* abortFlag, UrgentCheckCallback checkCallback) {
    _lastError = "";
    auto delayWithCheck = [&](unsigned long ms) {
        unsigned long end = millis() + ms;
        while (millis() < end) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (checkCallback) checkCallback();
            if (abortFlag && *abortFlag) return;
        }
    };
    float ch4 = -1.0, co2 = -1.0;
    float temp = -1.0, hum = -1.0;
    Serial.println("[MEAS] Reading EP32-SW (Slave 3)...");
    bool tempHumSuccess = false;
    for (int attempt = 1; attempt <= SENSOR_MAX_RETRIES; attempt++) {
        if (abortFlag && *abortFlag) break;
        
        if (measureEP32SW(temp, hum)) {
            Serial.printf("[MEAS] EP32-SW OK (attempt %d): T=%.2f, H=%.2f\n", attempt, temp, hum);
            tempHumSuccess = true;
            break;
        }
        
        Serial.printf("[MEAS] EP32-SW Failed (attempt %d/%d)\n", attempt, SENSOR_MAX_RETRIES);
        if (attempt < SENSOR_MAX_RETRIES) {
            delayWithCheck(SENSOR_RETRY_DELAY_MS);
            if (abortFlag && *abortFlag) break;
        }
    }
    
    if (!tempHumSuccess) {
        Serial.println("[MEAS] ERROR: EP32-SW Failed after retries -> Val = -1");
        temp = -1.0; hum = -1.0;
        if (_lastError == "") _lastError = "th";
    }
    
    if (abortFlag && *abortFlag) return false;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println("[MEAS] Reading CH4 (Slave 1)...");
    bool ch4Success = false;
    for (int attempt = 1; attempt <= SENSOR_MAX_RETRIES; attempt++) {
        if (abortFlag && *abortFlag) break;
        
        if (measureCH4(ch4)) {
            Serial.printf("[MEAS] CH4 OK (attempt %d): %.2f\n", attempt, ch4);
            ch4Success = true;
            break;
        }
        
        Serial.printf("[MEAS] CH4 Failed (attempt %d/%d)\n", attempt, SENSOR_MAX_RETRIES);
        if (attempt < SENSOR_MAX_RETRIES) {
            delayWithCheck(SENSOR_RETRY_DELAY_MS);
            if (abortFlag && *abortFlag) break;
        }
    }
    
    if (!ch4Success) {
        Serial.println("[MEAS] ERROR: CH4 Failed after retries -> Val = -1");
        ch4 = -1.0;
        if (_lastError == "") _lastError = "ch4";
    }

    if (abortFlag && *abortFlag) return false;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println("[MEAS] Reading CO2 (Slave 2)...");
    bool co2Success = false;
    for (int attempt = 1; attempt <= SENSOR_MAX_RETRIES; attempt++) {
        if (abortFlag && *abortFlag) break;
        
        if (measureCO2(co2)) {
            Serial.printf("[MEAS] CO2 OK (attempt %d): CO2=%.2f\n", attempt, co2);
            co2Success = true;
            break;
        }
        
        Serial.printf("[MEAS] CO2 Failed (attempt %d/%d)\n", attempt, SENSOR_MAX_RETRIES);
        if (attempt < SENSOR_MAX_RETRIES) {
            delayWithCheck(SENSOR_RETRY_DELAY_MS);
            if (abortFlag && *abortFlag) break;
        }
    }
    
    if (!co2Success) {
        Serial.println("[MEAS] ERROR: CO2 Failed after retries -> Val = -1");
        co2 = -1.0;
        if (_lastError == "") _lastError = "co2";
    }
    data.co2 = co2;
    data.ch4 = ch4;
    data.temp = temp; data.hum = hum;
    return true; 
}
bool Measurement::measureCH4(float &ch4) {
    uint16_t value = 0;
    if (!_rs485.readHoldingRegister(CH4_SLAVE_ID, CH4_DATA_REG, value, 2000)) return false;
    ch4 = static_cast<float>(value);
    return true;
}
bool Measurement::measureCO2(float &co2) {
    uint16_t value = 0;
    if (!_rs485.readHoldingRegister(CO2_SLAVE_ID, CO2_DATA_REG, value, 2000)) return false;
    co2 = static_cast<float>(value);
    return true;
}
bool Measurement::measureEP32SW(float &temp, float &hum) {
    uint16_t tempRaw = 0;
    uint16_t humRaw = 0;
    if (!_rs485.readHoldingRegister(EP32SW_SLAVE_ID, EP32SW_TEMP_REG, tempRaw, 2000)) return false;
    if (!_rs485.readHoldingRegister(EP32SW_SLAVE_ID, EP32SW_HUM_REG, humRaw, 2000)) return false;
    // EP32-SW reports temp/hum scaled by 10 -> divide by 10 to get real value
    temp = static_cast<float>(tempRaw) / 10.0f;
    hum = static_cast<float>(humRaw) / 10.0f;
    return true;
}