#include "Measurement.h"
#include "RS485Master.h"
#include "SHT31Sensor.h"
#include "JsonFormatter.h"

Measurement::Measurement(RS485Master& rs485, SHT31Sensor& sht31, JsonFormatter& json)
    : _rs485(rs485), _sht31(sht31), _json(json) {}

bool Measurement::doFullMeasurement(String& outputJson) {
    MeasurementData data;
    doFullMeasurement(data); 
    outputJson = _json.createDataJson(data.ch4, data.co, data.alc, data.nh3, data.h2, data.temp, data.hum);
    return true;
}
bool Measurement::doFullMeasurement(MeasurementData& data, bool* abortFlag) {
    // 1. Khởi tạo tất cả bằng 0 trước
    float ch4 = 0, co = 0, alc = 0, nh3 = 0, h2 = 0, temp = 0, hum = 0;
    
    // --- BƯỚC 1: ĐO CH4 (SLAVE 1) ---
    Serial.println("[MEAS] Reading CH4 (Slave 1)...");
    if (abortFlag && *abortFlag) return false;
    
    if (measureCH4(ch4)) {
        Serial.printf("[MEAS] Got CH4: %.2f\n", ch4);
    } else {
        Serial.println("[MEAS] ERROR: Slave 1 (CH4) failed/timeout -> Default 0");
        ch4 = 0; // Gán cứng bằng 0 nếu lỗi
    }
    
    // Delay nhỏ giữa các lần đọc để tránh nhiễu đường truyền
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // --- BƯỚC 2: ĐO KHÍ ĐỘC (SLAVE 2) ---
    Serial.println("[MEAS] Reading MICS (Slave 2)...");
    if (abortFlag && *abortFlag) return false;

    if (measureMICS(co, alc, nh3, h2)) {
        Serial.printf("[MEAS] Got MICS: CO=%.2f, ALC=%.2f...\n", co, alc);
    } else {
        Serial.println("[MEAS] ERROR: Slave 2 (MICS) failed/timeout -> Default 0");
        co = 0; alc = 0; nh3 = 0; h2 = 0;
    }

    // Delay nhỏ
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // --- BƯỚC 3: ĐO SHT31 (I2C) ---
    Serial.println("[MEAS] Reading SHT31...");
    if (measureSHT31(temp, hum)) {
        Serial.printf("[MEAS] Got SHT31: T=%.2f, H=%.2f\n", temp, hum);
    } else {
        Serial.println("[MEAS] ERROR: SHT31 failed -> Default 0");
        temp = 0; hum = 0;
    }

    // --- LƯU KẾT QUẢ ---
    data.ch4 = ch4;
    data.co = co; data.alc = alc; data.nh3 = nh3; data.h2 = h2;
    data.temp = temp; data.hum = hum;
    return true; 
}
// Measurement.cpp

bool Measurement::measureCH4(float &ch4) {
    // Gửi CMD 2, Timeout ACK = 1000ms
    if (!_rs485.sendCommand(1, 2, 1000)) {
        Serial.println("[RS485] Slave 1: Send CMD 2 failed (No ACK)");
        return false; 
    }
    
    // Chờ DATA, Timeout = 3000ms (cho chắc)
    String resp = _rs485.readResponse(3000);
    if (resp == "") {
        Serial.println("[RS485] Slave 1: Data timeout");
        return false;
    }
    return _rs485.parseCH4(resp, ch4);
}
bool Measurement::measureMICS(float &co, float &alc, float &nh3, float &h2) {
    // Gửi CMD 2, Timeout ACK = 1000ms
    if (!_rs485.sendCommand(2, 2, 1000)) {
        Serial.println("[RS485] Slave 2: Send CMD 2 failed (No ACK)");
        return false;
    }
    
    // Chờ DATA, Timeout = 3000ms
    String resp = _rs485.readResponse(3000);
    if (resp == "") {
        Serial.println("[RS485] Slave 2: Data timeout");
        return false;
    }

    return _rs485.parseMICS(resp, co, alc, nh3, h2);
}
bool Measurement::measureSHT31(float &temp, float &hum) {
    return _sht31.readData(temp, hum);
}