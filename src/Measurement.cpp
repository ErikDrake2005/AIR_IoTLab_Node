#include "Measurement.h"
#include "RS485Master.h"
#include "SHT31Sensor.h"
#include "JsonFormatter.h"

Measurement::Measurement(RS485Master& rs485, SHT31Sensor& sht31, JsonFormatter& json)
    : _rs485(rs485), _sht31(sht31), _json(json) {
    _lastError = "";
}

String Measurement::getLastError() {
    return _lastError;
}

// Wrapper: Luôn tạo JSON DATA thay vì JSON ERROR
bool Measurement::doFullMeasurement(String& outputJson) {
    MeasurementData data;
    doFullMeasurement(data); 
    outputJson = _json.createDataJson(data.ch4, data.co, data.alc, data.nh3, data.h2, data.temp, data.hum);
    return true; // Luôn báo thành công để gửi đi
}

bool Measurement::doFullMeasurement(MeasurementData& data, bool* abortFlag) {
    _lastError = "";
    
    // 1. KHỞI TẠO MẶC ĐỊNH LÀ -1 (Lỗi)
    float ch4 = -1.0, co = -1.0, alc = -1.0, nh3 = -1.0, h2 = -1.0;
    float temp = -1.0, hum = -1.0;

    // --- BƯỚC 1: ĐO SHT31 ---
    Serial.println("[MEAS] Reading SHT31...");
    if (measureSHT31(temp, hum)) {
        Serial.printf("[MEAS] SHT31 OK: T=%.2f, H=%.2f\n", temp, hum);
    } else {
        Serial.println("[MEAS] ERROR: SHT31 Failed -> Val = -1");
        temp = -1.0; hum = -1.0;
        if (_lastError == "") _lastError = "sht";
    }
    
    if (abortFlag && *abortFlag) return false;
    vTaskDelay(20 / portTICK_PERIOD_MS);

    // --- BƯỚC 2: ĐO CH4 (SLAVE 1) ---
    Serial.println("[MEAS] Reading CH4 (Slave 1)...");
    if (measureCH4(ch4)) {
        Serial.printf("[MEAS] CH4 OK: %.2f\n", ch4);
    } else {
        Serial.println("[MEAS] ERROR: CH4 Failed -> Val = -1");
        ch4 = -1.0;
        if (_lastError == "") _lastError = "ch4";
    }

    if (abortFlag && *abortFlag) return false;
    vTaskDelay(50 / portTICK_PERIOD_MS);

    // --- BƯỚC 3: ĐO KHÍ ĐỘC (SLAVE 2) ---
    Serial.println("[MEAS] Reading MICS (Slave 2)...");
    if (measureMICS(co, alc, nh3, h2)) {
        Serial.printf("[MEAS] MICS OK: CO=%.2f...\n", co);
    } else {
        Serial.println("[MEAS] ERROR: MICS Failed -> Val = -1");
        co = -1.0; alc = -1.0; nh3 = -1.0; h2 = -1.0;
        if (_lastError == "") _lastError = "mics";
    }

    // Gán dữ liệu vào struct trả về
    data.ch4 = ch4;
    data.co = co; data.alc = alc; data.nh3 = nh3; data.h2 = h2;
    data.temp = temp; data.hum = hum;

    // QUAN TRỌNG: Luôn trả về true để StateMachine không dừng chu trình
    return true; 
}

// --- CÁC HÀM CON (GIỮ NGUYÊN) ---
bool Measurement::measureCH4(float &ch4) {
    if (!_rs485.sendCommand(1, 2, 1000)) return false; 
    String resp = _rs485.readResponse(2000); 
    if (resp == "") return false;
    return _rs485.parseCH4(resp, ch4);
}
bool Measurement::measureMICS(float &co, float &alc, float &nh3, float &h2) {
    if (!_rs485.sendCommand(2, 2, 1000)) return false;
    String resp = _rs485.readResponse(2000);
    if (resp == "") return false;
    return _rs485.parseMICS(resp, co, alc, nh3, h2);
}
bool Measurement::measureSHT31(float &temp, float &hum) {
    return _sht31.readData(temp, hum);
}