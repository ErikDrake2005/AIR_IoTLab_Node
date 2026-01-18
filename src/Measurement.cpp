#include "Measurement.h"
#include "RS485Master.h"
#include "SHT31Sensor.h"
#include "JsonFormatter.h"

// --- CẤU HÌNH RETRY CHO CẢM BIẾN ---
const int SENSOR_MAX_RETRIES = 2;           // Số lần thử tối đa
const unsigned long SENSOR_RETRY_DELAY_MS = 3000; // Timeout 3 giây giữa các lần thử

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

bool Measurement::doFullMeasurement(MeasurementData& data, bool* abortFlag, UrgentCheckCallback checkCallback) {
    _lastError = "";
    
    // Helper lambda để chờ với check abort và callback
    auto delayWithCheck = [&](unsigned long ms) {
        unsigned long end = millis() + ms;
        while (millis() < end) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (checkCallback) checkCallback();
            if (abortFlag && *abortFlag) return;
        }
    };
    
    // 1. KHỞI TẠO MẶC ĐỊNH LÀ -1 (Lỗi)
    float ch4 = -1.0, co = -1.0, alc = -1.0, nh3 = -1.0, h2 = -1.0;
    float temp = -1.0, hum = -1.0;

    // --- BƯỚC 1: ĐO SHT31 (với retry) ---
    Serial.println("[MEAS] Reading SHT31...");
    bool sht31Success = false;
    for (int attempt = 1; attempt <= SENSOR_MAX_RETRIES; attempt++) {
        if (abortFlag && *abortFlag) break;
        
        if (measureSHT31(temp, hum)) {
            Serial.printf("[MEAS] SHT31 OK (attempt %d): T=%.2f, H=%.2f\n", attempt, temp, hum);
            sht31Success = true;
            break;
        }
        
        Serial.printf("[MEAS] SHT31 Failed (attempt %d/%d)\n", attempt, SENSOR_MAX_RETRIES);
        if (attempt < SENSOR_MAX_RETRIES) {
            delayWithCheck(SENSOR_RETRY_DELAY_MS);
            if (abortFlag && *abortFlag) break;
        }
    }
    
    if (!sht31Success) {
        Serial.println("[MEAS] ERROR: SHT31 Failed after retries -> Val = -1");
        temp = -1.0; hum = -1.0;
        if (_lastError == "") _lastError = "sht";
    }
    
    if (abortFlag && *abortFlag) return false;
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // --- BƯỚC 2: ĐO CH4 (SLAVE 1) (với retry) ---
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

    // --- BƯỚC 3: ĐO KHÍ ĐỘC MICS5524 (SLAVE 2) (với retry) ---
    Serial.println("[MEAS] Reading MICS (Slave 2)...");
    bool micsSuccess = false;
    for (int attempt = 1; attempt <= SENSOR_MAX_RETRIES; attempt++) {
        if (abortFlag && *abortFlag) break;
        
        if (measureMICS(co, alc, nh3, h2)) {
            Serial.printf("[MEAS] MICS OK (attempt %d): CO=%.2f, NH3=%.2f, H2=%.2f, C2H5OH=%.2f\n", 
                          attempt, co, nh3, h2, alc);
            micsSuccess = true;
            break;
        }
        
        Serial.printf("[MEAS] MICS Failed (attempt %d/%d)\n", attempt, SENSOR_MAX_RETRIES);
        if (attempt < SENSOR_MAX_RETRIES) {
            delayWithCheck(SENSOR_RETRY_DELAY_MS);
            if (abortFlag && *abortFlag) break;
        }
    }
    
    if (!micsSuccess) {
        Serial.println("[MEAS] ERROR: MICS Failed after retries -> Val = -1");
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