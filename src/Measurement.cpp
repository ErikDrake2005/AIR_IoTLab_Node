#include "Measurement.h"
#include "RS485Master.h"
#include "SHT31Sensor.h"
#include "JsonFormatter.h"

Measurement::Measurement(RS485Master& rs485, SHT31Sensor& sht31, JsonFormatter& json)
    : _rs485(rs485), _sht31(sht31), _json(json) {
    _lastError = "";
}

// Hàm helper trả về chuỗi lỗi
String Measurement::getLastError() {
    return _lastError;
}

bool Measurement::doFullMeasurement(String& outputJson) {
    MeasurementData data;
    // Gọi hàm đo chính
    bool success = doFullMeasurement(data); 
    
    if (success) {
        // Nếu thành công -> Tạo JSON dữ liệu
        outputJson = _json.createDataJson(data.ch4, data.co, data.alc, data.nh3, data.h2, data.temp, data.hum);
        return true;
    } else {
        // Nếu thất bại -> Tạo JSON báo lỗi (Sử dụng _lastError vừa cập nhật)
        outputJson = _json.createError(_lastError);
        return false;
    }
}

bool Measurement::doFullMeasurement(MeasurementData& data, bool* abortFlag) {
    // Reset lỗi cũ
    _lastError = "";
    bool isSuccess = true;

    // Khởi tạo giá trị an toàn (NaN để biết là không đo được, hoặc 0 tuỳ nhu cầu)
    // Ở đây dùng 0 như code cũ, nhưng đánh dấu lỗi
    float ch4 = 0, co = 0, alc = 0, nh3 = 0, h2 = 0, temp = 0, hum = 0;
    
    // --- BƯỚC 1: ĐO CH4 (SLAVE 1) ---
    Serial.println("[MEAS] Reading CH4 (Slave 1)...");
    if (abortFlag && *abortFlag) return false;
    
    if (measureCH4(ch4)) {
        Serial.printf("[MEAS] Got CH4: %.2f\n", ch4);
    } else {
        Serial.println("[MEAS] ERROR: Slave 1 (CH4) failed");
        _lastError = "ch4"; // Gán lỗi cụ thể
        isSuccess = false;  // Đánh dấu quy trình đo có lỗi
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // --- BƯỚC 2: ĐO KHÍ ĐỘC (SLAVE 2) ---
    Serial.println("[MEAS] Reading MICS (Slave 2)...");
    if (abortFlag && *abortFlag) return false;

    if (measureMICS(co, alc, nh3, h2)) {
        Serial.printf("[MEAS] Got MICS: CO=%.2f, ALC=%.2f...\n", co, alc);
    } else {
        Serial.println("[MEAS] ERROR: Slave 2 (MICS) failed");
        // Chỉ ghi đè lỗi nếu trước đó chưa có lỗi (ưu tiên lỗi đầu tiên phát hiện)
        if (_lastError == "") _lastError = "mics"; 
        isSuccess = false;
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);

    // --- BƯỚC 3: ĐO SHT31 (I2C) ---
    Serial.println("[MEAS] Reading SHT31...");
    if (measureSHT31(temp, hum)) {
        Serial.printf("[MEAS] Got SHT31: T=%.2f, H=%.2f\n", temp, hum);
    } else {
        Serial.println("[MEAS] ERROR: SHT31 failed");
        if (_lastError == "") _lastError = "sht";
        isSuccess = false;
    }

    // --- LƯU KẾT QUẢ ---
    // Vẫn lưu các giá trị đo được (kể cả khi isSuccess = false, có thể một số sens vẫn chạy)
    data.ch4 = ch4;
    data.co = co; data.alc = alc; data.nh3 = nh3; data.h2 = h2;
    data.temp = temp; data.hum = hum;

    // Trả về false nếu có BẤT KỲ lỗi nào
    return isSuccess; 
}

// --- CÁC HÀM CON (GIỮ NGUYÊN LOGIC, CHỈ SỬA NHỎ LOG) ---

bool Measurement::measureCH4(float &ch4) {
    if (!_rs485.sendCommand(1, 2, 1000)) {
        // Serial.println("[RS485] Slave 1: No ACK"); -> Đã log bên class RS485 nếu cần
        return false; 
    }
    String resp = _rs485.readResponse(3000);
    if (resp == "") return false;
    return _rs485.parseCH4(resp, ch4);
}

bool Measurement::measureMICS(float &co, float &alc, float &nh3, float &h2) {
    if (!_rs485.sendCommand(2, 2, 1000)) return false;
    String resp = _rs485.readResponse(3000);
    if (resp == "") return false;
    return _rs485.parseMICS(resp, co, alc, nh3, h2);
}

bool Measurement::measureSHT31(float &temp, float &hum) {
    return _sht31.readData(temp, hum);
}