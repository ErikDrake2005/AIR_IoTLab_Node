#pragma once
#include <Arduino.h>
#include "MeasurementData.h"
#include <functional>

class RS485Master;
class SHT31Sensor;
class JsonFormatter;

// Callback type để check lệnh khẩn cấp trong quá trình đo
typedef std::function<void()> UrgentCheckCallback;

class Measurement {
public:
    Measurement(RS485Master &rs485Master, SHT31Sensor &sht31Sensor, JsonFormatter &jsonFormatter);
    
    // Hàm thực hiện đo đạc, trả về false nếu có bất kỳ cảm biến nào lỗi
    bool doFullMeasurement(MeasurementData& data, bool* abortFlag = nullptr, UrgentCheckCallback checkCallback = nullptr);
    
    // Hàm wrapper tạo JSON (dùng cho debug hoặc tương thích ngược)
    bool doFullMeasurement(String& outputJson);

    // [MỚI] Hàm lấy tên lỗi gần nhất (để StateMachine gửi đi)
    String getLastError();

private:
    RS485Master& _rs485;
    SHT31Sensor& _sht31;
    JsonFormatter& _json;
    
    String _lastError; // Biến lưu tên lỗi (VD: "ch4", "mics", "sht")

    bool measureCH4(float &ch4);
    bool measureMICS(float &co, float &alc, float &nh3, float &h2);
    bool measureSHT31(float &temp, float &hum);
};