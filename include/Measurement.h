#pragma once
#include <Arduino.h>
#include "MeasurementData.h"
#include <functional>

class RS485Master;
class JsonFormatter;

typedef std::function<void()> UrgentCheckCallback;

class Measurement {
public:
    Measurement(RS485Master &rs485Master, JsonFormatter &jsonFormatter);
    bool doFullMeasurement(MeasurementData& data, bool* abortFlag = nullptr, UrgentCheckCallback checkCallback = nullptr);
    bool doFullMeasurement(String& outputJson);
    String getLastError();

private:
    RS485Master& _rs485;
    JsonFormatter& _json;
    String _lastError;
    bool measureCH4(float &ch4);
    bool measureCO2(float &co2);
    bool measureEP32SW(float &temp, float &hum);
};