#pragma once
#include <Arduino.h>
#include "MeasurementData.h"
#include <functional>

class RS485Master;
class SHT31Sensor;
class JsonFormatter;

typedef std::function<void()> UrgentCheckCallback;

class Measurement {
public:
    Measurement(RS485Master &rs485Master, SHT31Sensor &sht31Sensor, JsonFormatter &jsonFormatter);
    bool doFullMeasurement(MeasurementData& data, bool* abortFlag = nullptr, UrgentCheckCallback checkCallback = nullptr);
    bool doFullMeasurement(String& outputJson);
    String getLastError();

private:
    RS485Master& _rs485;
    SHT31Sensor& _sht31;
    JsonFormatter& _json;
    String _lastError;
    bool measureCH4(float &ch4);
    bool measureMICS(float &co, float &alc, float &nh3, float &h2);
    bool measureSHT31(float &temp, float &hum);
};