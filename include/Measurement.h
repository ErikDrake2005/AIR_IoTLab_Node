#pragma once
#include <Arduino.h>
#include "MeasurementData.h"
#include "Sensor.h"
#include <functional>

class JsonFormatter;

typedef std::function<void()> UrgentCheckCallback;

class Measurement {
public:
    Measurement(SensorRegistry& sensors, JsonFormatter& jsonFormatter);
    bool doFullMeasurement(MeasurementData& data, bool* abortFlag = nullptr, UrgentCheckCallback checkCallback = nullptr);
    bool doFullMeasurement(String& outputJson);
    String getLastError();

private:
    SensorRegistry& _sensors;
    JsonFormatter& _json;
    String _lastError;
};