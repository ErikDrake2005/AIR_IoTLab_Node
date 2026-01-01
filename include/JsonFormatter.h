#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include "Config.h"

class JsonFormatter {
public:
    JsonFormatter();
    String createDataJson(float ch4, float co, float alc, float nh3, float h2, float temp, float hum);
    String createAck(const String& cmd);
    String createError(const String& msg);
    String createTimeSyncRequest();

private:
    unsigned long getTimestamp();
};