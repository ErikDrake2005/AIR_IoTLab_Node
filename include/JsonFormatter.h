#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

class JsonFormatter {
public:
    JsonFormatter();
    unsigned long getTimestamp();
    String createDataJson(float co2, float ch4, float temp, float hum);
    String createAck(String status);
    String createMachineStatus(String mode, int chamberStatus, int doorStatus, int fanStatus, int manualCycle, int measuresPerDay, float batt);
    String createTimeSyncRequest();
    String createWakeupAck();
};