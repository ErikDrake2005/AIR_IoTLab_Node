#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

class JsonFormatter {
public:
    JsonFormatter();
    unsigned long getTimestamp();
    String createDataJson(float ch4, float co, float alc, float nh3, float h2, float temp, float hum);
    String createAck(String status);
    String createMachineStatus(String mode, int chamberStatus, int doorStatus, int fanStatus, int manualCycle, int measuresPerDay, float batt);
    String createTimeSyncRequest();
    String createWakeupAck();
};