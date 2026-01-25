#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

class JsonFormatter {
public:
    JsonFormatter();
    unsigned long getTimestamp();

    // { "type": "data", "content": { ... } }
    String createDataJson(float ch4, float co, float alc, float nh3, float h2, float temp, float hum);
    String createAck(String status);
    // { "type": "machine_status", "content": { ... } }
    // chamberStatus: 1=measuring, 0=stop | doorStatus: 1=open, 0=close | fanStatus: 1=on, 0=off
    String createMachineStatus(String mode, int chamberStatus, int doorStatus, int fanStatus, int manualCycle, int measuresPerDay, float batt);
    
    // { "type": "time_req", "content": null }
    String createTimeSyncRequest();
    
    // { "WakeUp": "Done" } -> Đặc biệt cho Handshake
    String createWakeupAck();
};