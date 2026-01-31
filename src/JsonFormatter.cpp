#include "JsonFormatter.h"
#include "TimeSync.h"

extern TimeSync timeSync; 

JsonFormatter::JsonFormatter() {}
String JsonFormatter::createAck(String status) {
    JsonDocument doc;
    doc["type"] = "ACK";
    doc["status"] = status;
    
    String output;
    serializeJson(doc, output);
    return output;
}
unsigned long JsonFormatter::getTimestamp() {
    return timeSync.getCurrentTime();
}

float round2(float val) {
    if (val < 0) return val; 
    return (int)(val * 100 + 0.5) / 100.0;
}

String JsonFormatter::createDataJson(float ch4, float co, float alc, float nh3, float h2, float temp, float hum) {
    JsonDocument doc;
    doc["type"] = "data";
    JsonObject content = doc["content"].to<JsonObject>();
    content["temp"] = round2(temp);
    content["hum"] = round2(hum);
    content["co"] = round2(co);
    content["nh3"] = round2(nh3);
    content["h2"] = round2(h2);
    content["c2h5oh"] = round2(alc); 
    content["ch4"] = round2(ch4);
    content["timestamp"] = getTimestamp();
    
    String output;
    serializeJson(doc, output);
    return output;
}

String JsonFormatter::createMachineStatus(String mode, int chamberStatus, int doorStatus, int fanStatus, int manualCycle, int measuresPerDay, float batt) {
    JsonDocument doc;
    doc["type"] = "machine_status";
    JsonObject content = doc["content"].to<JsonObject>();
    content["mode"] = mode;                 
    content["chamberStatus"] = chamberStatus; 
    content["doorStatus"] = doorStatus; 
    content["fanStatus"] = fanStatus; 
    content["saved_manual_cycle"] = manualCycle;
    content["saved_daily_meansure"] = measuresPerDay;
    content["timestamp"] = getTimestamp();
    String output;
    serializeJson(doc, output);
    return output;
}

String JsonFormatter::createTimeSyncRequest() {
    JsonDocument doc;
    doc["type"] = "time_req";
    doc["content"] = nullptr;
    String output;
    serializeJson(doc, output);
    return output;
}

String JsonFormatter::createWakeupAck() {
    JsonDocument doc;
    doc["WakeUp"] = "Done";
    String output;
    serializeJson(doc, output);
    return output;
}