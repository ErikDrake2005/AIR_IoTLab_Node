#include "JsonFormatter.h"
#include "TimeSync.h"

extern TimeSync timeSync; 

JsonFormatter::JsonFormatter() {}

unsigned long JsonFormatter::getTimestamp() {
    return timeSync.getCurrentTime();
}
float roundVal(float val) {
    if (val < 0) return val; 
    return (int)(val * 100 + 0.5) / 100.0;
}

String JsonFormatter::createDataJson(float ch4, float co, float alc, float nh3, float h2, float temp, float hum) {
    JsonDocument doc;
    doc["type"] = "data";
    doc["timestamp"] = getTimestamp();
    
    // Sử dụng hàm roundVal thay vì công thức cũ
    doc["ch4"] = roundVal(ch4);
    doc["co"]  = roundVal(co);
    doc["alc"] = roundVal(alc);
    doc["nh3"] = roundVal(nh3);
    doc["h2"]  = roundVal(h2);
    doc["temp"] = roundVal(temp);
    doc["hum"]  = roundVal(hum);
    String output;
    serializeJson(doc, output);
    return output;
}

String JsonFormatter::createAck(const String& cmd) {
    JsonDocument doc;
    doc["type"] = "ack";
    doc["cmd"] = cmd;
    String output;
    serializeJson(doc, output);
    return output;
}

String JsonFormatter::createError(const String& msg) {
    JsonDocument doc;
    doc["type"] = "error";
    doc["msg"] = msg;
    String output;
    serializeJson(doc, output);
    return output;
}

String JsonFormatter::createTimeSyncRequest() {
    JsonDocument doc;
    doc["type"] = "time_req";
    String output;
    serializeJson(doc, output);
    return output;
}