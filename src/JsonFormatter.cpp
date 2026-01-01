#include "JsonFormatter.h"
#include "TimeSync.h"

extern TimeSync timeSync; 
JsonFormatter::JsonFormatter() {}
unsigned long JsonFormatter::getTimestamp() {
    return timeSync.getCurrentTime();
}
String JsonFormatter::createDataJson(float ch4, float co, float alc, float nh3, float h2, float temp, float hum) {
    JsonDocument doc;
    doc["type"] = "data";
    doc["timestamp"] = getTimestamp();
    doc["ch4"] = (int)(ch4 * 100 + 0.5) / 100.0;
    doc["co"]  = (int)(co * 100 + 0.5) / 100.0;
    doc["alc"] = (int)(alc * 100 + 0.5) / 100.0;
    doc["nh3"] = (int)(nh3 * 100 + 0.5) / 100.0;
    doc["h2"]  = (int)(h2 * 100 + 0.5) / 100.0;
    doc["temp"] = (int)(temp * 100 + 0.5) / 100.0;
    doc["hum"]  = (int)(hum * 100 + 0.5) / 100.0;
    
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