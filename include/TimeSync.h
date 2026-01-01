#pragma once
#include <Arduino.h>
#include <sys/time.h>

class JsonFormatter;

class TimeSync {
public:
    TimeSync(JsonFormatter& json);
    void begin();
    void updateEpoch(unsigned long epoch);                
    void syncFromTimestamp(const String& timeStr); 
    
    unsigned long getCurrentTime();
    void requestTimeSync();
    String getRequestJson();

private:
    unsigned long _parseTimestamp(const String& timestamp); 
    unsigned long _epochBase = 0;
    uint32_t _syncMillis = 0;
    JsonFormatter& _json;
    const unsigned long MAX_EPOCH_JUMP_SECONDS = 360; 
};