#pragma once
#include <Arduino.h>
#include <Update.h>
#include "Config.h"

class OTAHandler {
public:
    OTAHandler();
    bool startOTA(size_t totalSize);
    bool handleChunk(const uint8_t* data, size_t len);
    bool finishOTA();
    void abortOTA();
    size_t getReceived() const { return _received; }  // để in tiến độ

private:
    UpdateClass _updater;
    size_t _totalSize = 0;
    size_t _received = 0;
    unsigned long _lastChunkTime = 0;
};