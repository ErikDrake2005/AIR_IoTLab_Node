#include "CommandProcessor.h"
#include "CRC32.h"

CommandProcessor::CommandProcessor() {}

bool CommandProcessor::verifyCRC(const String& rawInput, String& jsonOutput) {
    int pipeIndex = rawInput.lastIndexOf('|');
    if (pipeIndex == -1) {
        jsonOutput = rawInput;
        return true;
    }

    String dataPart = rawInput.substring(0, pipeIndex);
    String crcPart = rawInput.substring(pipeIndex + 1);
    crcPart.trim();
    
    uint32_t calcCRC = CRC32::calculate(dataPart);
    uint32_t recvCRC = strtoul(crcPart.c_str(), NULL, 16);

    if (calcCRC == recvCRC) {
        jsonOutput = dataPart;
        return true;
    }
    
    Serial.printf("[CMD] CRC mismatch: calc=0x%08X, recv=0x%08X\n", calcCRC, recvCRC);
    return false;
}

CommandData CommandProcessor::parse(const String& rawInput) {
    CommandData cmd; 
    cmd.enable = true; 
    
    String jsonStr;

    // 1. Verify CRC
    if (!verifyCRC(rawInput, jsonStr)) {
        Serial.println("[CMD] CRC verification failed");
        cmd.isValid = false; 
        return cmd; 
    }

    // 2. Deserialize JSON
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (error) {
        Serial.printf("[CMD] JSON parse error: %s\n", error.c_str());
        cmd.isValid = false;
        return cmd;
    }

    cmd.isValid = true;

    // 3. Parse Mode "set"
    if (!doc["set"].isNull()) {
        String s = doc["set"].as<String>();
        s.trim();
        s.toUpperCase(); 
        
        if (s == "MANUAL") cmd.setMode = MODE_MANUAL;
        else if (s == "AUTO") cmd.setMode = MODE_AUTO;
        else if (s == "TIMESTAMP") cmd.setMode = MODE_TIMESTAMP;
        else if (s == "SLEEP") {
            cmd.setMode = MODE_SLEEP;
            cmd.enable = false;
        }
        
        Serial.printf("[CMD] Mode parsed: %s\n", s.c_str());
    }

    // 4. Parse chi tiết lệnh trong "cmd"
    JsonObject rootObj = doc.as<JsonObject>();
    JsonObject cmdObj;

    if (!rootObj["cmd"].isNull()) {
        if (cmd.setMode == MODE_TIMESTAMP) {
            if (rootObj["cmd"].is<unsigned long>()) {
                cmd.timestamp = rootObj["cmd"].as<unsigned long>();
                Serial.printf("[CMD] Timestamp: %lu\n", cmd.timestamp);
            } else if (rootObj["cmd"].is<long long>()) {
                cmd.timestamp = (unsigned long)rootObj["cmd"].as<long long>();
                Serial.printf("[CMD] Timestamp (ll): %lu\n", cmd.timestamp);
            }
        }
        // Nếu là Object (cho Manual/Auto)
        else if (rootObj["cmd"].is<JsonObject>()) {
            cmdObj = rootObj["cmd"];
        }
    }

    // 5. Parse nội dung của cmdObj
    if (!cmdObj.isNull()) {
        // transmissionIntervalMinutes (cho MANUAL mode)
        if (!cmdObj["transmissionIntervalMinutes"].isNull()) {
            int interval = cmdObj["transmissionIntervalMinutes"].as<int>();
            if (interval > 0 && interval <= 59) {
                cmd.manualInterval = interval;
                Serial.printf("[CMD] Manual interval: %d minutes\n", interval);
            }
        }

        // Hành động cụ thể trong khóa "do"
        if (!cmdObj["do"].isNull() && cmdObj["do"].is<JsonObject>()) {
            JsonObject doObj = cmdObj["do"];
            cmd.hasActions = true;
            
            // measurementCount (cho AUTO mode)
            if (!doObj["measurementCount"].isNull()) {
                int count = doObj["measurementCount"].as<int>();
                if (count > 0) {
                    cmd.autoMeasureCount = count;
                    Serial.printf("[CMD] Measure count: %d\n", count);
                }
            }
            
            // startTime (cho AUTO mode)
            if (!doObj["startTime"].isNull()) {
                cmd.autoStartTime = doObj["startTime"].as<String>();
                int colonIndex = cmd.autoStartTime.indexOf(':');
                if (colonIndex > 0) {
                    int hour = cmd.autoStartTime.substring(0, colonIndex).toInt();
                    int minute = cmd.autoStartTime.substring(colonIndex + 1).toInt();
                    if (hour >= 0 && hour < 24 && minute >= 0 && minute < 60) {
                        cmd.startTimeSeconds = hour * 3600 + minute * 60;
                        Serial.printf("[CMD] Start time: %s (%d sec)\n", 
                                      cmd.autoStartTime.c_str(), cmd.startTimeSeconds);
                    }
                }
            }

            // chamberStatus (start/stop measurement)
            if (!doObj["chamberStatus"].isNull()) {
                cmd.chamberStatus = doObj["chamberStatus"].as<String>();
                cmd.chamberStatus.trim();
                cmd.chamberStatus.toLowerCase();
                Serial.printf("[CMD] Chamber status: %s\n", cmd.chamberStatus.c_str());
            }
            
            // doorStatus (open/close)
            if (!doObj["doorStatus"].isNull()) {
                cmd.doorStatus = doObj["doorStatus"].as<String>();
                cmd.doorStatus.trim();
                cmd.doorStatus.toLowerCase();
                Serial.printf("[CMD] Door status: %s\n", cmd.doorStatus.c_str());
            }
            
            // fanStatus (on/off)
            if (!doObj["fanStatus"].isNull()) {
                cmd.fanStatus = doObj["fanStatus"].as<String>();
                cmd.fanStatus.trim();
                cmd.fanStatus.toLowerCase();
                Serial.printf("[CMD] Fan status: %s\n", cmd.fanStatus.c_str());
            }
        }
    }

    return cmd;
}