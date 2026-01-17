#include "CommandProcessor.h"
#include "CRC32.h"

CommandProcessor::CommandProcessor() {}

bool CommandProcessor::verifyCRC(const String& rawInput, String& jsonOutput) {
    int pipeIndex = rawInput.lastIndexOf('|');
    if (pipeIndex == -1) return false;

    String dataPart = rawInput.substring(0, pipeIndex);
    String crcPart = rawInput.substring(pipeIndex + 1);
    crcPart.trim();
    
    uint32_t calcCRC = CRC32::calculate(dataPart);
    uint32_t recvCRC = strtoul(crcPart.c_str(), NULL, 16);

    if (calcCRC == recvCRC) {
        jsonOutput = dataPart;
        return true;
    }
    return false;
}

CommandData CommandProcessor::parse(const String& rawInput) {
    CommandData cmd; 
    
    // Mặc định THỨC (Enable = true) vì Bridge đã lọc
    cmd.enable = true; 
    
    String jsonStr;

    // 1. Verify CRC
    if (!verifyCRC(rawInput, jsonStr)) {
        cmd.isValid = false; 
        return cmd; 
    }

    // 2. Deserialize JSON
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (error) {
        cmd.isValid = false;
        return cmd;
    }

    cmd.isValid = true;

    // 3. Parse Mode "set"
    if (!doc["set"].isNull()) {
        String s = doc["set"].as<String>();
        if (s == "MANUAL") cmd.setMode = MODE_MANUAL;
        else if (s == "AUTO") cmd.setMode = MODE_AUTO;
        else if (s == "TIMESTAMP") cmd.setMode = MODE_TIMESTAMP;
        else if (s == "SLEEP") {
            cmd.setMode = MODE_SLEEP;
            cmd.enable = false; // Chỉ ngủ khi lệnh là SLEEP
        }
    }

    // 4. Parse chi tiết lệnh (Sửa lỗi containsKey)
    JsonObject rootObj = doc.as<JsonObject>();
    JsonObject cmdObj;

    // Thay containsKey bằng cách kiểm tra !isNull()
    if (!rootObj["cmd"].isNull()) {
        // Nếu mode là TIMESTAMP thì cmd là số
        if (cmd.setMode == MODE_TIMESTAMP && rootObj["cmd"].is<unsigned long>()) {
             cmd.timestamp = rootObj["cmd"].as<unsigned long>();
        }
        // Nếu là Object (cho Manual/Auto)
        else if (rootObj["cmd"].is<JsonObject>()) {
             cmdObj = rootObj["cmd"];
        }
    } else {
        cmdObj = rootObj; // Fallback
    }

    if (!cmdObj.isNull()) {
        // Manual Interval
        if (!cmdObj["transmissionIntervalMinutes"].isNull()) {
             cmd.manualInterval = cmdObj["transmissionIntervalMinutes"].as<int>();
        }

        // Hành động cụ thể (DO)
        JsonObject doObj = cmdObj["do"];
        if (!doObj.isNull()) {
            cmd.hasActions = true;
            
            if (!doObj["measurementCount"].isNull()) cmd.autoMeasureCount = doObj["measurementCount"].as<int>();
            if (!doObj["startTime"].isNull()) {
                cmd.autoStartTime = doObj["startTime"].as<String>();
                int colonIndex = cmd.autoStartTime.indexOf(':');
                if (colonIndex > 0) {
                    int hour = cmd.autoStartTime.substring(0, colonIndex).toInt();
                    int minute = cmd.autoStartTime.substring(colonIndex + 1).toInt();
                    cmd.startTimeSeconds = hour * 3600 + minute * 60;
                }
            }

            if (!doObj["chamberStatus"].isNull()) cmd.chamberStatus = doObj["chamberStatus"].as<String>();
            if (!doObj["doorStatus"].isNull())    cmd.doorStatus = doObj["doorStatus"].as<String>();
            if (!doObj["fanStatus"].isNull())     cmd.fanStatus = doObj["fanStatus"].as<String>();
        }
    }

    return cmd;
}