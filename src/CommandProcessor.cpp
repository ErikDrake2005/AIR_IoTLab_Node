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
    CommandData cmd; // Struct từ CommonTypes.h
    String jsonStr;

    // 1. Verify CRC
    if (!verifyCRC(rawInput, jsonStr)) {
        return cmd; // isValid mặc định là false
    }

    // 2. Deserialize JSON
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (error) return cmd;

    cmd.isValid = true;

    // 3. --- BẮT ĐẦU PARSE THEO CẤU TRÚC MỚI ---
    
    // 3.1 Lấy NID và Enable (Lớp ngoài cùng)
    // [FIX LỖI WARNING]: Thay containsKey("en") bằng !doc["en"].isNull()
    if (!doc["en"].isNull()) {
        cmd.enable = (doc["en"].as<int>() == 1);
    }
    // Lưu ý: Node thực ra không cần quan tâm NID vì Bridge đã lọc rồi.

    // 3.2 Vào lớp "req" (Yêu cầu)
    JsonObject req = doc["req"];
    if (req.isNull()) {
        // Nếu không có req, có thể chỉ là lệnh ngủ từ Bridge (en=0)
        // Nếu enable = false (0) -> Chuyển sang mode SLEEP
        if (cmd.enable == false) cmd.setMode = MODE_SLEEP;
        return cmd;
    }

    // 3.3 Lấy "set" (Chế độ)
    String setStr = req["set"].as<String>();
    if (setStr == "AUTO") cmd.setMode = MODE_AUTO;
    else if (setStr == "MANUAL") cmd.setMode = MODE_MANUAL;
    else if (setStr == "TIMESTAMP") cmd.setMode = MODE_TIMESTAMP;
    else if (setStr == "SLEEP") cmd.setMode = MODE_SLEEP;

    // 3.4 Lấy "cmd" (Tham số chi tiết)
    JsonVariant cmdVar = req["cmd"];
    
    // --- TRƯỜNG HỢP 1: TIMESTAMP (cmd là số long) ---
    if (cmd.setMode == MODE_TIMESTAMP) {
        if (cmdVar.is<unsigned long>()) {
            cmd.timestamp = cmdVar.as<unsigned long>();
        }
        return cmd; // Xong
    }

    // --- TRƯỜNG HỢP 2: AUTO / MANUAL (cmd là Object) ---
    if (cmdVar.is<JsonObject>()) {
        JsonObject cmdObj = cmdVar.as<JsonObject>();

        // Lấy cấu hình chung
        if (!cmdObj["transmissionIntervalMinutes"].isNull()) {
            cmd.manualInterval = cmdObj["transmissionIntervalMinutes"].as<int>();
        }

        // Vào lớp "do" (Hành động)
        JsonObject doObj = cmdObj["do"];
        if (!doObj.isNull()) {
            cmd.hasActions = true;
            
            // Lệnh Auto
            if (!doObj["measurementCount"].isNull()) {
                cmd.autoMeasureCount = doObj["measurementCount"].as<int>();
            }
            if (!doObj["startTime"].isNull()) {
                cmd.autoStartTime = doObj["startTime"].as<String>();
                
                // === PARSE startTime "HH:MM" -> seconds ===
                int colonIndex = cmd.autoStartTime.indexOf(':');
                if (colonIndex > 0) {
                    int hour = cmd.autoStartTime.substring(0, colonIndex).toInt();
                    int minute = cmd.autoStartTime.substring(colonIndex + 1).toInt();
                    cmd.startTimeSeconds = hour * 3600 + minute * 60;
                    Serial.printf("[CMD] Parsed startTime %s -> %u seconds\n", 
                                  cmd.autoStartTime.c_str(), cmd.startTimeSeconds);
                }
            }

            // Lệnh Manual / Tức thời
            if (!doObj["chamberStatus"].isNull()) {
                cmd.chamberStatus = doObj["chamberStatus"].as<String>();
            }
            if (!doObj["doorStatus"].isNull()) {
                cmd.doorStatus = doObj["doorStatus"].as<String>();
            }
            if (!doObj["fanStatus"].isNull()) {
                cmd.fanStatus = doObj["fanStatus"].as<String>();
            }
        }
    }

    return cmd;
}