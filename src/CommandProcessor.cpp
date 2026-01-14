#include "CommandProcessor.h"

CommandProcessor::CommandProcessor() {
}

CommandData CommandProcessor::parseCommand(const String& jsonStr) {
    CommandData cmd = {};
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonStr);
    if (error) {
        Serial.printf("[CMD] JSON Parse Error: %s\n", error.c_str());
        return cmd;
    }
    
    // Priority 0: EN (Sleep/Wake)
    if (!doc["EN"].isNull()) {
        cmd.hasEnSleep = true;
        cmd.enValue = doc["EN"].as<int>();
    }
    
    // Priority 1: Time Sync (timestamp)
    if (!doc["timestamp"].isNull()) {
        cmd.hasTimeSync = true;
        cmd.epochTime = doc["timestamp"].as<unsigned long>();
    }
    
    // Priority 2: Mode
    if (!doc["mode"].isNull()) {
        cmd.hasMode = true;
        cmd.mode = doc["mode"].as<String>();
    } else if (!doc["set_mode"].isNull()) {
        cmd.hasMode = true;
        cmd.mode = doc["set_mode"].as<String>();
    }
    
    // Priority 3: Time Functions
    if (!doc["cycle_manual"].isNull()) {
        cmd.hasCycleManual = true;
        cmd.cycleManualMin = doc["cycle_manual"].as<int>();
    }
    
    if (!doc["measures_per_day"].isNull()) {
        cmd.hasMeasuresPerDay = true;
        cmd.measuresPerDay = doc["measures_per_day"].as<int>();
    }
    
    if (!doc["schedules"].isNull() && doc["schedules"].is<JsonArray>()) {
        cmd.hasSchedules = true;
        cmd.schedulesArray = doc["schedules"].as<JsonArray>();
    }
    
    // Priority 4: Measurement Control
    if (!doc["set_state"].isNull()) {
        String state = doc["set_state"].as<String>();
        if (state == "measure" || state == "start") {
            cmd.hasMeasurement = true;
            cmd.measureCmd = "start";
        } else if (state == "stop") {
            cmd.hasMeasurement = true;
            cmd.measureCmd = "stop";
        }
    }
    
    // Priority 5: Relay Control - Door
    if (!doc["set_door"].isNull()) {
        cmd.hasDoor = true;
        cmd.doorCmd = doc["set_door"].as<String>();
    } else if (!doc["open"].isNull() && doc["open"].as<int>() == 1) {
        cmd.hasDoor = true;
        cmd.doorCmd = "open";
    } else if (!doc["close"].isNull() && doc["close"].as<int>() == 1) {
        cmd.hasDoor = true;
        cmd.doorCmd = "close";
    }
    
    // Priority 5: Relay Control - Fan
    if (!doc["set_fans"].isNull()) {
        cmd.hasFan = true;
        cmd.fanCmd = doc["set_fans"].as<String>();
    } else if (!doc["fan"].isNull()) {
        cmd.hasFan = true;
        cmd.fanCmd = (doc["fan"].as<int>() == 1) ? "on" : "off";
    } else if (!doc["set_fan"].isNull()) {
        cmd.hasFan = true;
        cmd.fanCmd = doc["set_fan"].as<String>();
    }
    
    return cmd;
}

bool CommandProcessor::isValidCommand(const JsonDocument& doc) {
    return (!doc["EN"].isNull() ||
            !doc["timestamp"].isNull() ||
            !doc["mode"].isNull() ||
            !doc["set_mode"].isNull() ||
            !doc["cycle_manual"].isNull() ||
            !doc["measures_per_day"].isNull() ||
            !doc["schedules"].isNull() ||
            !doc["set_state"].isNull() ||
            !doc["set_door"].isNull() ||
            !doc["open"].isNull() ||
            !doc["close"].isNull() ||
            !doc["set_fans"].isNull() ||
            !doc["set_fan"].isNull() ||
            !doc["fan"].isNull());
}

String CommandProcessor::extractString(const JsonDocument& doc, const char* key, const String& defaultVal) {
    if (!doc[key].isNull()) {
        return doc[key].as<String>();
    }
    return defaultVal;
}

int CommandProcessor::extractInt(const JsonDocument& doc, const char* key, int defaultVal) {
    if (!doc[key].isNull()) {
        return doc[key].as<int>();
    }
    return defaultVal;
}
