#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

/**
 * CommandProcessor - Priority-based command processing for Node
 * 
 * Processes commands received from Gateway->Bridge->Node with strict priority:
 * 1. EN (Sleep command) - Highest priority
 * 2. Time Sync (from Gateway)
 * 3. Mode (MANUAL/AUTO)
 * 4. Time Functions (schedules, time grid, manual cycle)
 * 5. Measurement Start/Stop (MANUAL mode only)
 * 6. Relay Control (door/fan) (MANUAL mode only)
 * 
 * All commands are collected in a single JSON, then processed sequentially
 * by priority. After all commands execute, machine_status is sent.
 */

struct CommandData {
    // Priority 0: Special
    bool hasEnSleep;
    int enValue;  // 0 = sleep, 1 = wake
    
    // Priority 1: Time Sync
    bool hasTimeSync;
    unsigned long epochTime;
    
    // Priority 2: Mode
    bool hasMode;
    String mode;  // "auto" or "manual"
    
    // Priority 3: Time Functions
    bool hasCycleManual;
    int cycleManualMin;
    bool hasMeasuresPerDay;
    int measuresPerDay;
    bool hasSchedules;
    JsonArray schedulesArray;
    
    // Priority 4: Measurement Control
    bool hasMeasurement;
    String measureCmd;  // "start" or "stop"
    
    // Priority 5: Relay Control
    bool hasDoor;
    String doorCmd;     // "open" or "close"
    bool hasFan;
    String fanCmd;      // "on" or "off"
};

class CommandProcessor {
public:
    CommandProcessor();
    
    /**
     * Parse incoming JSON command and extract all command fields
     * Handles both single commands and batched commands
     * 
     * @param jsonStr Full JSON string from Bridge
     * @return CommandData struct with all parsed fields
     */
    static CommandData parseCommand(const String& jsonStr);
    
    /**
     * Check if command is valid and contains known fields
     * 
     * @param doc ArduinoJson document
     * @return true if contains any valid command fields
     */
    static bool isValidCommand(const JsonDocument& doc);
    
    /**
     * Get priority order for processing
     * Ensures commands execute in correct sequence
     */
    static const uint8_t* getPriorityOrder();
    
private:
    /**
     * Helper to safely extract string value from JSON
     */
    static String extractString(const JsonDocument& doc, const char* key, const String& defaultVal = "");
    
    /**
     * Helper to safely extract int value from JSON
     */
    static int extractInt(const JsonDocument& doc, const char* key, int defaultVal = 0);
};
