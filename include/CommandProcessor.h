#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "CommonTypes.h" // <--- Include để lấy struct CommandData chuẩn

// --- XÓA struct CommandData CŨ Ở ĐÂY NẾU CÓ ---

class CommandProcessor {
public:
    CommandProcessor();
    
    // Hàm parse trả về CommandData (định nghĩa trong CommonTypes.h)
    CommandData parse(const String& rawInput);

private:
    bool verifyCRC(const String& rawInput, String& jsonOutput);
};

#endif