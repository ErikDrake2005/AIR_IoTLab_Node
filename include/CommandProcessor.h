#ifndef COMMAND_PROCESSOR_H
#define COMMAND_PROCESSOR_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "CommonTypes.h"

class CommandProcessor {
public:
    CommandProcessor();
    CommandData parse(const String& rawInput);

private:
    bool verifyCRC(const String& rawInput, String& jsonOutput);
};

#endif