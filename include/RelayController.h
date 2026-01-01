#pragma once
#include <Arduino.h>
#include "config.h"

class RelayController {
public:
    RelayController();
    void begin();
    void ON_DOOR();
    void OFF_DOOR();
    void ON_FAN();
    void OFF_FAN();
    void ON();
    void OFF();
};