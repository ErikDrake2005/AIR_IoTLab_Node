// include/SHT31Sensor.h
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include "Config.h"

class SHT31Sensor {
public:
    SHT31Sensor();
    bool begin();
    bool readData(float &temperature, float &humidity);

private:
    Adafruit_SHT31 _sht31;
    bool _initialized=false;
};