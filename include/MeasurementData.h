#pragma once
#include <Arduino.h>
struct MeasurementData {
    float co2;   // ppm
    float ch4;   // %LEL
    float temp;  // degrees Celsius
    float hum;   // %RH
};
