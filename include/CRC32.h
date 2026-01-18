#pragma once
#include <Arduino.h>

class CRC32 {
public:
    static unsigned long calculate(const char* data) {
        unsigned long crc = 0xFFFFFFFF;
        while (*data) {
            crc ^= *data++;
            for (int j = 0; j < 8; j++) {
                crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : crc >> 1;
            }
        }
        return ~crc;
    }
    
    static unsigned long calculate(const String& data) {
        return calculate(data.c_str());
    }
};