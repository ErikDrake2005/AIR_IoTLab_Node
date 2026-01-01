#pragma once
#include <Arduino.h>
class CRC32 {
public:
    static unsigned long calculate(const String& data) {
        unsigned long crc = 0xFFFFFFFF;
        for (size_t i = 0; i < data.length(); i++) {
            char c = data[i];
            crc ^= c;
            for (size_t j = 0; j < 8; j++) {
                if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
                else crc >>= 1;
            }
        }
        return ~crc;
    }
};