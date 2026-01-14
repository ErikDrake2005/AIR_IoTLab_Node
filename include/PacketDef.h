#pragma once
#include <Arduino.h>
#include <ArduinoJson.h> 

// Cấu trúc hàng đợi gửi xuống LoRa
typedef struct { uint8_t payload[256]; size_t length; uint8_t nodeId; } LoraQueueMsg;

struct PacketHeader { uint8_t nodeId; uint32_t counter; };

enum DataType : uint8_t { DT_END=0, DT_KEY_TOKEN=1, DT_VAL_TOKEN=2, DT_VAL_INT8=3, DT_VAL_INT16=4, DT_VAL_INT32=5, DT_VAL_FLOAT=6, DT_VAL_RAW_STR=7 };

// [QUAN TRỌNG] Dictionary phải khớp 100% với Bridge
const char* const DICTIONARY[] = {
    "type", "cmd", "msg", "value", "error", "timestamp", "id",
    "EN", "ack_rec", "batt", "bridge_volt", "poll", "awake", "sleep", "running", "status", "info",
    "set_state", "set_door", "set_fans", "set_time", 
    "mode", "cycle_manual", "measures_per_day", "device", "device_id", "target_id", 
    "temp", "hum", "ch4", "co", "nh3", "h2", "alc", "rssi", "node_id", "mics",
    "ack", "data", "manual", "auto", "measure", "stop", 
    "trigger_measure", "stop_measure", "open", "close", "on", "off",
    "MEASURE_STARTED", "MEASURE_DONE", "SLEEP_CONFIRMED",
    "machine_status", "machine_sync", "time_sync", "time_req", "retry",
    "door_status", "fan_status", "measuring", "Pin", "Sleep_Mode"
};

class PacketUtils {
public:
    static const char* getString(uint8_t token) {
        if (token < sizeof(DICTIONARY)/sizeof(DICTIONARY[0])) return DICTIONARY[token];
        return nullptr;
    }

    static int getToken(const char* str) {
        for (size_t i = 0; i < sizeof(DICTIONARY)/sizeof(DICTIONARY[0]); i++) {
            if (strcmp(DICTIONARY[i], str) == 0) return i;
        }
        return -1;
    }

    // --- ENCODER (JSON -> BINARY) ---
    static int encodeJsonToBinary(JsonDocument& doc, uint8_t* buffer, int maxLen) {
        int idx = 0;
        JsonObject obj = doc.as<JsonObject>();
        for (JsonPair p : obj) {
            // Key
            int kTok = getToken(p.key().c_str());
            if (kTok != -1) { buffer[idx++] = DT_KEY_TOKEN; buffer[idx++] = kTok; }
            else return 0; // Key không có trong từ điển -> Lỗi

            // Value
            if (p.value().is<int>()) {
                int32_t val = p.value().as<int32_t>();
                if (val >= -128 && val <= 127) { buffer[idx++] = DT_VAL_INT8; buffer[idx++] = (int8_t)val; }
                else if (val >= -32768 && val <= 32767) { buffer[idx++] = DT_VAL_INT16; memcpy(buffer+idx, &val, 2); idx+=2; }
                else { buffer[idx++] = DT_VAL_INT32; memcpy(buffer+idx, &val, 4); idx+=4; }
            }
            else if (p.value().is<float>()) {
                float val = p.value().as<float>();
                buffer[idx++] = DT_VAL_FLOAT; memcpy(buffer+idx, &val, 4); idx+=4;
            }
            else if (p.value().is<const char*>()) {
                const char* s = p.value().as<const char*>();
                int vTok = getToken(s);
                if (vTok != -1) { buffer[idx++] = DT_VAL_TOKEN; buffer[idx++] = vTok; }
                else { 
                    buffer[idx++] = DT_VAL_RAW_STR; 
                    uint8_t l = strlen(s); 
                    buffer[idx++] = l; memcpy(buffer+idx, s, l); idx+=l; 
                }
            }
        }
        buffer[idx++] = DT_END; return idx;
    }

    // --- DECODER (BINARY -> JSON) ---
    static void decodeBinaryToJson(uint8_t* buffer, int len, JsonDocument& doc) {
        int idx = 0; const char* currentKey = nullptr;
        while (idx < len) {
            uint8_t type = buffer[idx++];
            if (type == DT_END) break;
            
            if (type == DT_KEY_TOKEN) { 
                currentKey = getString(buffer[idx++]); 
                continue; 
            }
            if (!currentKey) continue; 

            if (type == DT_VAL_TOKEN) doc[currentKey] = getString(buffer[idx++]);
            else if (type == DT_VAL_INT8) doc[currentKey] = (int8_t)buffer[idx++];
            else if (type == DT_VAL_INT16) { int16_t v; memcpy(&v, buffer+idx, 2); idx+=2; doc[currentKey] = v; }
            else if (type == DT_VAL_INT32) { int32_t v; memcpy(&v, buffer+idx, 4); idx+=4; doc[currentKey] = v; }
            else if (type == DT_VAL_FLOAT) { float v; memcpy(&v, buffer+idx, 4); idx+=4; doc[currentKey] = v; }
            else if (type == DT_VAL_RAW_STR) { 
                uint8_t l = buffer[idx++]; 
                char s[256]; memcpy(s, buffer+idx, l); s[l] = 0; 
                doc[currentKey] = s; idx+=l; 
            }
        }
    }
};
