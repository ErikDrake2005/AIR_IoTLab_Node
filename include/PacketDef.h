#pragma once
#include <Arduino.h>
#include <ArduinoJson.h> 

// Cấu trúc hàng đợi gửi xuống LoRa
typedef struct { uint8_t payload[512]; size_t length; uint8_t nodeId; } LoraQueueMsg;

// Header gói tin LoRa (5 bytes)
struct PacketHeader { uint8_t nodeId; uint32_t counter; };
// FIXED-SCHEMA PACKET TYPES 
enum FixedPacketType : uint8_t {
    PKT_UPLINK_DATA      = 0x01,  // Sensor data (23 bytes)
    PKT_UPLINK_STATUS    = 0x02,  // Machine status (13 bytes)
    PKT_UPLINK_TIME_REQ  = 0x03,  // Time request (9 bytes)
    PKT_DOWNLINK_TIME    = 0x80,  // Time sync response (5 bytes)
    PKT_DOWNLINK_CMD     = 0x81,  // Command (legacy dictionary encoding)
    PKT_LEGACY           = 0xFF,  // Fallback to dictionary encoding
};

// ── UPLINK_DATA: Sensor readings (21 bytes total) ──
// Removed rssi/snr, added isSleeping
#pragma pack(push, 1)
struct UplinkDataPacket {
    uint8_t  type;       // = PKT_UPLINK_DATA (0x01)
    uint8_t  deviceId;   // Bridge ID
    uint16_t pinMv;      // Battery voltage in mV
    uint8_t  isSleeping; // 0=Node awake, 1=Node sleeping
    int16_t  tempX100;   // Temperature x100 (e.g., 2750 = 27.50°C)
    int16_t  humX100;    // Humidity x100 (e.g., 6500 = 65.00%)
    int16_t  ch4;        // CH4 ppm
    int16_t  co;         // CO ppm
    int16_t  nh3;        // NH3 ppm
    int16_t  h2;         // H2 ppm
    int16_t  c2h5oh;     // C2H5OH (alcohol) ppm
    uint32_t timestamp;  // Unix epoch (local time)
};
#pragma pack(pop)

// ── UPLINK_STATUS: Machine status (11 bytes total) ──
// Removed rssi/snr, added isSleeping
#pragma pack(push, 1)
struct UplinkStatusPacket {
    uint8_t  type;  
    uint8_t  deviceId; 
    uint16_t pinMv;  
    uint8_t  isSleeping; 
    uint8_t  flags; 
    uint8_t  manualCycle;  
    uint8_t  dailyMeasures; 
    uint32_t timestamp; 
};
#pragma pack(pop)

// Flags bits definition for UplinkStatusPacket
#define STATUS_FLAG_MODE_MANUAL   0x01  // bit0: 0=AUTO, 1=MANUAL
#define STATUS_FLAG_MEASURING     0x02  // bit1: 1=measuring in progress
#define STATUS_FLAG_DOOR_OPEN     0x04  // bit2: 1=door open
#define STATUS_FLAG_FAN_ON        0x08  // bit3: 1=fan on

// ── UPLINK_TIME_REQ: Time sync request (5 bytes total) ──
// Removed rssi/snr
#pragma pack(push, 1)
struct UplinkTimeReqPacket {
    uint8_t  type;       // = PKT_UPLINK_TIME_REQ (0x03)
    uint8_t  deviceId;   // Bridge ID
    uint16_t pinMv;      // Battery voltage in mV
    uint8_t  isSleeping; // 0=Node awake, 1=Node sleeping
};
#pragma pack(pop)

// ── DOWNLINK_TIME: Time sync response (5 bytes total) ──
#pragma pack(push, 1)
struct DownlinkTimePacket {
    uint8_t  type;       // = PKT_DOWNLINK_TIME (0x80)
    uint32_t epoch;      // Unix epoch (local time from Gateway)
};
#pragma pack(pop)

// ── DOWNLINK_CMD: Command packet (10 bytes total) ──
#pragma pack(push, 1)
struct DownlinkCmdPacket {
    uint8_t  type; // = PKT_DOWNLINK_CMD (0x81)
    uint8_t  targetId; // 0=ALL, 1-255=specific Bridge ID
    uint8_t  en; // 0=sleep, 1=execute
    uint8_t  setMode; // 0=AUTO, 1=MANUAL, 2=TIMESTAMP, 3=SLEEP
    uint8_t  intervalMin; // transmissionIntervalMinutes (0=null, 1-59=value)
    uint8_t  measureCount; // measurementCount for AUTO (0=null)
    uint16_t startTimeMin; // startTime as minutes from 00:00 (0xFFFF=null)
    uint8_t  doFlags; // MANUAL: bit0-1=chamber, bit2-3=door, bit4-5=fan
};
#pragma pack(pop)

// ── DOWNLINK_CMD doFlags encoding ──
// For MANUAL mode actions (2 bits each: 0=null/ignore, 1=stop/close/off, 2=start/open/on)
#define CMD_FLAG_CHAMBER_MASK    0x03  // bit0-1
#define CMD_FLAG_CHAMBER_NULL    0x00
#define CMD_FLAG_CHAMBER_STOP    0x01  // stop-measurement
#define CMD_FLAG_CHAMBER_START   0x02  // start-measurement
#define CMD_FLAG_DOOR_MASK  0x0C  // bit2-3
#define CMD_FLAG_DOOR_NULL       0x00
#define CMD_FLAG_DOOR_CLOSE      0x04  // close
#define CMD_FLAG_DOOR_OPEN       0x08  // open
#define CMD_FLAG_FAN_MASK        0x30  // bit4-5
#define CMD_FLAG_FAN_NULL        0x00
#define CMD_FLAG_FAN_OFF         0x10  // off
#define CMD_FLAG_FAN_ON          0x20  // on

// ── setMode values ──
#define CMD_MODE_AUTO      0
#define CMD_MODE_MANUAL    1
#define CMD_MODE_TIMESTAMP 2
#define CMD_MODE_SLEEP     3

// LEGACY DICTIONARY ENCODING (Vẫn giữ cho commands và backward compat)
enum DataType : uint8_t { 
    DT_END=0, DT_KEY_TOKEN=1, DT_VAL_TOKEN=2, DT_VAL_INT8=3, DT_VAL_INT16=4, 
    DT_VAL_INT32=5, DT_VAL_FLOAT=6, DT_VAL_RAW_STR=7, DT_OBJ_START=8, DT_OBJ_END=9, DT_NULL=10        
};

// [TỪ ĐIỂN] CẬP NHẬT ĐẦY ĐỦ CÁC TỪ KHÓA CỦA GIAO THỨC V2
const char* const DICTIONARY[] = {
    // 0-9: Protocol Keys
    "type", "NID", "en", "req", "set", "cmd", "do", "content", "node", "id",
    
    // 10-19: Identification & Hardware
    "device_ID", "device", "target_id", "node_id", "timestamp", "pin", "rssi", "batt", "bridge_volt", "Pin",
    
    // 20-29: Values & Status
    "AUTO", "MANUAL", "TIMESTAMP", "SLEEP", "ack", "error", "msg", "value", "info", "status",
    
    // 30-39: Node States
    "awake", "sleep", "running", "poll", "connected", "disconnected", "open", "close", "on", "off",
    
    // 40-49: Actions & Controls
    "start", "stop", "trigger_measure", "stop_measure", "set_state", "set_door", "set_fans", "WakeUp", "MEASURE_STARTED", "ack_rec",
    
    // 50-59: Sensors
    "temp", "hum", "ch4", "co", "nh3", "h2", "alc", "c2h5oh", "measuring", "measure_status",
    
    // 60+: NEW KEYS FOR V2 (BẮT BUỘC PHẢI CÓ)
    "transmissionIntervalMinutes", 
    "measurementCount",            
    "startTime",                   
    "cycle_manual",                
    "measures_per_day",            
    "schedules",
    "chamberStatus",               
    "doorStatus",                  
    "fanStatus",
    "saved_manual_cycle",          
    "saved_daily_meansure",        
    "Sleep_Mode",                  
    "machine_status",              
    "time_req",                    
    "time_res",                    
    "data",
    "gw_rssi",
    "gw_snr",
    
    // 78+: MACHINE STATUS KEYS (THIẾU - ĐÃ THÊM)
    "mode",
    "door",
    "fan",
    "snr",
    
    NULL // Điểm kết thúc
};

class PacketUtils {
public:
    static const char* getString(uint8_t token) {
        int count = 0; while(DICTIONARY[count] != NULL) count++;
        if (token >= count) return nullptr;
        return DICTIONARY[token];
    }

    // Encoder (JSON -> Binary)
    static int encodeJsonToBinary(JsonDocument& doc, uint8_t* buffer, int maxLen) {
        int idx = 0;
        JsonObject root = doc.as<JsonObject>();
        serializeObject(root, buffer, idx, maxLen);
        buffer[idx++] = DT_END; 
        return idx;
    }

    // Decoder (Binary -> JSON)
    static void decodeBinaryToJson(uint8_t* buffer, int len, JsonDocument& doc) {
        int idx = 0;
        JsonObject root = doc.to<JsonObject>();
        deserializeObject(root, buffer, idx, len);
    }

private:
    static void serializeObject(JsonObject obj, uint8_t* buffer, int& idx, int maxLen) {
        for (JsonPair kv : obj) {
            if (idx >= maxLen - 10) return; 
            const char* key = kv.key().c_str();
            int keyToken = -1;
            for (int i = 0; DICTIONARY[i] != NULL; i++) {
                if (strcmp(key, DICTIONARY[i]) == 0) { keyToken = i; break; }
            }
            if (keyToken != -1) { 
                buffer[idx++] = DT_KEY_TOKEN; buffer[idx++] = (uint8_t)keyToken; 
            } else continue; 

            JsonVariant v = kv.value();
            if (v.is<JsonObject>()) {
                buffer[idx++] = DT_OBJ_START; serializeObject(v.as<JsonObject>(), buffer, idx, maxLen); buffer[idx++] = DT_OBJ_END;
            } else if (v.isNull()) { buffer[idx++] = DT_NULL; }
            else if (v.is<int>()) {
                int32_t val = v.as<int32_t>();
                if (val >= -128 && val <= 127) { buffer[idx++] = DT_VAL_INT8; buffer[idx++] = (int8_t)val; } 
                else if (val >= -32768 && val <= 32767) { buffer[idx++] = DT_VAL_INT16; int16_t x = (int16_t)val; memcpy(buffer+idx, &x, 2); idx+=2; } 
                else { buffer[idx++] = DT_VAL_INT32; memcpy(buffer+idx, &val, 4); idx+=4; }
            } else if (v.is<float>()) { buffer[idx++] = DT_VAL_FLOAT; float f = v.as<float>(); memcpy(buffer+idx, &f, 4); idx+=4; }
            else if (v.is<const char*>()) {
                const char* s = v.as<const char*>();
                int valToken = -1;
                for (int i = 0; DICTIONARY[i] != NULL; i++) { if (strcmp(s, DICTIONARY[i]) == 0) { valToken = i; break; } }
                if (valToken != -1) { buffer[idx++] = DT_VAL_TOKEN; buffer[idx++] = (uint8_t)valToken; } 
                else { buffer[idx++] = DT_VAL_RAW_STR; uint8_t l = strlen(s); if(l>200) l=200; buffer[idx++] = l; memcpy(buffer+idx, s, l); idx+=l; }
            }
        }
    }

    static void deserializeObject(JsonObject obj, uint8_t* buffer, int& idx, int len) {
        const char* currentKey = nullptr;
        while (idx < len) {
            uint8_t type = buffer[idx++];
            if (type == DT_END) return;
            if (type == DT_OBJ_END) return;
            if (type == DT_KEY_TOKEN) { uint8_t k = buffer[idx++]; currentKey = getString(k); continue; }
            if (!currentKey) continue;
            if (type == DT_OBJ_START) { JsonObject nested = obj[currentKey].to<JsonObject>(); deserializeObject(nested, buffer, idx, len); }
            else if (type == DT_NULL) { obj[currentKey] = (char*)nullptr; }
            else if (type == DT_VAL_TOKEN) { uint8_t v = buffer[idx++]; obj[currentKey] = getString(v); }
            else if (type == DT_VAL_INT8) { obj[currentKey] = (int8_t)buffer[idx++]; }
            else if (type == DT_VAL_INT16) { int16_t v; memcpy(&v, buffer+idx, 2); idx+=2; obj[currentKey] = v; }
            else if (type == DT_VAL_INT32) { int32_t v; memcpy(&v, buffer+idx, 4); idx+=4; obj[currentKey] = v; }
            else if (type == DT_VAL_FLOAT) { float v; memcpy(&v, buffer+idx, 4); idx+=4; obj[currentKey] = v; }
            else if (type == DT_VAL_RAW_STR) { uint8_t l = buffer[idx++]; char t[256]; memcpy(t, buffer+idx, l); t[l]=0; idx+=l; obj[currentKey] = t; }
        }
    }
};

// ═══════════════════════════════════════════════════════════════════════
// FIXED-SCHEMA ENCODER/DECODER (Dùng cho gói tin thường xuyên)
// ═══════════════════════════════════════════════════════════════════════
class FixedPacket {
public:
    // ── BRIDGE: Encode Uplink DATA packet (Node JSON → Fixed Binary) ──
    // Input: Parsed JSON from Node + Bridge metadata (deviceId, pin, isSleeping)
    // Output: Fixed-schema binary packet (21 bytes)
    static int encodeUplinkData(JsonDocument& nodeDoc, uint8_t deviceId, uint16_t pinMv, 
                                 bool nodeSleeping, uint8_t* buffer) {
        UplinkDataPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        
        pkt.type = PKT_UPLINK_DATA;
        pkt.deviceId = deviceId;
        pkt.pinMv = pinMv;
        pkt.isSleeping = nodeSleeping ? 1 : 0;
        
        // Extract sensor data from "content" object
        JsonObject content = nodeDoc["content"];
        pkt.tempX100 = (int16_t)(content["temp"].as<float>() * 100);
        pkt.humX100  = (int16_t)(content["hum"].as<float>() * 100);
        pkt.ch4      = content["ch4"].as<int16_t>();
        pkt.co       = content["co"].as<int16_t>();
        pkt.nh3      = content["nh3"].as<int16_t>();
        pkt.h2       = content["h2"].as<int16_t>();
        pkt.c2h5oh   = content["c2h5oh"].as<int16_t>();
        pkt.timestamp = content["timestamp"].as<uint32_t>();
        
        memcpy(buffer, &pkt, sizeof(pkt));
        return sizeof(pkt);
    }
    
    // ── BRIDGE: Encode Uplink STATUS packet (Node JSON → Fixed Binary) ──
    static int encodeUplinkStatus(JsonDocument& nodeDoc, uint8_t deviceId, uint16_t pinMv,
                                   bool nodeSleeping, uint8_t* buffer) {
        UplinkStatusPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        
        pkt.type = PKT_UPLINK_STATUS;
        pkt.deviceId = deviceId;
        pkt.pinMv = pinMv;
        pkt.isSleeping = nodeSleeping ? 1 : 0;
        
        // Extract status from "content" object
        JsonObject content = nodeDoc["content"];
        
        // Build flags byte
        pkt.flags = 0;
        const char* mode = content["mode"] | "AUTO";
        if (strcmp(mode, "MANUAL") == 0) pkt.flags |= STATUS_FLAG_MODE_MANUAL;
        
        // chamberStatus: 1=measuring, 0=stop
        int chamberStatus = content["chamberStatus"] | 0;
        if (chamberStatus == 1) pkt.flags |= STATUS_FLAG_MEASURING;
        
        // doorStatus: 1=open, 0=close
        int doorStatus = content["doorStatus"] | 0;
        if (doorStatus == 1) pkt.flags |= STATUS_FLAG_DOOR_OPEN;
        
        // fanStatus: 1=on, 0=off
        int fanStatus = content["fanStatus"] | 0;
        if (fanStatus == 1) pkt.flags |= STATUS_FLAG_FAN_ON;
        
        pkt.manualCycle = content["saved_manual_cycle"] | 0;
        pkt.dailyMeasures = content["saved_daily_meansure"] | 0;
        pkt.timestamp = content["timestamp"].as<uint32_t>();
        
        memcpy(buffer, &pkt, sizeof(pkt));
        return sizeof(pkt);
    }
    
    // ── BRIDGE: Encode Uplink TIME_REQ packet ──
    static int encodeUplinkTimeReq(uint8_t deviceId, uint16_t pinMv, bool nodeSleeping, uint8_t* buffer) {
        UplinkTimeReqPacket pkt;
        pkt.type = PKT_UPLINK_TIME_REQ;
        pkt.deviceId = deviceId;
        pkt.pinMv = pinMv;
        pkt.isSleeping = nodeSleeping ? 1 : 0;
        
        memcpy(buffer, &pkt, sizeof(pkt));
        return sizeof(pkt);
    }
    
    // ── GATEWAY: Encode Downlink TIME_SYNC packet ──
    static int encodeDownlinkTime(uint32_t epoch, uint8_t* buffer) {
        DownlinkTimePacket pkt;
        pkt.type = PKT_DOWNLINK_TIME;
        pkt.epoch = epoch;
        
        memcpy(buffer, &pkt, sizeof(pkt));
        return sizeof(pkt);
    }
    
    // ── GATEWAY: Encode Downlink CMD packet from JSON ──
    // Supports both formats:
    // Format 1 (MQTT): {"NID":"NODE_01","en":1,"req":{"set":"MANUAL","cmd":{...}}}
    // Format 2 (direct): {"NID":"NODE_01","en":1,"set":"MANUAL","cmd":{...}}
    static int encodeDownlinkCmd(JsonDocument& doc, uint8_t* buffer) {
        DownlinkCmdPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        
        pkt.type = PKT_DOWNLINK_CMD;
        
        // targetId = NID (from "NID" field - string to int)
        const char* nid = doc["NID"] | "1";
        // Parse NODE_XX format
        const char* lastUnderscore = strrchr(nid, '_');
        if (lastUnderscore) {
            pkt.targetId = (uint8_t)atoi(lastUnderscore + 1);
        } else {
            pkt.targetId = (uint8_t)atoi(nid);
        }
        
        // en field (0 = sleep/disable, 1 = execute)
        pkt.en = doc["en"] | 1;
        
        // If en = 0, just send SLEEP command
        if (pkt.en == 0) {
            pkt.setMode = CMD_MODE_SLEEP;
            memcpy(buffer, &pkt, sizeof(pkt));
            return sizeof(pkt);
        }
        
        // Support both formats: check if "req" exists (MQTT format)
        JsonObject reqObj;
        if (doc["req"].is<JsonObject>()) {
            reqObj = doc["req"].as<JsonObject>();
        } else {
            reqObj = doc.as<JsonObject>();  // Direct format
        }
        
        // Parse "set" field from req object
        const char* setMode = reqObj["set"] | "";
        JsonObject cmd = reqObj["cmd"];
        
        if (strcmp(setMode, "AUTO") == 0) {
            pkt.setMode = CMD_MODE_AUTO;
            
            JsonObject doObj = cmd["do"];
            
            // measurementCount
            if (!doObj["measurementCount"].isNull()) {
                pkt.measureCount = doObj["measurementCount"].as<uint8_t>();
            }
            
            // startTime (parse "HH:MM" format)
            if (!doObj["startTime"].isNull()) {
                const char* timeStr = doObj["startTime"] | "";
                int hour = 0, minute = 0;
                sscanf(timeStr, "%d:%d", &hour, &minute);
                pkt.startTimeMin = hour * 60 + minute;
            } else {
                pkt.startTimeMin = 0xFFFF;  // null marker
            }
        }
        else if (strcmp(setMode, "MANUAL") == 0) {
            pkt.setMode = CMD_MODE_MANUAL;
            
            JsonObject doObj = cmd["do"];
            
            // transmissionIntervalMinutes
            if (!cmd["transmissionIntervalMinutes"].isNull()) {
                pkt.intervalMin = cmd["transmissionIntervalMinutes"].as<uint8_t>();
            }
            
            // chamberStatus
            if (!doObj["chamberStatus"].isNull()) {
                const char* chamber = doObj["chamberStatus"] | "";
                if (strstr(chamber, "start")) {
                    pkt.doFlags |= CMD_FLAG_CHAMBER_START;
                } else if (strstr(chamber, "stop")) {
                    pkt.doFlags |= CMD_FLAG_CHAMBER_STOP;
                }
            }
            
            // doorStatus
            if (!doObj["doorStatus"].isNull()) {
                const char* door = doObj["doorStatus"] | "";
                if (strcmp(door, "open") == 0) {
                    pkt.doFlags |= CMD_FLAG_DOOR_OPEN;
                } else if (strcmp(door, "close") == 0) {
                    pkt.doFlags |= CMD_FLAG_DOOR_CLOSE;
                }
            }
            
            // fanStatus
            if (!doObj["fanStatus"].isNull()) {
                const char* fan = doObj["fanStatus"] | "";
                if (strcmp(fan, "ON") == 0 || strcmp(fan, "on") == 0) {
                    pkt.doFlags |= CMD_FLAG_FAN_ON;
                } else if (strcmp(fan, "OFF") == 0 || strcmp(fan, "off") == 0) {
                    pkt.doFlags |= CMD_FLAG_FAN_OFF;
                }
            }
        }
        else if (strcmp(setMode, "SLEEP") == 0) {
            pkt.setMode = CMD_MODE_SLEEP;
        }
        else if (strcmp(setMode, "TIMESTAMP") == 0) {
            pkt.setMode = CMD_MODE_TIMESTAMP;
        }
        
        memcpy(buffer, &pkt, sizeof(pkt));
        return sizeof(pkt);
    }
    
    // ── GATEWAY: Decode Uplink packets to JSON (for MQTT) ──
    static bool decodeUplinkToJson(uint8_t* buffer, int len, JsonDocument& doc) {
        if (len < 1) return false;
        
        uint8_t type = buffer[0];
        
        // Helper to format device_ID as "NODE_XX"
        auto formatDeviceId = [](uint8_t id, char* buf, int bufSize) {
            snprintf(buf, bufSize, "NODE_%02d", id);
        };
        char deviceIdStr[16];
        
        switch (type) {
            case PKT_UPLINK_DATA: {
                if (len < (int)sizeof(UplinkDataPacket)) return false;
                UplinkDataPacket* pkt = (UplinkDataPacket*)buffer;
                
                formatDeviceId(pkt->deviceId, deviceIdStr, sizeof(deviceIdStr));
                doc["device_ID"] = deviceIdStr;
                doc["pin"] = pkt->pinMv;
                doc["isSleeping"] = pkt->isSleeping ? true : false;
                
                JsonObject node = doc["node"].to<JsonObject>();
                node["type"] = "data";
                
                JsonObject content = node["content"].to<JsonObject>();
                content["temp"] = pkt->tempX100 / 100.0f;
                content["hum"] = pkt->humX100 / 100.0f;
                content["ch4"] = pkt->ch4;
                content["co"] = pkt->co;
                content["nh3"] = pkt->nh3;
                content["h2"] = pkt->h2;
                content["c2h5oh"] = pkt->c2h5oh;
                content["timestamp"] = pkt->timestamp;
                return true;
            }
            
            case PKT_UPLINK_STATUS: {
                if (len < (int)sizeof(UplinkStatusPacket)) return false;
                UplinkStatusPacket* pkt = (UplinkStatusPacket*)buffer;
                
                formatDeviceId(pkt->deviceId, deviceIdStr, sizeof(deviceIdStr));
                doc["device_ID"] = deviceIdStr;
                doc["pin"] = pkt->pinMv;
                doc["isSleeping"] = pkt->isSleeping ? true : false;
                
                JsonObject node = doc["node"].to<JsonObject>();
                node["type"] = "machine_status";
                
                JsonObject content = node["content"].to<JsonObject>();
                content["mode"] = (pkt->flags & STATUS_FLAG_MODE_MANUAL) ? "MANUAL" : "AUTO";
                content["chamberStatus"] = (pkt->flags & STATUS_FLAG_MEASURING) ? 1 : 0;
                content["doorStatus"] = (pkt->flags & STATUS_FLAG_DOOR_OPEN) ? 1 : 0;
                content["fanStatus"] = (pkt->flags & STATUS_FLAG_FAN_ON) ? 1 : 0;
                content["saved_manual_cycle"] = pkt->manualCycle;
                content["saved_daily_meansure"] = pkt->dailyMeasures;
                content["timestamp"] = pkt->timestamp;
                return true;
            }
            
            case PKT_UPLINK_TIME_REQ: {
                if (len < (int)sizeof(UplinkTimeReqPacket)) return false;
                UplinkTimeReqPacket* pkt = (UplinkTimeReqPacket*)buffer;
                
                formatDeviceId(pkt->deviceId, deviceIdStr, sizeof(deviceIdStr));
                doc["device_ID"] = deviceIdStr;
                doc["pin"] = pkt->pinMv;
                doc["isSleeping"] = pkt->isSleeping ? true : false;
                
                JsonObject node = doc["node"].to<JsonObject>();
                node["type"] = "time_req";
                node["content"] = (char*)nullptr;  // null content
                return true;
            }
            
            default:
                return false;  // Unknown type, fallback to legacy decoder
        }
    }
    
    // ── BRIDGE: Decode Downlink TIME packet ──
    static bool decodeDownlinkTime(uint8_t* buffer, int len, uint32_t& epoch) {
        if (len < (int)sizeof(DownlinkTimePacket)) return false;
        if (buffer[0] != PKT_DOWNLINK_TIME) return false;
        
        DownlinkTimePacket* pkt = (DownlinkTimePacket*)buffer;
        epoch = pkt->epoch;
        return true;
    }
    
    // ── BRIDGE: Decode Downlink CMD packet → JSON cho Node ──
    // Trả về: targetId để Bridge kiểm tra NID, và tạo JSON gửi xuống Node
    static bool decodeDownlinkCmd(uint8_t* buffer, int len, uint8_t& targetId, uint8_t& en,
                                   char* jsonBuf, int jsonBufSize) {
        if (len < (int)sizeof(DownlinkCmdPacket)) return false;
        if (buffer[0] != PKT_DOWNLINK_CMD) return false;
        
        DownlinkCmdPacket* pkt = (DownlinkCmdPacket*)buffer;
        targetId = pkt->targetId;
        en = pkt->en;
        
        // Nếu en = 0, gửi lệnh SLEEP
        if (en == 0) {
            snprintf(jsonBuf, jsonBufSize, "{\"set\":\"SLEEP\"}");
            return true;
        }
        
        // Build JSON based on setMode
        JsonDocument doc;
        
        // ═══ TIMESTAMP MODE ═══
        if (pkt->setMode == CMD_MODE_TIMESTAMP) {
            // Timestamp được encode trong các field khác (không có trong CMD packet)
            // Trường hợp này nên dùng PKT_DOWNLINK_TIME thay vì CMD
            doc["set"] = "TIMESTAMP";
            doc["cmd"] = 0;  // Placeholder - timestamp should use TIME packet
        }
        // ═══ SLEEP MODE ═══
        else if (pkt->setMode == CMD_MODE_SLEEP) {
            snprintf(jsonBuf, jsonBufSize, "{\"set\":\"SLEEP\"}");
            return true;
        }
        // ═══ AUTO MODE ═══
        else if (pkt->setMode == CMD_MODE_AUTO) {
            doc["set"] = "AUTO";
            
            JsonObject cmd = doc["cmd"].to<JsonObject>();
            cmd["transmissionIntervalMinutes"] = (char*)nullptr;  // null cho AUTO
            
            JsonObject doObj = cmd["do"].to<JsonObject>();
            
            // measurementCount
            if (pkt->measureCount == 0) {
                doObj["measurementCount"] = (char*)nullptr;
            } else {
                doObj["measurementCount"] = pkt->measureCount;
            }
            
            // startTime (convert minutes → "HH:MM")
            if (pkt->startTimeMin == 0xFFFF) {
                doObj["startTime"] = (char*)nullptr;
            } else {
                char timeBuf[8];
                int hour = pkt->startTimeMin / 60;
                int minute = pkt->startTimeMin % 60;
                snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d", hour, minute);
                doObj["startTime"] = timeBuf;
            }
        }
        // ═══ MANUAL MODE ═══
        else if (pkt->setMode == CMD_MODE_MANUAL) {
            doc["set"] = "MANUAL";
            
            JsonObject cmd = doc["cmd"].to<JsonObject>();
            
            // transmissionIntervalMinutes
            if (pkt->intervalMin == 0) {
                cmd["transmissionIntervalMinutes"] = (char*)nullptr;
            } else {
                cmd["transmissionIntervalMinutes"] = pkt->intervalMin;
            }
            
            JsonObject doObj = cmd["do"].to<JsonObject>();
            
            // chamberStatus
            uint8_t chamber = pkt->doFlags & CMD_FLAG_CHAMBER_MASK;
            if (chamber == CMD_FLAG_CHAMBER_START) {
                doObj["chamberStatus"] = "start-measurement";
            } else if (chamber == CMD_FLAG_CHAMBER_STOP) {
                doObj["chamberStatus"] = "stop-measurement";
            } else {
                doObj["chamberStatus"] = (char*)nullptr;
            }
            
            // doorStatus
            uint8_t door = pkt->doFlags & CMD_FLAG_DOOR_MASK;
            if (door == CMD_FLAG_DOOR_OPEN) {
                doObj["doorStatus"] = "open";
            } else if (door == CMD_FLAG_DOOR_CLOSE) {
                doObj["doorStatus"] = "close";
            } else {
                doObj["doorStatus"] = (char*)nullptr;
            }
            
            // fanStatus
            uint8_t fan = pkt->doFlags & CMD_FLAG_FAN_MASK;
            if (fan == CMD_FLAG_FAN_ON) {
                doObj["fanStatus"] = "ON";
            } else if (fan == CMD_FLAG_FAN_OFF) {
                doObj["fanStatus"] = "OFF";
            } else {
                doObj["fanStatus"] = (char*)nullptr;
            }
        }
        
        serializeJson(doc, jsonBuf, jsonBufSize);
        return true;
    }
    
    // ── Check if packet is Fixed-Schema (uplink) ──
    static bool isFixedUplink(uint8_t firstByte) {
        return (firstByte == PKT_UPLINK_DATA || 
                firstByte == PKT_UPLINK_STATUS || 
                firstByte == PKT_UPLINK_TIME_REQ);
    }
    
    // ── Check if packet is Fixed-Schema (downlink) ──
    static bool isFixedDownlink(uint8_t firstByte) {
        return (firstByte == PKT_DOWNLINK_TIME || firstByte == PKT_DOWNLINK_CMD);
    }
};