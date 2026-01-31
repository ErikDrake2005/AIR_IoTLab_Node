#pragma once
#include <Arduino.h>

#define MAX_JSON_SIZE 1024  // Kích thước tối đa cho gói tin JSON

enum SystemMode {
    MODE_MANUAL = 0,
    MODE_AUTO = 1,
    MODE_TIMESTAMP = 2,
    MODE_SLEEP = 3
};

enum CycleState {
    STATE_IDLE,         // Đang nghỉ/chờ
    STATE_PREPARE,      // Chuẩn bị (đóng cửa, bật quạt)
    STATE_WAIT_1,       // Chờ đo lần 1 (phút 3)
    STATE_WAIT_2,       // Chờ đo lần 2 (phút 8)
    STATE_WAIT_3,       // Chờ đo lần 3 (phút 15)
    STATE_MANUAL_WAIT,  // Chờ trong chế độ Manual (nếu cần)
    STATE_FINISH        // Đã đo xong, xử lý kết quả
};
//State Machine Status Structure
struct MachineStatus {
    SystemMode mode;            // AUTO / MANUAL
    bool isMeasuring;           // Đang đo hay không
    bool isDoorOpen;            // Trạng thái cửa
    bool isFanOn;               // Trạng thái quạt
    int saved_manual_cycle;     // Chu kỳ lấy mẫu Manual (phút)
    int saved_daily_measures;   // Số lần đo Auto/ngày
    
    // Constructor mặc định
    MachineStatus() {
        mode = MODE_MANUAL;
        isMeasuring = false;
        isDoorOpen = true;  
        isFanOn = false;
        saved_manual_cycle = 1;
        saved_daily_measures = 4;
    }
};

// Server Command Data Structure
struct CommandData {
    bool isValid;           // Đánh dấu gói tin có hợp lệ không
    String targetNID;       // NID của gói tin
    bool enable;            // en: 1 (thức) hoặc 0 (ngủ)
    bool reset;
    // Phân loại lệnh
    SystemMode setMode;     // AUTO/MANUAL/TIMESTAMP/SLEEP
    
    // Dữ liệu thời gian (cho TIMESTAMP)
    unsigned long timestamp;

    // Dữ liệu cấu hình (trong "cmd")
    int manualInterval;     // transmissionIntervalMinutes
    int autoMeasureCount;   // measurementCount
    String autoStartTime;   // startTime (String "20:20")
    int startTimeSeconds;   // startTime parsed (giây trong ngày: 73200 cho 20:20)
    
    // Dữ liệu hành động tức thời (trong "do")
    bool hasActions;        // Có lệnh hành động không?
    String chamberStatus;   // "stop-measurement" ...
    String doorStatus;      // "open", "close"
    String fanStatus;       // "on", "off"

    // Constructor reset
    CommandData() {
        isValid = false;
        enable = false;
        hasActions = false;
        setMode = MODE_MANUAL;
        timestamp = 0;
        manualInterval = -1; 
        autoMeasureCount = -1;
        startTimeSeconds = 0;
    }
};