#pragma once
#include <Arduino.h>
#include <vector>
#include "Measurement.h"
#include "RelayController.h"
#include "UARTCommander.h"
#include "TimeSync.h"
#include "JsonFormatter.h"

// Enum trạng thái chu trình
enum CycleState {
    STATE_IDLE,
    STATE_PREPARING,
    STATE_STABILIZING,
    STATE_WAIT_MEASURE_1,
    STATE_WAIT_MEASURE_2,
    STATE_WAIT_MEASURE_3,
    STATE_FINISHING,
    STATE_MANUAL_LOOP
};

// Enum chế độ
enum MachineMode { MANUAL, AUTO };

// Struct lịch hẹn (Chỉ giữ Hour/Minute)
struct ScheduleTime {
    int hour;
    int minute;
};

class StateMachine {
public:
    StateMachine(Measurement& meas, RelayController& relay, UARTCommander& cmd, TimeSync& timeSync);
    
    void begin();
    void update();
    void handleCommand(const String& jsonCmd);
    void setStopFlag(bool flag);

    // Hàm tính toán Light Sleep
    uint32_t getSleepIntervalMs(); 

private:
    Measurement& _meas;
    RelayController& _relay;
    UARTCommander& _cmd;
    TimeSync& _timeSync;
    JsonFormatter _json; 

    MachineMode _mode;
    CycleState _cycleState;
    
    bool _stopRequested;
    unsigned long _cycleStartMillis;
    
    // Config Auto
    int _measuresPerDay;
    unsigned long _gridIntervalSeconds;
    std::vector<ScheduleTime> _schedules;
    
    // [ĐÃ KHÔI PHỤC] Biến theo dõi phút để tránh trigger lặp lại
    int _lastTriggerMinute;

    // Data Storage
    MeasurementData _miniData[3];     
    int _measureCount;                

    // Manual Loop Config
    unsigned long _currentManualInterval;   
    unsigned long _nextManualMeasureMillis; 

    // --- CÁC HÀM PRIVATE (ĐÃ KHÔI PHỤC ĐẦY ĐỦ) ---
    void _processAutoLogic();
    void _processManualLogic();
    void _processCycleLogic();
    
    void _startCycle();       
    void _finishCycle();
    void _stopAndResetCycle();
    
    // [ĐÃ KHÔI PHỤC] Hàm khởi động vòng lặp Manual
    void _startManualLoop(); 

    void _calculateGridInterval();
    long _getSecondsToNextGrid();
    
    // [ĐÃ KHÔI PHỤC] Hàm kiểm tra thời gian
    bool _isTimeForGridMeasure();     
    bool _isTimeForScheduledMeasure();
    void _parseSetTime(String timeStr);
    
    void _sendResponse(String json);
};