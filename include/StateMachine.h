#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>
#include <vector>
#include "Measurement.h"      
#include "RelayController.h"
#include "UARTCommander.h"
#include "TimeSync.h"
#include "CommandProcessor.h"
#include "JsonFormatter.h"    
#include "driver/uart.h"
#include "Config.h"           
#include "CommonTypes.h" // <--- BẮT BUỘC PHẢI CÓ FILE NÀY

struct ScheduleTime {
    int hour;
    int minute;
};

class StateMachine {
public:
    StateMachine(Measurement& meas, RelayController& relay, UARTCommander& cmd, TimeSync& timeSync);
    void begin();
    void update();
    void processRawCommand(String rawStr);
    void setStopFlag(bool flag) { _stopRequested = flag; }

private:
    // --- MODULES ---
    Measurement& _meas;
    RelayController& _relay;
    UARTCommander& _cmd;
    TimeSync& _timeSync;
    CommandProcessor _cmdProcessor;
    JsonFormatter _jsonFormatter; 

    // --- QUẢN LÝ TRẠNG THÁI (MỚI) ---
    MachineStatus _status; // <--- SỬA LỖI: Biến _status chứa toàn bộ mode, fan, door...
    CycleState _cycleState;
    
    bool _stopRequested;
    
    // --- THỜI GIAN & LỊCH TRÌNH ---
    unsigned long _gridInterval;
    unsigned long _startTimeSeconds;   // startTime parsed (giây trong ngày)
    unsigned long _manualIntervalMs; // Vẫn giữ để tính toán nhanh
    unsigned long _nextManualRun;
    
    unsigned long _cycleStartMs;
    unsigned long _nextTimeSync;
    
    // --- SLEEP WAKEUP ---
    bool _isUartWakeup;
    unsigned long _uartWakeupMs;
    
    std::vector<ScheduleTime> _schedules;
    int _lastTriggerMin;
    
    // --- COOLDOWN (Chống đụng lịch) ---
    unsigned long _lastCycleStartSeconds;  // Thời điểm bắt đầu cycle gần nhất (epoch seconds)
    static const unsigned long CYCLE_COOLDOWN = 900;  // 15 phút cooldown
    
    // --- BUFFER DỮ LIỆU ĐO (Cho Auto Cycle) ---
    MeasurementData _miniData[3]; 

    // --- CÁC HÀM NỘI BỘ (INTERNAL METHODS) ---
    void _applyCommand(CommandData& cmd);
    
    // Logic Auto (3 Mini-cycles)
    void _startAutoCycle();     
    void _processAutoCycle();  
    void _finishAutoCycle(bool aborted); 
    
    // Logic Manual (Single Shot)
    void _performManualMeasurement();

    // Hàm chung
    void _resetCycle();
    void _resetMiniData(); 
    
    // Sleep logic
    void _handleLightSleep();
    unsigned long _calcSleepTime();
    void _enterDeepSleep();
    
    // Helpers
    void _recalcGrid();
    bool _checkSchedule();
    bool _checkGrid();
    
    void _sendMachineStatus();
    void _requestTimeSync();
    void _sendPacket(String json);
    
    void _setDoor(bool open);
    void _setFan(bool on);
    
    // Check lệnh khẩn cấp (stop) trong quá trình đo
    void _checkUrgentCommands();
};

#endif