#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <vector>

#include "Measurement.h"
#include "RelayController.h"
#include "UARTCommander.h"
#include "TimeSync.h"
#include "JsonFormatter.h"
#include "CommandProcessor.h"
#include "Config.h" // Đảm bảo có file này chứa PIN definitions

// Định nghĩa các trạng thái máy đo
enum MachineState {
    STATE_IDLE,
    STATE_PREPARING,
    STATE_WAIT_MEASURE_1,
    STATE_WAIT_MEASURE_2,
    STATE_WAIT_MEASURE_3,
    STATE_FINISHING,
    STATE_MANUAL_LOOP
};

enum OperationMode {
    MANUAL,
    AUTO
};

struct Schedule {
    int hour;
    int minute;
    int second;
};

// Machine Status Struct - Sent whenever state changes
struct MachineStatus {
    String type;           // "machine_status"
    String deviceId;       // Device identifier
    String door;           // "open" or "close"
    String fan;            // "on" or "off"
    String mode;           // "auto" or "manual"
    String measuring;      // "YES" or "NO"
    String cycleManual;    // Current manual cycle in minutes
    String measuresPerDay; // Current measures per day
    unsigned long timestamp; // Current epoch time
};

class StateMachine {
public:
    StateMachine(Measurement& meas, RelayController& relay, UARTCommander& cmd, TimeSync& timeSync);
    void begin();
    void update();
    
    // --- Public Methods ---
    void handleCommand(String jsonStr);
    void setStopFlag(bool flag);

private:
    // Modules
    Measurement& _meas;
    RelayController& _relay;
    UARTCommander& _cmd;
    TimeSync& _timeSync;

    // State Variables
    OperationMode _mode;
    MachineState _cycleState;
    bool _stopRequested;
    
    // Relay Status
    bool _isDoorOpen;
    bool _isFanOn;
    MachineStatus _lastStatus;  // Track for change detection
    bool _statusChanged;        // Flag to send status update
    
    // Time & Config Variables
    int _measuresPerDay;
    unsigned long _gridIntervalSeconds;
    std::vector<Schedule> _schedules;
    int _lastTriggerMinute;
    
    // Cycle Variables
    unsigned long _cycleStartMillis;
    unsigned long _currentManualInterval;
    unsigned long _nextManualMeasureMillis;
    int _measureCount;
    
    // Data buffer cho 3 giai đoạn đo
    MeasurementData _miniData[3];

    // Wakeup Variables
    bool _isUartWakeupActive;
    unsigned long _uartWakeupMillis;
    
    // Auto Time Sync Variables
    unsigned long _lastTimeSyncRequest;  // Timestamp of last time_req sent
    unsigned long _nextTimeSyncTime;     // When to request next sync
    bool _waitingForTimeSync;            // Waiting for Gateway response

    // --- Private Methods (Internal Logic) ---
    void _ctrlDoor(bool open);
    void _ctrlFan(bool on);
    
    // Command Processing with Priority
    void _processCommandByPriority(const CommandData& cmd);
    void _handleEnCommand(int enValue);
    void _handleTimeSync(unsigned long epochTime);
    void _handleModeCommand(const String& mode);
    void _handleTimeFunctions(const CommandData& cmd);
    void _handleMeasurementCommand(const String& cmd);
    void _handleRelayControl(const CommandData& cmd);

    void _processCycleLogic();
    void _startCycle();
    void _finishCycle();
    void _stopAndResetCycle();
    
    // Sleep & Power
    void _tryLightSleep();
    void _handleDeepSleepSequence();
    unsigned long _calculateNextWakeTime();
    void _setBusyPin(bool isBusy);
    
    // Time Sync Auto-Request
    void _checkAndRequestTimeSync();  // Called periodically from update()
    void _sendTimeSyncRequest();      // Send time_req to Gateway
    
    // Utils
    void _sendMachineStatus();        // Send status whenever changed
    void _updateMachineStatus();      // Internal: update status struct
    void _sendResponse(String json);
    
    void _calculateGridInterval();
    void _parseSetTime(String tStr);
    bool _isTimeForGridMeasure();
    bool _isTimeForScheduledMeasure();
};

#endif