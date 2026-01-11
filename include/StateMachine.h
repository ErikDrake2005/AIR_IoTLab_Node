#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>
#include <vector>
#include <ArduinoJson.h>
#include "Measurement.h"
#include "RelayController.h"
#include "UARTCommander.h"
#include "TimeSync.h"
#include "JsonFormatter.h"
#include "Config.h"

enum CycleState { STATE_IDLE, STATE_PREPARING, STATE_STABILIZING, STATE_WAIT_MEASURE_1, STATE_WAIT_MEASURE_2, STATE_WAIT_MEASURE_3, STATE_FINISHING, STATE_MANUAL_LOOP };
enum MachineMode { MANUAL, AUTO };
struct ScheduleTime { int hour; int minute; int second; };

class StateMachine {
public:
    StateMachine(Measurement& meas, RelayController& relay, UARTCommander& cmd, TimeSync& timeSync);
    void begin();
    void update();
    void handleCommand(const String& jsonCmd);
    void setStopFlag(bool flag);
    void processJsonCommand(const String& jsonStr);

private:
    Measurement& _meas;
    RelayController& _relay;
    UARTCommander& _cmd;
    TimeSync& _timeSync;
    JsonFormatter _json; 

    MachineMode _mode;
    CycleState _cycleState;
    
    unsigned long _cycleStartMillis;
    bool _stopRequested;
    unsigned long _lastGridBlock;
    int _lastTriggerMinute;

    int _measuresPerDay;
    unsigned long _gridIntervalSeconds;
    std::vector<ScheduleTime> _schedules;
    MeasurementData _miniData[3];
    int _measureCount;

    unsigned long _currentManualInterval;
    unsigned long _nextManualMeasureMillis;
    unsigned long _uartWakeupMillis;
    bool _isUartWakeupActive;

    void _handleDirectCommand(const char* cmd);
    void _startManualLoop();
    void _calculateGridInterval();
    void _parseSetTime(String timeStr);
    bool _isTimeForGridMeasure();
    bool _isTimeForScheduledMeasure();
    void _startCycle();       
    void _processCycleLogic();
    void _finishCycle();
    void _stopAndResetCycle();
    void _handleDeepSleepSequence();
    void _tryLightSleep();
    unsigned long _calculateNextWakeTime();
    void _setBusyPin(bool isBusy);
    void _sendResponse(String json);
};
#endif