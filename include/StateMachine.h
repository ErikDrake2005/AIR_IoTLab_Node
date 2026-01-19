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
#include "CommonTypes.h"

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
    Measurement& _meas;
    RelayController& _relay;
    UARTCommander& _cmd;
    TimeSync& _timeSync;
    CommandProcessor _cmdProcessor;
    JsonFormatter _jsonFormatter; 
    MachineStatus _status;
    CycleState _cycleState;
    
    bool _stopRequested;
    unsigned long _gridInterval;
    unsigned long _startTimeSeconds; 
    unsigned long _manualIntervalMs; 
    unsigned long _nextManualRun;
    
    unsigned long _cycleStartMs;
    unsigned long _nextTimeSync;
    
    bool _isUartWakeup;
    unsigned long _uartWakeupMs;
    
    std::vector<ScheduleTime> _schedules;
    int _lastTriggerMin;
    
    unsigned long _lastCycleStartSeconds;
    static const unsigned long CYCLE_COOLDOWN = 900;
    
    MeasurementData _miniData[3]; 

    void _applyCommand(CommandData& cmd);
    void _startAutoCycle();     
    void _processAutoCycle();  
    void _finishAutoCycle(bool aborted); 
    void _performManualMeasurement();
    void _resetCycle();
    void _resetMiniData(); 
    void _handleLightSleep();
    unsigned long _calcSleepTime();
    void _enterDeepSleep();
    void _recalcGrid();
    bool _checkSchedule();
    bool _checkGrid();
    void _sendMachineStatus();
    void _requestTimeSync();
    void _sendPacket(String json);
    void _sendDataPacket(const MeasurementData& data, int sampleNum);
    void _setDoor(bool open);
    void _setFan(bool on);
    void _checkUrgentCommands();
};

#endif
