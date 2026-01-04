#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <Arduino.h>
#include <vector>
#include "Measurement.h"
#include "RelayController.h"
#include "UARTCommander.h"
#include "TimeSync.h"
#include "JsonFormatter.h" // [BẮT BUỘC] Vì khai báo object _json bên trong

// 1. Định nghĩa các trạng thái của chu trình
enum CycleState {
    STATE_IDLE,
    STATE_PREPARING,
    STATE_STABILIZING,
    STATE_WAIT_MEASURE_1,
    STATE_WAIT_MEASURE_2,
    STATE_WAIT_MEASURE_3,
    STATE_FINISHING,
    STATE_MANUAL_LOOP // [MỚI] Trạng thái vòng lặp thủ công
};

// 2. Định nghĩa chế độ hoạt động
enum MachineMode {
    MANUAL,
    AUTO
};

// 3. Struct lưu lịch hẹn giờ cố định
struct ScheduleTime {
    int hour;
    int minute;
    int second;
};

// 4. Class chính
class StateMachine {
public:
    // Constructor (Lưu ý: Không truyền JsonFormatter vào, class tự khởi tạo bên trong)
    StateMachine(Measurement& meas, RelayController& relay, UARTCommander& cmd, TimeSync& timeSync);
    
    void begin();
    void update();
    void handleCommand(const String& jsonCmd);

    // [FIX] Hàm này cần thiết vì main.cpp gọi khi nhận lệnh stop bên ngoài
    void setStopFlag(bool flag);
    void processJsonCommand(const String& jsonStr);

private:
    // Các module con (Tham chiếu)
    Measurement& _meas;
    RelayController& _relay;
    UARTCommander& _cmd;
    TimeSync& _timeSync;
    
    // [QUAN TRỌNG] Object xử lý JSON nội bộ
    JsonFormatter _json; 

    // Biến trạng thái chung
    MachineMode _mode;
    CycleState _cycleState;
    unsigned long _cycleStartMillis;
    void _handleDirectCommand(const char* cmd);
    // [FIX] Các biến kiểm soát luồng (Bị thiếu trong phiên bản trước)
    bool _stopRequested;          // Cờ báo dừng khẩn cấp
    unsigned long _lastGridBlock; // Đánh dấu block thời gian lưới (Grid)
    int _lastTriggerMinute;       // Đánh dấu phút đã trigger (Scheduled)

    // Biến cấu hình Auto Mode
    int _measuresPerDay;
    unsigned long _gridIntervalSeconds;
    std::vector<ScheduleTime> _schedules;
    MeasurementData _miniData[3];     // Lưu dữ liệu 3 lần đo
    int _measureCount;                // Đếm số lần đo

    // --- BIẾN CHO TÍNH NĂNG MANUAL LOOP (Vòng lặp thủ công) ---
    unsigned long _currentManualInterval;   // Chu kỳ đo (ms)
    unsigned long _nextManualMeasureMillis; // Thời điểm đo kế tiếp
    
    // --- CÁC HÀM LOGIC NỘI BỘ (PRIVATE) ---
    void _startManualLoop();                // Khởi động loop
    
    // Tính toán thời gian
    void _calculateGridInterval();
    void _parseSetTime(String timeStr);
    bool _isTimeForGridMeasure();
    bool _isTimeForScheduledMeasure();

    // Quy trình đo 15 phút (Mini-Cycle)
    void _startCycle();       
    void _processCycleLogic();
    void _finishCycle();
    void _stopAndResetCycle(); // Dùng chung cho cả Stop và Reset
    
    // Gửi dữ liệu
    void _sendResponse(String json);
};

#endif