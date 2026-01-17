
#include "TimeSync.h"
#include "JsonFormatter.h"
#include <time.h>

TimeSync::TimeSync(JsonFormatter& json) : _json(json) {
    _syncMillis = millis();
    _epochBase = 0;
    _lastRtcTime = 0;
}

String TimeSync::getRequestJson() {
    return _json.createTimeSyncRequest();
}

void TimeSync::syncFromTimestamp(const String& timeStr) {
    unsigned long epoch = _parseTimestamp(timeStr);
    if (epoch > 0) updateEpoch(epoch);
}

void TimeSync::updateEpoch(unsigned long epoch) {
    _epochBase = epoch;
    _syncMillis = millis();
    struct timeval now = { .tv_sec = (time_t)epoch, .tv_usec = 0 };
    settimeofday(&now, NULL);
    
    // Lưu RTC time
    _lastRtcTime = epoch;
    
    Serial.printf("[TIME] Updated: %lu (epoch=%lu, millis=%lu)\n", epoch, _epochBase, _syncMillis);
}

unsigned long TimeSync::getCurrentTime() {
    if (_epochBase == 0) {
        // Nếu chưa cập nhật, lấy từ RTC của ESP32
        time_t now;
        time(&now);
        if (now > 100000) return (unsigned long)now;
        return 0;
    }
    
    // Phương pháp thường: Cộng elapsed millis
    return _epochBase + ((millis() - _syncMillis) / 1000UL);
}

// Gọi TRỚ KHI VÀO DEEP SLEEP
void TimeSync::beforeDeepSleep() {
    // Lấy thời gian hiện tại từ RTC
    time_t now;
    time(&now);
    
    // Cập nhật lại _epochBase dựa trên thời gian hiện tại
    // Để khi wake up, giờ sẽ chính xác
    if (_epochBase > 0) {
        // Tính lại elapsed time từ lần cập nhật
        unsigned long currentTime = _epochBase + ((millis() - _syncMillis) / 1000UL);
        _epochBase = currentTime;  // Cập nhật base
        _syncMillis = millis();     // Reset millis counter
        
        // Lưu vào RTC của ESP32
        struct timeval tv = { .tv_sec = (time_t)currentTime, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        _lastRtcTime = currentTime;
        
        Serial.printf("[TIME PRE-SLEEP] Saved: %lu\n", currentTime);
    }
}

// Gọi SAU KHI WAKE UP TỪ DEEP SLEEP
void TimeSync::afterWakeup() {
    // Lấy thời gian từ RTC (chạy ngầm khi sleep)
    time_t rtcNow;
    time(&rtcNow);
    
    if (_lastRtcTime > 0 && (time_t)_lastRtcTime < rtcNow) {
        unsigned long sleepDuration = (unsigned long)(rtcNow - _lastRtcTime);
        
        // Cập nhật epoch dựa trên thời gian ngủ
        _epochBase = (unsigned long)rtcNow;
        _syncMillis = millis();
        
        Serial.printf("[TIME POST-WAKE] RTC: %lu, Sleep duration: %lu sec, Restored: %lu\n", 
                      (unsigned long)rtcNow, sleepDuration, _epochBase);
    } else if (_epochBase > 0) {
        // Fallback: Tiếp tục từ _epochBase + thời gian đã trôi
        _syncMillis = millis();
    }
}

unsigned long TimeSync::_parseTimestamp(const String& timestamp) {
    int year, month, day, hour, minute, second;
    if (sscanf(timestamp.c_str(), "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second) == 6) {
        struct tm tm;
        tm.tm_year = year - 1900; tm.tm_mon = month - 1; tm.tm_mday = day;
        tm.tm_hour = hour; tm.tm_min = minute; tm.tm_sec = second; tm.tm_isdst = 0;
        return (unsigned long)mktime(&tm);
    }
    return 0;
}
