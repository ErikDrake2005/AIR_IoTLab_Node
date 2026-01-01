
#include "TimeSync.h"
#include "JsonFormatter.h"

TimeSync::TimeSync(JsonFormatter& json) : _json(json) {
    _syncMillis = millis();
    _epochBase = 0;
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
}

unsigned long TimeSync::getCurrentTime() {
    if (_epochBase == 0) return 0;
    return _epochBase + ((millis() - _syncMillis) / 1000UL);
}

void TimeSync::requestTimeSync() {
    // Hàm này sẽ được gọi từ main hoặc StateMachine
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
