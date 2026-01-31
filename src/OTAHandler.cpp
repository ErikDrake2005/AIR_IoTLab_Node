
// #include "OTAHandler.h"

// OTAHandler::OTAHandler() {}

// bool OTAHandler::startOTA(size_t totalSize) {
//     if (!Update.begin(totalSize)) {
//         Serial.println("[OTA] Khong the bat dau OTA");
//         return false;
//     }
//     _totalSize = totalSize;
//     _received = 0;
//     _lastChunkTime = millis();
//     Serial.printf("[OTA] Bat dau OTA, kich thuoc: %zu bytes\n", totalSize);
//     return true;
// }
// bool OTAHandler::handleChunk(const uint8_t* data, size_t len) {
//     size_t written = Update.write((uint8_t*)data, len);
//     if (written != len) {
//         Serial.printf("[OTA] Loi ghi chunk: %zu/%zu\n", written, len);
//         return false;
//     }
//     _received += len;
//     _lastChunkTime = millis();
//     Serial.printf("[OTA] Da nhan: %zu/%zu bytes\n", _received, _totalSize);
//     return true;
// }

// bool OTAHandler::finishOTA() {
//     if (_received != _totalSize) {
//         Serial.println("[OTA] Kich thuoc nhan du khong khop");
//         return false;
//     }
//     if (!Update.end(true)) {
//         Serial.println("[OTA] Loi khi ket thuc OTA");
//         return false;
//     }
//     Serial.println("[OTA] OTA HOAN THANH – Reboot...");
//     ESP.restart();
//     return true;
// }

// void OTAHandler::abortOTA() {
//     Update.abort();
//     _totalSize = 0;
//     _received = 0;
//     _lastChunkTime = 0;
//     Serial.println("[OTA] OTA bi huy");
// }