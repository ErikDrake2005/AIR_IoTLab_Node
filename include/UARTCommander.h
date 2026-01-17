#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Cấu trúc gói tin TX
struct UartTxMessage {
    uint8_t buffer[256];
    size_t length;
};

// Cấu trúc gói tin RX (Lưu chuỗi lệnh nhận được)
struct UartRxMessage {
    char cmd[512]; // Buffer cho chuỗi JSON nhận được
};

class UARTCommander {
private:
    HardwareSerial* _serial;
    QueueHandle_t _txQueue; // Hàng đợi gửi đi (có delay 10ms)
    QueueHandle_t _rxQueue; // Hàng đợi nhận về (cho main xử lý)
    
    static UARTCommander* instance;
    
    // Task gửi tin (có delay 10ms để bảo vệ Bridge)
    static void txTask(void* pvParameters);
    
    // Task nhận tin (lắng nghe liên tục)
    static void rxTask(void* pvParameters);

public:
    UARTCommander();
    void begin(HardwareSerial& serial);

    // --- PHẦN GỬI (TX) ---
    // Thay thế pushToQueue bằng send (đã bao gồm tính CRC)
    void send(const String& jsonStr);
    void sendBinary(JsonDocument& doc);
    
    // Hàm nội bộ đẩy vào queue TX
    void enqueueTx(const uint8_t* data, size_t len, const char* debugTag);

    // --- PHẦN NHẬN (RX) - Để sửa lỗi main.cpp ---
    bool hasCommand();      // Kiểm tra có lệnh mới không
    String getCommand();    // Lấy lệnh ra để xử lý
};