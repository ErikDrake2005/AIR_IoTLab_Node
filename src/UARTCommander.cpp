#include "UARTCommander.h"
#include "CRC32.h" // [BẮT BUỘC] Thêm thư viện CRC32

UARTCommander* UARTCommander::instance = nullptr;

UARTCommander::UARTCommander() : _serial(nullptr) {
    instance = this;
    _commandQueue = xQueueCreate(20, 512); 
}

void UARTCommander::begin(HardwareSerial& serial) {
    _serial = &serial; 
    Serial.println("[UART] Commander Init OK");
}

void UARTCommander::send(const String& jsonStr) {
    if (!_serial) return;

    // [FIX] Tính CRC32 để khớp với Bridge
    unsigned long crc = CRC32::calculate(jsonStr);

    // Gửi theo định dạng: {"data":...}|CRC_HEX
    _serial->print(jsonStr);
    _serial->print("|");
    _serial->println(String(crc, HEX));

    Serial.print("[UART_TX] "); 
    Serial.println(jsonStr);
}

void UARTCommander::pushToQueue(String data) {
    char buffer[512]; 
    memset(buffer, 0, 512);
    // Copy an toàn
    if (data.length() > 511) data = data.substring(0, 511);
    strcpy(buffer, data.c_str());
    
    if (xQueueSend(_commandQueue, &buffer, 0) != pdTRUE) {
        Serial.println("[UART] Queue Full!");
    }
}

bool UARTCommander::hasCommand() { 
    return uxQueueMessagesWaiting(_commandQueue) > 0; 
}

String UARTCommander::getCommand() {
    char buffer[512];
    if (xQueueReceive(_commandQueue, &buffer, 0) == pdTRUE) {
        return String(buffer);
    }
    return "";
}

void UARTCommander::clearCommand() { 
    xQueueReset(_commandQueue); 
}