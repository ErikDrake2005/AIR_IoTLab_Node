#include "UARTCommander.h"
#include "CRC32.h"
#include "PacketDef.h"

UARTCommander* UARTCommander::instance = nullptr;

UARTCommander::UARTCommander() : _serial(nullptr) {
    instance = this;
    _commandQueue = xQueueCreate(20, 512); 
}

void UARTCommander::begin(HardwareSerial& serial) {
    _serial = &serial; 
    Serial.println("[UART] Commander Init OK");
}

// === JSON FORMAT (Backward Compatible) ===
void UARTCommander::send(const String& jsonStr) {
    if (!_serial) return;

    unsigned long crc = CRC32::calculate(jsonStr);
    _serial->print(jsonStr);
    _serial->print("|");
    _serial->println(String(crc, HEX));

    Serial.print("[UART_TX JSON] "); 
    Serial.println(jsonStr);
}

// === BINARY FORMAT (Efficient - PacketDef) ===
void UARTCommander::sendBinary(JsonDocument& doc) {
    if (!_serial) return;

    // Encode JSON -> Binary
    uint8_t buffer[256];
    int len = PacketUtils::encodeJsonToBinary(doc, buffer, 256);
    
    if (len <= 0) {
        Serial.println("[ERR] Binary encode failed!");
        return;
    }

    // Send binary data
    _serial->write(buffer, len);
    _serial->write('\n');  // Terminator
    
    String jsonStr;
    serializeJson(doc, jsonStr);
    Serial.printf("[UART_TX BIN] %d bytes: %s\n", len, jsonStr.c_str());
}

void UARTCommander::pushToQueue(String data) {
    char buffer[512]; 
    memset(buffer, 0, 512);
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
