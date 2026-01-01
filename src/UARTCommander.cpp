#include "UARTCommander.h"

UARTCommander* UARTCommander::instance = nullptr;

UARTCommander::UARTCommander() : _serial(nullptr) {
    instance = this;
    _commandQueue = xQueueCreate(20, 512); 
}

void UARTCommander::begin(HardwareSerial& serial) {
    _serial = &serial; 
    Serial.println("[UARTCommander] Initialized.");
}

void UARTCommander::send(const String& data) {
    if (_serial) _serial->println(data);
    Serial.print("\n[UART_TX_LOG] Sent: "); 
        Serial.println(data);
}
void UARTCommander::pushToQueue(String data) {
    char buffer[512]; 
    memset(buffer, 0, 512);
    strncpy(buffer, data.c_str(), 511);
    if (xQueueSend(_commandQueue, &buffer, 0) != pdTRUE) {
        Serial.println("[UARTCommander] Queue Full!");
    }
}

bool UARTCommander::hasCommand() { return uxQueueMessagesWaiting(_commandQueue) > 0; }

String UARTCommander::getCommand() {
    char buffer[512];
    if (xQueueReceive(_commandQueue, &buffer, 0) == pdTRUE) {
        return String(buffer);
    }
    return "";
}

void UARTCommander::clearCommand() { xQueueReset(_commandQueue); }
void UARTCommander::uartISR() { }