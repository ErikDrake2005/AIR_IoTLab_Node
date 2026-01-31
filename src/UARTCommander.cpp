#include "UARTCommander.h"
#include "CRC32.h"
#include "PacketDef.h"

UARTCommander* UARTCommander::instance = nullptr;

UARTCommander::UARTCommander() : _serial(nullptr) {
    instance = this;
    _txQueue = xQueueCreate(20, sizeof(UartTxMessage)); 
    _rxQueue = xQueueCreate(20, sizeof(UartRxMessage)); 
}

void UARTCommander::begin(HardwareSerial& serial) {
    _serial = &serial;
    xTaskCreatePinnedToCore(txTask, "UART_TX", 4096, this, 1, NULL, 1);
    xTaskCreatePinnedToCore(rxTask, "UART_RX", 4096, this, 1, NULL, 1);
    Serial.println("[UART] Commander Init (Queue & Tasks Ready)");
}

//TX TASK
void UARTCommander::txTask(void* pvParameters) {
    UARTCommander* self = (UARTCommander*)pvParameters;
    UartTxMessage msg;

    for (;;) {
        if (xQueueReceive(self->_txQueue, &msg, portMAX_DELAY) == pdTRUE) {
            if (self->_serial) {
                self->_serial->write(msg.buffer, msg.length);
                self->_serial->write('\n');
                vTaskDelay(10 / portTICK_PERIOD_MS); 
            }
        }
    }
}

void UARTCommander::enqueueTx(const uint8_t* data, size_t len, const char* debugTag) {
    if (len > 511) { // Buffer size check
        Serial.println("[UART ERR] Packet too large!");
        return;
    }
    UartTxMessage msg;
    memcpy(msg.buffer, data, len);
    msg.length = len;
    
    if(xQueueSend(_txQueue, &msg, 0) != pdTRUE) {
        Serial.println("[UART ERR] TX Queue Full!");
    }
}

void UARTCommander::send(const String& jsonStr) {
    unsigned long crc = CRC32::calculate(jsonStr);
    String packet = jsonStr + "|" + String(crc, HEX);
    Serial.print("[TX] ");
    Serial.println(packet);
    
    enqueueTx((const uint8_t*)packet.c_str(), packet.length(), "JSON");
}

//RX TASK

void UARTCommander::rxTask(void* pvParameters) {
    UARTCommander* self = (UARTCommander*)pvParameters;
    String inputBuffer = "";
    inputBuffer.reserve(512);

    for (;;) {
        if (self->_serial && self->_serial->available()) {
            while (self->_serial->available()) {
                char c = (char)self->_serial->read();
                
                if (c == '\n') {
                    if (inputBuffer.length() > 0) {
                        Serial.print("[RX] ");
                        Serial.println(inputBuffer);
                        
                        UartRxMessage rxMsg;
                        size_t copyLen = (inputBuffer.length() < sizeof(rxMsg.cmd)) ? inputBuffer.length() : (sizeof(rxMsg.cmd) - 1);
                        strncpy(rxMsg.cmd, inputBuffer.c_str(), copyLen);
                        rxMsg.cmd[copyLen] = '\0'; 
                        
                        if(xQueueSend(self->_rxQueue, &rxMsg, 0) != pdTRUE) {
                             Serial.println("[UART ERR] RX Queue Full!");
                        }
                        inputBuffer = "";
                    }
                } 
                else if (c != '\r') {
                    if (inputBuffer.length() < 1024) inputBuffer += c; 
                }
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS); 
    }
}

bool UARTCommander::hasCommand() {
    return uxQueueMessagesWaiting(_rxQueue) > 0;
}

String UARTCommander::getCommand() {
    UartRxMessage msg;
    if (xQueueReceive(_rxQueue, &msg, 0) == pdTRUE) {
        return String(msg.cmd);
    }
    return "";
}