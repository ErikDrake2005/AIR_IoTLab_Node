#include "RelayController.h"

RelayController::RelayController() {}

void RelayController::begin() {
    pinMode(CTL_ON_RELAY, OUTPUT);
    pinMode(CTL_OFF_RELAY, OUTPUT);
    pinMode(FANA, OUTPUT);
    pinMode(FAN1, OUTPUT);
    ON_DOOR(); 
    OFF_FAN();
}

// Logic: ON_DOOR = Kích hoạt relay mở cửa
void RelayController::ON_DOOR() { 
    digitalWrite(CTL_ON_RELAY, HIGH);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    digitalWrite(CTL_OFF_RELAY, LOW); 
}

// Logic: OFF_DOOR = Kích hoạt relay đóng cửa
void RelayController::OFF_DOOR() { 
    digitalWrite(CTL_ON_RELAY, LOW);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    digitalWrite(CTL_OFF_RELAY, HIGH); 
}

void RelayController::ON_FAN() { 
    digitalWrite(FANA, HIGH); 
    digitalWrite(FAN1, HIGH); 
}

void RelayController::OFF_FAN() { 
    digitalWrite(FANA, LOW); 
    digitalWrite(FAN1, LOW); 
}