#include "RelayController.h"

RelayController::RelayController() {}

void RelayController::begin() {
    pinMode(CTL_ON_RELAY, OUTPUT);
    pinMode(CTL_OFF_RELAY, OUTPUT);
    pinMode(FANA, OUTPUT);
    pinMode(FAN1, OUTPUT);
    OFF(); 
}
void RelayController::ON_DOOR() { 
    digitalWrite(CTL_OFF_RELAY, LOW); 
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(CTL_ON_RELAY, HIGH);
    Serial.println("[RELAY] Door OPENING...");
}

void RelayController::OFF_DOOR() { 
    digitalWrite(CTL_ON_RELAY, LOW);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    digitalWrite(CTL_OFF_RELAY, HIGH); 
    Serial.println("[RELAY] Door CLOSING...");
}

void RelayController::ON_FAN() { 
    digitalWrite(FANA, HIGH); 
    digitalWrite(FAN1, HIGH); 
    Serial.println("[RELAY] Fan ON");
}

void RelayController::OFF_FAN() { 
    digitalWrite(FANA, LOW); 
    digitalWrite(FAN1, LOW); 
    Serial.println("[RELAY] Fan OFF");
}

void RelayController::ON() { 
    Serial.println("[MACHINE] MODE ON (Measuring State)");
    OFF_DOOR();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    ON_FAN();
}

void RelayController::OFF() { 
    Serial.println("[MACHINE] MODE OFF (Idle State)");
    OFF_FAN();
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ON_DOOR();
}