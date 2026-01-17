#include "RelayController.h"

RelayController::RelayController() {}

void RelayController::begin() {
    pinMode(CTL_ON_RELAY, OUTPUT);
    pinMode(CTL_OFF_RELAY, OUTPUT);
    pinMode(FANA, OUTPUT);
    pinMode(FAN1, OUTPUT);
    
    // Mặc định khi khởi động: Vào chế độ nghỉ (Mở cửa, Tắt quạt)
    OFF(); 
}

// --- ĐIỀU KHIỂN CỬA (MOTOR/ACTUATOR) ---

// Hàm thực thi MỞ CỬA (Dùng cho chế độ OFF/Nghỉ)
void RelayController::ON_DOOR() { 
    // 1. Ngắt chiều Đóng trước (Safety)
    digitalWrite(CTL_OFF_RELAY, LOW); 
    
    // 2. Chờ cơ khí/từ trường xả hết (Dead-time)
    vTaskDelay(100 / portTICK_PERIOD_MS); 
    
    // 3. Kích hoạt chiều Mở
    digitalWrite(CTL_ON_RELAY, HIGH);
    Serial.println("[RELAY] Door OPENING...");
}

// Hàm thực thi ĐÓNG CỬA (Dùng cho chế độ ON/Đo)
void RelayController::OFF_DOOR() { 
    // 1. Ngắt chiều Mở trước
    digitalWrite(CTL_ON_RELAY, LOW);
    
    // 2. Dead-time
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // 3. Kích hoạt chiều Đóng
    digitalWrite(CTL_OFF_RELAY, HIGH); 
    Serial.println("[RELAY] Door CLOSING...");
}

// --- ĐIỀU KHIỂN QUẠT ---

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

// --- HÀM TỔNG HỢP (THEO YÊU CẦU CỦA BẠN) ---

// ON = CHẾ ĐỘ ĐO (Kín khí) -> Đóng cửa + Bật quạt
void RelayController::ON() { 
    Serial.println("[MACHINE] MODE ON (Measuring State)");
    OFF_DOOR(); // Đóng cửa
    vTaskDelay(500 / portTICK_PERIOD_MS); // Đợi cửa bắt đầu chạy ổn định
    ON_FAN();   // Bật quạt hút khí vào/trộn khí
}

// OFF = CHẾ ĐỘ NGHỈ (Thoáng khí) -> Mở cửa + Tắt quạt
void RelayController::OFF() { 
    Serial.println("[MACHINE] MODE OFF (Idle State)");
    OFF_FAN();  // Tắt quạt trước cho đỡ bụi
    vTaskDelay(200 / portTICK_PERIOD_MS);
    ON_DOOR();  // Mở cửa ra
}