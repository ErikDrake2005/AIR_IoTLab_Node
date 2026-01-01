#pragma once
#include <Arduino.h>

// ========== THIẾT BỊ ==========
#define DEVICE_ID       "AIR_VL_01"
#define DEFAULT_MEASURES_PER_DAY 4
#define SECONDS_IN_DAY  86400UL

// ========== NTP (Dự phòng) ==========
#define NTP_SERVER "pool.ntp.org"
#define NTP_TIMEZONE  "ICT-7" 

// ========== UART GIAO TIẾP BRIDGE (Serial1) ==========
// Theo yêu cầu của bạn: 921600
// LƯU Ý: Thiết bị Bridge đấu vào chân 16/17 CŨNG PHẢI cài 921600 mới chạy được.
#define UART_BAUD       921600   
#define UART_RX_PIN     16
#define UART_TX_PIN     17

// ========== RS485 CẢM BIẾN (Serial2) ==========
#define RS485_BAUD      9600   // Tốc độ chuẩn của cảm biến
#define RS485_TX_PIN    26
#define RS485_RX_PIN    25
#define PIN_RS485_DE    4

// ========== RELAY + FAN ==========
#define CTL_ON_RELAY    13
#define CTL_OFF_RELAY   12
#define FANA            27
#define FAN1            14

// ========== SHT31 ==========
#define SHT31_I2C_ADDR  0x44
// [QUAN TRỌNG] Phải định nghĩa chân để SHT31Sensor.cpp dùng
#define SHT31_SDA_PIN   21
#define SHT31_SCL_PIN   22

// ========== OTA ==========
#define OTA_CHUNK_SIZE  1024