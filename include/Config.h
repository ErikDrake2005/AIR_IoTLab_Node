#pragma once
#include <Arduino.h>

// ========== PINOUT ===========
#define PIN_SLEEP_STATUS 32  // LOW=Sleep, HIGH=Awake (Báo cho Bridge)
#define PIN_WAKEUP_GPIO  33

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

// ========== I2C  ==========
#define SHT31_I2C_ADDR  0x44
#define I2C_SDA_PIN   21
#define I2C_SCL_PIN   22

// ========== BATTERY & POWER SAVING ==========
#define BAT_VOLTAGE_DIVIDER 2.0     
#define BAT_MIN_VOLTAGE     3.7     // Ngưỡng NGẮT (Deep Sleep)
#define BAT_RECOVERY_VOLTAGE 3.9
#define BAT_CHECK_INTERVAL  60

// ========== TIME SYNC AUTO-REQUEST ==========
#define TIME_SYNC_INTERVAL_SECONDS  3600  // Request time sync every 1 hour
#define TIME_SYNC_RETRY_INTERVAL    30    // Retry after 30 seconds if no response

// ========== RESPONSE TYPES ==========
// Used in JSON responses: "type": "data" or "type": "machine_status"
#define RESP_TYPE_DATA            "data"
#define RESP_TYPE_MACHINE_STATUS  "machine_status"

// ========== OTA ==========
#define OTA_CHUNK_SIZE  1024

