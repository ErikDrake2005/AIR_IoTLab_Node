#pragma once
#include <Arduino.h>

// ========== PINOUT & GPIO ==========
#define PIN_SLEEP_STATUS 32  // Output: LOW=Sleep, HIGH=Awake
#define PIN_WAKEUP_GPIO  33  // Input: Kích hoạt High để đánh thức DeepSleep

// ========== UART ==========
#define UART_BAUD       921600   
#define UART_RX_PIN     16
#define UART_TX_PIN     17

// ========== RELAY & SENSORS ==========
#define CTL_ON_RELAY    13
#define CTL_OFF_RELAY   12
#define FANA            27
#define FAN1            14

#define RS485_BAUD      9600  // All sensors on the shared bus must use 9600 baud
#define RS485_TX_PIN    26
#define RS485_RX_PIN    25
#define PIN_RS485_DE    4
#define RS485_MIN_REQUEST_INTERVAL_MS 1000UL

#define RS485_ADDR_CO2       1  // EP-MED-CO2-01
#define RS485_ADDR_CH4       2  // EP-ENV-CH4-01
#define RS485_ADDR_TEMP_HUM  3  // ES35-SW

// ========== SYSTEM ==========
#define DEVICE_ID       "AIR_VL_01"
#define DEFAULT_MEASURES_PER_DAY 4
#define SECONDS_IN_DAY  86400UL
#define TIME_SYNC_INTERVAL_SECONDS  3600
#define WAKEUP_IDLE_TIMEOUT 15000 // 15s chờ lệnh sau khi wake up
#define GMT_OFFSET_SEC  25200 // Vietnam UTC+7 = 7*3600
