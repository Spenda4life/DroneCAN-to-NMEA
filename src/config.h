#pragma once

#include <driver/gpio.h>

// --- CAN / DroneCAN ---
#define CAN_TX_PIN       GPIO_NUM_5
#define CAN_RX_PIN       GPIO_NUM_4
#define DRONECAN_NODE_ID 100          // pick any unused node ID 1-127

// --- RS485 Serial ---
#define UART2_TX_PIN     17
#define UART2_RX_PIN     16
#define RS485_DE_PIN     18
#define SERIAL_BAUD      4800         // GX1600 GPS IN expects 4800 by default

// --- WiFi ---
#define WIFI_AP_SSID     "ESP32-GPS"
#define WIFI_AP_PASS     "dronecan1"
#define WIFI_TCP_PORT    10110        // IANA assigned for NMEA-0183 over IP
#define WIFI_UDP_BROADCAST true

// --- NMEA Sentence Enable Flags ---
#define EMIT_RMC         true   // Required by GX1600
#define EMIT_GGA         true   // Required by OpenPlotter
#define EMIT_VTG         true
#define EMIT_HDM         true   // Magnetic heading - requires magnetometer data
#define EMIT_GSA         true   // DOP values
#define EMIT_XDR_BARO    false  // Phase 2 - barometric pressure
#define EMIT_XDR_TEMP    false  // Phase 2 - air temperature

// --- Timing ---
#define GPS_OUTPUT_HZ    5      // Rate to emit RMC/GGA/VTG (match Fix2 rate)
#define HDM_OUTPUT_HZ    10     // Magnetic heading output rate
#define AUX_OUTPUT_HZ    1      // GSA / XDR output rate
#define CAN_SILENCE_TIMEOUT_MS 5000  // Emit void sentences after this

// --- LED ---
#define LED_STATUS_PIN   2      // Onboard LED GPIO 2
