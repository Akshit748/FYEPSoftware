#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ============================================================================
//  SMART ASSISTIVE WALKING STICK — GLOBAL CONFIGURATION (v2)
// ============================================================================
//  Revised to match actual hardware:
//    - Digital IR obstacle sensor (FC-51) replaces analog Sharp GP2Y
//    - Vibration motors removed (feedback via mobile app)
//    - No voltage regulator / boost (raw VBAT powers sensors)
//    - HC-SR04 ECHO protected by 1k/2k voltage divider
// ============================================================================

// ---------------------------------------------------------------------------
//  GPIO Pin Assignments
// ---------------------------------------------------------------------------

// Ultrasonic (HC-SR04)
constexpr uint8_t PIN_US_TRIG       = 5;
constexpr uint8_t PIN_US_ECHO       = 18;    // via 1k/2k voltage divider

// IR obstacle sensor — digital output (FC-51 or similar)
// LOW = obstacle detected, HIGH = clear (most modules are active-low)
constexpr uint8_t PIN_IR            = 34;

// MPU6050 (I2C)
constexpr uint8_t PIN_I2C_SDA       = 21;
constexpr uint8_t PIN_I2C_SCL       = 22;

// GPS (Hardware Serial1)
constexpr uint8_t PIN_GPS_RX        = 16;    // ESP32 RX ← GPS TX
constexpr uint8_t PIN_GPS_TX        = 17;    // ESP32 TX → GPS RX

// GSM (Hardware Serial2)
constexpr uint8_t PIN_GSM_RX        = 25;    // ESP32 RX ← GSM TX
constexpr uint8_t PIN_GSM_TX        = 26;    // ESP32 TX → GSM RX

// NOTE: Motor pins removed — feedback handled by mobile app

// ---------------------------------------------------------------------------
//  Serial Baud Rates
// ---------------------------------------------------------------------------
constexpr uint32_t BAUD_DEBUG       = 115200;
constexpr uint32_t BAUD_GPS         = 9600;
constexpr uint32_t BAUD_GSM         = 9600;

// ---------------------------------------------------------------------------
//  Ultrasonic Sensor Thresholds
// ---------------------------------------------------------------------------
constexpr float US_MIN_CM           = 2.0f;     // reject readings below this
constexpr float US_MAX_CM           = 400.0f;   // reject readings above this
constexpr float US_VALID_MAX_CM     = 200.0f;   // reduced from 300: running at ~3.7V
constexpr float US_TIMEOUT_US       = 25000.0f; // echo timeout in microseconds

// ---------------------------------------------------------------------------
//  IR Obstacle Sensor (Digital)
// ---------------------------------------------------------------------------
//  The FC-51 outputs LOW when an obstacle is detected (active-low).
//  If your module outputs HIGH on detection, flip this constant.
constexpr bool IR_ACTIVE_STATE      = LOW;       // pin state when obstacle present
constexpr uint8_t IR_DEBOUNCE_COUNT = 3;         // consecutive reads to confirm

// ---------------------------------------------------------------------------
//  Sensor Fusion / Obstacle Thresholds
// ---------------------------------------------------------------------------
constexpr float OBSTACLE_CLOSE_CM   = 40.0f;    // "close" obstacle (ultrasonic)
constexpr float OBSTACLE_FAR_CM     = 120.0f;   // "far" obstacle (ultrasonic)
constexpr uint8_t DEBOUNCE_COUNT    = 3;         // consecutive US detections to confirm

// ---------------------------------------------------------------------------
//  Median Filter (ultrasonic only — IR is digital, no filter needed)
// ---------------------------------------------------------------------------
constexpr uint8_t MEDIAN_WINDOW     = 5;

// ---------------------------------------------------------------------------
//  MPU6050 / Fall Detection
// ---------------------------------------------------------------------------
constexpr uint8_t  MPU_ADDR         = 0x68;
constexpr float    FALL_ACCEL_G     = 2.5f;      // acceleration magnitude threshold
constexpr float    FALL_TILT_DEG    = 60.0f;     // tilt angle threshold
constexpr uint32_t FALL_GRACE_MS    = 5000;      // grace period before SMS (ms)

// ---------------------------------------------------------------------------
//  GPS
// ---------------------------------------------------------------------------
constexpr uint32_t GPS_READ_INTERVAL_MS = 1000;

// ---------------------------------------------------------------------------
//  GSM
// ---------------------------------------------------------------------------
// CHANGE THIS to the actual emergency contact number
constexpr char     GSM_TARGET_NUMBER[]  = "+1234567890";
constexpr uint32_t GSM_CMD_TIMEOUT_MS   = 5000;

// ---------------------------------------------------------------------------
//  Main Loop Timing
// ---------------------------------------------------------------------------
constexpr uint32_t LOOP_PERIOD_MS   = 50;        // 50 ms → 20 Hz target

// ---------------------------------------------------------------------------
//  Debug Toggle
// ---------------------------------------------------------------------------
#define DEBUG_ENABLED   1

#if DEBUG_ENABLED
    #define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
    #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
    #define DEBUG_PRINTF(...)  Serial.printf(__VA_ARGS__)
#else
    #define DEBUG_PRINT(...)
    #define DEBUG_PRINTLN(...)
    #define DEBUG_PRINTF(...)
#endif

#endif // CONFIG_H
