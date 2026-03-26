// ============================================================================
//  SMART ASSISTIVE WALKING STICK — Main Entry Point (v2)
// ============================================================================
//  Microcontroller: ESP32-S3 (Arduino framework)
//  Power: Raw VBAT (3.0–4.2V) via TP4056 → ESP32 VIN
//
//  Changes from v1:
//    - IR sensor is digital (FC-51), not analog
//    - Vibration motors removed — no actuator output
//    - Feedback will be delivered via mobile app (future)
//    - System detects obstacles + falls, sends GSM alerts
//
//  Program Flow (20 Hz loop):
//    1. Read ultrasonic + digital IR sensors
//    2. Read IMU (MPU6050)
//    3. Read GPS
//    4. Run decision engine (fusion + fall state machine + GSM)
//    5. Enforce loop timing
// ============================================================================

#include "config.h"
#include "sensors.h"
#include "motion.h"
#include "gps_mod.h"
#include "gsm.h"
#include "decision.h"
#include "test_console.h"

// ---------------------------------------------------------------------------
//  Per-loop data buffers (stack-allocated, reused each cycle)
// ---------------------------------------------------------------------------
static SensorData  g_sensors;
static MotionData  g_motion;
static GPSData     g_gps;

// Loop timing
static uint32_t    g_loop_start = 0;
static uint32_t    g_loop_count = 0;

// ---------------------------------------------------------------------------
//  setup()
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(BAUD_DEBUG);
    while (!Serial && millis() < 2000) { /* wait up to 2 s for USB serial */ }
    DEBUG_PRINTLN("========================================");
    DEBUG_PRINTLN("  Smart Assistive Walking Stick v2.0");
    DEBUG_PRINTLN("  No motors — detection + alerts only");
    DEBUG_PRINTLN("========================================");

    sensors_init();

    if (!motion_init()) {
        DEBUG_PRINTLN("[MAIN] WARNING: MPU6050 init failed — fall detection disabled");
    }

    gps_init();
    gsm_init();
    decision_init();
    test_console_init();

    DEBUG_PRINTLN("[MAIN] All subsystems initialised. Entering main loop.\n");
}

// ---------------------------------------------------------------------------
//  loop()
// ---------------------------------------------------------------------------
void loop() {
    g_loop_start = millis();

    // 1. Read distance sensors (ultrasonic + digital IR)
    sensors_read(g_sensors);

    // 2. Read IMU
    motion_read(g_motion);

    // 2b. Allow simulated fall from test console
    extern bool test_console_fall_simulated();
    if (test_console_fall_simulated()) {
        g_motion.fall_detected = true;
    }

    // 3. Read GPS (internally rate-limited to 1 Hz)
    gps_read(g_gps);

    // 4. Run decision engine (fuses data, manages fall state, sends alerts)
    decision_update(g_sensors, g_motion, g_gps);

    // ----- Periodic debug summary (every ~2 seconds) -----
    g_loop_count++;
    if (g_loop_count % 40 == 0) {
        DecisionState ds = decision_get_state();
        DEBUG_PRINTLN("--- STATUS ---");
        DEBUG_PRINTF("  Ultrasonic     : %.1f cm\n", ds.us_distance_cm);
        DEBUG_PRINTF("  IR triggered   : %s\n", ds.ir_triggered ? "YES" : "no");
        DEBUG_PRINTF("  Obstacle level : %d (0=NONE 1=FAR 2=CLOSE)\n",
                     static_cast<int>(ds.obstacle));
        DEBUG_PRINTF("  Fall state     : %d (0=IDLE 1=DET 2=GRACE 3=SENT)\n",
                     static_cast<int>(ds.fall));
        DEBUG_PRINTF("  GPS fix        : %s  (%.6f, %.6f)\n",
                     g_gps.fix_valid ? "YES" : "NO",
                     g_gps.latitude, g_gps.longitude);
        DEBUG_PRINTF("  Loop time      : %u ms\n", millis() - g_loop_start);
        DEBUG_PRINTLN("--------------\n");
    }

    // 5. Poll test console for serial commands (non-blocking)
    test_console_poll();

    // 6. Enforce 20 Hz loop timing
    uint32_t elapsed = millis() - g_loop_start;
    if (elapsed < LOOP_PERIOD_MS) {
        delay(LOOP_PERIOD_MS - elapsed);
    }
}
