// ============================================================================
//  SMART ASSISTIVE WALKING STICK — Main Entry Point
// ============================================================================
//  Microcontroller: ESP32-S3 (Arduino framework)
//
//  Program Flow (20 Hz loop):
//    1. Read distance sensors (IR + Ultrasonic)
//    2. Read IMU (MPU6050)
//    3. Read GPS
//    4. Run decision engine (sensor fusion + fall state machine)
//    5. Actuators are driven inside decision_update()
//    6. Enforce loop timing
//
//  Author: [Your Name]
//  Date:   [Date]
// ============================================================================

#include "config.h"
#include "sensors.h"
#include "motion.h"
#include "gps_mod.h"
#include "gsm.h"
#include "decision.h"

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
    // Debug serial
    Serial.begin(BAUD_DEBUG);
    while (!Serial && millis() < 2000) { /* wait up to 2 s for USB serial */ }
    DEBUG_PRINTLN("========================================");
    DEBUG_PRINTLN("  Smart Assistive Walking Stick v1.0");
    DEBUG_PRINTLN("========================================");

    // Initialise each subsystem
    sensors_init();

    if (!motion_init()) {
        DEBUG_PRINTLN("[MAIN] WARNING: MPU6050 init failed — fall detection disabled");
        // System continues without fall detection rather than halting
    }

    gps_init();
    gsm_init();
    decision_init();

    DEBUG_PRINTLN("[MAIN] All subsystems initialised. Entering main loop.\n");
}

// ---------------------------------------------------------------------------
//  loop()
// ---------------------------------------------------------------------------
void loop() {
    g_loop_start = millis();

    // 1. Read distance sensors
    sensors_read(g_sensors);

    // 2. Read IMU
    motion_read(g_motion);

    // 3. Read GPS (internally rate-limited)
    gps_read(g_gps);

    // 4. Run decision engine (fuses data, drives motors, sends alerts)
    decision_update(g_sensors, g_motion, g_gps);

    // ----- Periodic debug summary (every ~2 seconds) -----
    g_loop_count++;
    if (g_loop_count % 40 == 0) {   // 40 iterations × 50 ms = 2 s
        DecisionState ds = decision_get_state();
        DEBUG_PRINTLN("--- STATUS ---");
        DEBUG_PRINTF("  Fused distance : %.1f cm\n", ds.fused_distance_cm);
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

    // 5. Enforce 20 Hz loop timing (non-blocking)
    uint32_t elapsed = millis() - g_loop_start;
    if (elapsed < LOOP_PERIOD_MS) {
        delay(LOOP_PERIOD_MS - elapsed);
    }
}
