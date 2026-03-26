#include "test_console.h"
#include "sensors.h"
#include "motion.h"
#include "gps_mod.h"
#include "gsm.h"
#include "decision.h"

// ============================================================================
//  SERIAL TEST CONSOLE — Implementation
// ============================================================================

// Flag to simulate a fall from the console
static bool s_simulate_fall = false;

// Accessor for the decision module to check simulated falls
bool test_console_fall_simulated() { return s_simulate_fall; }

// ---------------------------------------------------------------------------
//  Command Handlers
// ---------------------------------------------------------------------------

static void cmd_ultrasonic() {
    SensorData data;
    sensors_read(data);
    Serial.println("=== ULTRASONIC TEST ===");
    Serial.printf("  Raw:      %.1f cm\n", data.ultrasonic.raw_cm);
    Serial.printf("  Filtered: %.1f cm\n", data.ultrasonic.filtered_cm);
    Serial.printf("  Valid:    %s\n", data.ultrasonic.valid ? "YES" : "NO");
    if (!data.ultrasonic.valid) {
        Serial.println("  Check: wiring, voltage divider, object in range?");
    }
    Serial.println();
}

static void cmd_ir() {
    SensorData data;
    sensors_read(data);
    Serial.println("=== IR SENSOR TEST ===");
    Serial.printf("  Pin state:    %s\n", data.ir.raw_triggered ? "ACTIVE (obstacle)" : "IDLE (clear)");
    Serial.printf("  Debounced:    %s\n", data.ir.triggered ? "TRIGGERED" : "clear");
    Serial.printf("  Consec count: %u / %u\n", data.ir.consec_count, IR_DEBOUNCE_COUNT);
    Serial.printf("  Active state: %s (config)\n", IR_ACTIVE_STATE == LOW ? "LOW" : "HIGH");
    if (data.ir.raw_triggered) {
        Serial.println("  Obstacle detected by IR sensor");
    } else {
        Serial.println("  No obstacle — try placing hand 5-10 cm in front");
    }
    Serial.println();
}

static void cmd_mpu() {
    MotionData data;
    motion_read(data);
    Serial.println("=== MPU6050 TEST ===");
    Serial.printf("  Accel:     ax=%.2f  ay=%.2f  az=%.2f  (g)\n",
                  data.raw.ax, data.raw.ay, data.raw.az);
    Serial.printf("  Gyro:      gx=%.1f  gy=%.1f  gz=%.1f  (deg/s)\n",
                  data.raw.gx, data.raw.gy, data.raw.gz);
    Serial.printf("  Magnitude: %.2f g  (threshold: %.1f g)\n",
                  data.accel_magnitude, FALL_ACCEL_G);
    Serial.printf("  Tilt:      %.1f deg  (threshold: %.1f deg)\n",
                  data.tilt_deg, FALL_TILT_DEG);
    Serial.printf("  Fall:      %s\n", data.fall_detected ? "YES!" : "no");
    if (data.accel_magnitude < 0.5f) {
        Serial.println("  WARNING: magnitude too low — check I2C wiring");
    }
    Serial.println();
}

static void cmd_gps() {
    GPSData data;
    gps_read(data);
    Serial.println("=== GPS TEST ===");
    Serial.printf("  Fix valid:  %s\n", data.fix_valid ? "YES" : "NO");
    Serial.printf("  Latitude:   %.6f\n", data.latitude);
    Serial.printf("  Longitude:  %.6f\n", data.longitude);
    Serial.printf("  Satellites: %u\n", data.satellites);
    Serial.printf("  Fix age:    %u ms\n", data.fix_age_ms);
    if (!data.fix_valid && data.latitude == 0.0 && data.longitude == 0.0) {
        Serial.println("  No fix yet — GPS needs clear sky, cold start ~30s");
    }
    Serial.println();
}

static void cmd_test_sms() {
    Serial.println("=== GSM SMS TEST ===");
    Serial.println("  Checking modem...");
    if (!gsm_is_ready()) {
        Serial.println("  FAIL: GSM modem not responding (check power + wiring)");
        Serial.println();
        return;
    }
    Serial.println("  Modem OK. Sending test SMS...");

    GPSData gps = gps_last_valid();
    char msg[160];
    if (gps.latitude != 0.0 || gps.longitude != 0.0) {
        snprintf(msg, sizeof(msg),
                 "TEST: Smart stick system check.\n"
                 "Location: https://maps.google.com/?q=%.6f,%.6f",
                 gps.latitude, gps.longitude);
    } else {
        snprintf(msg, sizeof(msg),
                 "TEST: Smart stick system check.\n"
                 "Location: GPS not available");
    }

    bool ok = gsm_send_sms(GSM_TARGET_NUMBER, msg);
    Serial.printf("  Result: %s\n", ok ? "SMS SENT" : "SEND FAILED");
    Serial.printf("  Target: %s\n", GSM_TARGET_NUMBER);
    Serial.println();
}

static void cmd_simulate_fall() {
    Serial.println("=== SIMULATING FALL ===");
    Serial.println("  Setting fall flag — grace period starts now");
    Serial.printf("  SMS will send in %u ms unless reset (type 'r')\n", FALL_GRACE_MS);
    s_simulate_fall = true;
    Serial.println();
}

static void cmd_reset_fall() {
    Serial.println("=== RESET FALL STATE ===");
    s_simulate_fall = false;
    // Note: this only clears the simulated flag.
    // The actual fall state in decision.cpp will auto-reset after 30s,
    // or you can power-cycle to reset immediately.
    Serial.println("  Simulation flag cleared");
    Serial.println("  Note: decision state auto-resets in ~30s after alert");
    Serial.println();
}

static void cmd_dump() {
    Serial.println("=== FULL SYSTEM DUMP ===");

    SensorData sd;
    sensors_read(sd);
    MotionData md;
    motion_read(md);
    GPSData gd;
    gps_read(gd);
    DecisionState ds = decision_get_state();

    Serial.println("  [Ultrasonic]");
    Serial.printf("    Distance: %.1f cm (filtered)  Valid: %s\n",
                  sd.ultrasonic.filtered_cm, sd.ultrasonic.valid ? "Y" : "N");

    Serial.println("  [IR]");
    Serial.printf("    Triggered: %s  Raw: %s\n",
                  sd.ir.triggered ? "YES" : "no",
                  sd.ir.raw_triggered ? "active" : "idle");

    Serial.println("  [MPU6050]");
    Serial.printf("    Mag: %.2fg  Tilt: %.1f deg  Fall: %s\n",
                  md.accel_magnitude, md.tilt_deg,
                  md.fall_detected ? "YES" : "no");

    Serial.println("  [GPS]");
    Serial.printf("    Fix: %s  Pos: (%.6f, %.6f)  Sats: %u\n",
                  gd.fix_valid ? "Y" : "N",
                  gd.latitude, gd.longitude, gd.satellites);

    Serial.println("  [Decision]");
    Serial.printf("    Obstacle: %d (0=NONE 1=FAR 2=CLOSE)\n",
                  static_cast<int>(ds.obstacle));
    Serial.printf("    Fall:     %d (0=IDLE 1=DET 2=GRACE 3=SENT)\n",
                  static_cast<int>(ds.fall));

    Serial.println("  [GSM]");
    Serial.printf("    Modem:  %s\n", gsm_is_ready() ? "responding" : "NOT responding");
    Serial.printf("    Target: %s\n", GSM_TARGET_NUMBER);

    Serial.println("=======================\n");
}

static void cmd_help() {
    Serial.println("=== TEST CONSOLE COMMANDS ===");
    Serial.println("  u  Ultrasonic sensor test");
    Serial.println("  i  IR sensor test");
    Serial.println("  m  MPU6050 / IMU test");
    Serial.println("  g  GPS status");
    Serial.println("  s  Send test SMS");
    Serial.println("  f  Simulate fall (triggers grace period → SMS)");
    Serial.println("  r  Reset fall simulation");
    Serial.println("  d  Dump full system status");
    Serial.println("  h  This help message");
    Serial.println("=============================\n");
}

// ---------------------------------------------------------------------------
//  Public Functions
// ---------------------------------------------------------------------------

void test_console_init() {
    Serial.println("[TEST] Serial test console active — type 'h' for commands\n");
}

void test_console_poll() {
    if (!Serial.available()) return;

    char c = Serial.read();

    // Consume any trailing newline / carriage return
    while (Serial.available()) {
        char peek = Serial.peek();
        if (peek == '\n' || peek == '\r') {
            Serial.read();
        } else {
            break;
        }
    }

    switch (c) {
        case 'u': cmd_ultrasonic(); break;
        case 'i': cmd_ir();         break;
        case 'm': cmd_mpu();        break;
        case 'g': cmd_gps();        break;
        case 's': cmd_test_sms();   break;
        case 'f': cmd_simulate_fall(); break;
        case 'r': cmd_reset_fall(); break;
        case 'd': cmd_dump();       break;
        case 'h':
        case '?': cmd_help();       break;
        case '\n':
        case '\r': break;  // ignore bare newlines
        default:
            Serial.printf("[TEST] Unknown command '%c' — type 'h' for help\n\n", c);
            break;
    }
}
