#include "decision.h"
#include "gsm.h"

// ============================================================================
//  DECISION ENGINE — Implementation
// ============================================================================

// ---------------------------------------------------------------------------
//  File-scoped State
// ---------------------------------------------------------------------------

static DecisionState s_state;

// Debounce counters for obstacle detection
static uint8_t s_close_count = 0;   // consecutive "close" detections
static uint8_t s_far_count   = 0;   // consecutive "far" detections

// Fall state machine timing
static uint32_t s_fall_detect_time = 0;

// Vibration pattern execution state (non-blocking)
static uint32_t s_vib_start_ms    = 0;
static bool     s_vib_active      = false;
static VibPattern s_current_vib   = VibPattern::OFF;

// ---------------------------------------------------------------------------
//  Motor Control Helpers
// ---------------------------------------------------------------------------

static void motors_on() {
    digitalWrite(PIN_MOTOR_LEFT,  HIGH);
    digitalWrite(PIN_MOTOR_RIGHT, HIGH);
}

static void motors_off() {
    digitalWrite(PIN_MOTOR_LEFT,  LOW);
    digitalWrite(PIN_MOTOR_RIGHT, LOW);
}

/// Non-blocking vibration pattern driver.
/// Call every loop iteration; it manages timing internally.
static void vib_update(VibPattern pattern) {
    uint32_t now     = millis();
    uint32_t elapsed = now - s_vib_start_ms;

    // If the requested pattern changed, restart
    if (pattern != s_current_vib) {
        s_current_vib  = pattern;
        s_vib_start_ms = now;
        s_vib_active   = false;
        elapsed        = 0;
    }

    switch (pattern) {
        case VibPattern::OFF:
            motors_off();
            break;

        case VibPattern::SHORT_PULSE:
            // |===ON===|---OFF---| (one-shot, 200 ms on)
            if (elapsed < VIB_SHORT_MS) {
                motors_on();
            } else {
                motors_off();
            }
            break;

        case VibPattern::DOUBLE_PULSE:
            // |==ON==|.gap.|==ON==|---OFF---|
            if (elapsed < VIB_QUICK_MS) {
                motors_on();
            } else if (elapsed < VIB_QUICK_MS + VIB_GAP_MS) {
                motors_off();
            } else if (elapsed < 2 * VIB_QUICK_MS + VIB_GAP_MS) {
                motors_on();
            } else {
                motors_off();
            }
            break;

        case VibPattern::CONTINUOUS:
            if (elapsed < VIB_FALL_MS) {
                motors_on();
            } else {
                motors_off();
            }
            break;
    }
}

// ---------------------------------------------------------------------------
//  Sensor Fusion
// ---------------------------------------------------------------------------

/// Combine IR and ultrasonic readings into a single effective obstacle distance.
///
/// Strategy:
///   1. If both sensors report valid readings in their overlapping zone
///      (50–80 cm), take the minimum (conservative / safer).
///   2. If only IR is valid (10–80 cm), trust IR alone.
///   3. If only ultrasonic is valid (50–300 cm), trust ultrasonic alone.
///   4. If neither is valid, return a large "no obstacle" value.
///
/// This avoids false positives from ground reflections (IR) and
/// stale echoes (ultrasonic) by requiring either confirmation from
/// both or a strong single-sensor reading within its reliable range.

static float fuse_distance(const SensorData& s) {
    bool ir_ok = s.ir.valid;
    bool us_ok = s.ultrasonic.valid && (s.ultrasonic.filtered_cm <= US_VALID_MAX_CM);

    float ir_cm = s.ir.filtered_cm;
    float us_cm = s.ultrasonic.filtered_cm;

    // Both valid in overlapping zone (50–80 cm)
    if (ir_ok && us_ok && ir_cm >= 50.0f && us_cm <= 80.0f) {
        return fminf(ir_cm, us_cm);     // conservative: trust the closer reading
    }

    // Close range: IR only
    if (ir_ok && ir_cm < 50.0f) {
        return ir_cm;
    }

    // Far range: ultrasonic only
    if (us_ok && us_cm > 80.0f) {
        return us_cm;
    }

    // General case: if both valid, prefer the closer reading
    if (ir_ok && us_ok) {
        return fminf(ir_cm, us_cm);
    }

    // Single sensor fallback
    if (ir_ok) return ir_cm;
    if (us_ok) return us_cm;

    // Neither valid
    return US_MAX_CM;   // "no obstacle"
}

// ---------------------------------------------------------------------------
//  Obstacle Classification with Debounce
// ---------------------------------------------------------------------------

static ObstacleLevel classify_obstacle(float distance_cm) {
    // Determine raw classification
    ObstacleLevel raw;
    if (distance_cm <= OBSTACLE_CLOSE_CM) {
        raw = ObstacleLevel::CLOSE;
    } else if (distance_cm <= OBSTACLE_FAR_CM) {
        raw = ObstacleLevel::FAR;
    } else {
        raw = ObstacleLevel::NONE;
    }

    // Debounce: require DEBOUNCE_COUNT consecutive detections
    // before promoting to a higher alert level.
    // Downgrading (CLOSE → NONE) is immediate to avoid "stuck" alerts.
    switch (raw) {
        case ObstacleLevel::CLOSE:
            s_close_count = (s_close_count < 255) ? s_close_count + 1 : 255;
            s_far_count   = 0;
            if (s_close_count >= DEBOUNCE_COUNT) return ObstacleLevel::CLOSE;
            return s_state.obstacle;  // hold previous level during ramp-up
            break;

        case ObstacleLevel::FAR:
            s_far_count   = (s_far_count < 255) ? s_far_count + 1 : 255;
            s_close_count = 0;
            if (s_far_count >= DEBOUNCE_COUNT) return ObstacleLevel::FAR;
            return s_state.obstacle;
            break;

        case ObstacleLevel::NONE:
        default:
            s_close_count = 0;
            s_far_count   = 0;
            return ObstacleLevel::NONE;
    }
}

// ---------------------------------------------------------------------------
//  Fall State Machine
// ---------------------------------------------------------------------------

static FallState advance_fall_state(FallState current, bool fall_now, const GPSData& gps) {
    uint32_t now = millis();

    switch (current) {
        case FallState::IDLE:
            if (fall_now) {
                s_fall_detect_time = now;
                DEBUG_PRINTLN("[DECISION] Fall DETECTED — entering grace period");
                return FallState::DETECTED;
            }
            return FallState::IDLE;

        case FallState::DETECTED:
            // Immediately transition to grace period
            // (vibration is triggered by the decision_update caller)
            return FallState::GRACE_PERIOD;

        case FallState::GRACE_PERIOD:
            // -------------------------------------------------------
            //  FUTURE: Check cancel button here.
            //  if (cancel_button_pressed()) return FallState::IDLE;
            // -------------------------------------------------------

            if (now - s_fall_detect_time >= FALL_GRACE_MS) {
                DEBUG_PRINTLN("[DECISION] Grace period expired — sending alert");
                gsm_send_fall_alert(gps);
                return FallState::ALERT_SENT;
            }
            return FallState::GRACE_PERIOD;

        case FallState::ALERT_SENT:
            // Stay here until the system is manually reset or a timeout.
            // For now, auto-reset after 30 seconds so the stick remains usable.
            if (now - s_fall_detect_time >= FALL_GRACE_MS + 30000) {
                DEBUG_PRINTLN("[DECISION] Fall state auto-reset");
                return FallState::IDLE;
            }
            return FallState::ALERT_SENT;
    }

    return FallState::IDLE;  // fallback (should never reach)
}

// ---------------------------------------------------------------------------
//  Choose Vibration Pattern
// ---------------------------------------------------------------------------

static VibPattern choose_vibration(ObstacleLevel obstacle, FallState fall) {
    // Fall always takes priority
    if (fall == FallState::DETECTED || fall == FallState::GRACE_PERIOD) {
        return VibPattern::CONTINUOUS;
    }

    switch (obstacle) {
        case ObstacleLevel::CLOSE: return VibPattern::DOUBLE_PULSE;
        case ObstacleLevel::FAR:   return VibPattern::SHORT_PULSE;
        case ObstacleLevel::NONE:
        default:                   return VibPattern::OFF;
    }
}

// ---------------------------------------------------------------------------
//  Public Functions
// ---------------------------------------------------------------------------

void decision_init() {
    pinMode(PIN_MOTOR_LEFT,  OUTPUT);
    pinMode(PIN_MOTOR_RIGHT, OUTPUT);
    motors_off();

    s_state.obstacle          = ObstacleLevel::NONE;
    s_state.fall              = FallState::IDLE;
    s_state.vib_pattern       = VibPattern::OFF;
    s_state.fused_distance_cm = US_MAX_CM;

    s_close_count = 0;
    s_far_count   = 0;

    DEBUG_PRINTLN("[DECISION] Initialised");
}

void decision_update(const SensorData& sensors,
                     const MotionData& motion,
                     const GPSData&    gps)
{
    // 1. Sensor fusion → effective distance
    s_state.fused_distance_cm = fuse_distance(sensors);

    // 2. Obstacle classification (with debounce)
    s_state.obstacle = classify_obstacle(s_state.fused_distance_cm);

    // 3. Fall state machine
    s_state.fall = advance_fall_state(s_state.fall, motion.fall_detected, gps);

    // 4. Decide vibration pattern
    s_state.vib_pattern = choose_vibration(s_state.obstacle, s_state.fall);

    // 5. Drive motors (non-blocking pattern update)
    vib_update(s_state.vib_pattern);

    DEBUG_PRINTF("[DECISION] dist=%.1f cm  obstacle=%d  fall=%d  vib=%d\n",
                 s_state.fused_distance_cm,
                 static_cast<int>(s_state.obstacle),
                 static_cast<int>(s_state.fall),
                 static_cast<int>(s_state.vib_pattern));
}

DecisionState decision_get_state() {
    return s_state;
}
