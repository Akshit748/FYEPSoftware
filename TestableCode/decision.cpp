#include "decision.h"
#include "gsm.h"

// ============================================================================
//  DECISION ENGINE — Implementation (v2)
// ============================================================================
//  Changes from v1:
//    - Sensor fusion redesigned: ultrasonic provides distance,
//      digital IR acts as binary close-range override
//    - All motor/vibration code removed
//    - Output is detection state + GSM alerts only
//    - DecisionState exposed for future mobile app integration
// ============================================================================

// ---------------------------------------------------------------------------
//  File-scoped State
// ---------------------------------------------------------------------------

static DecisionState s_state;

// Debounce counters for ultrasonic-based obstacle classification
static uint8_t s_close_count = 0;
static uint8_t s_far_count   = 0;

// Fall state machine timing
static uint32_t s_fall_detect_time = 0;

// ---------------------------------------------------------------------------
//  Sensor Fusion
// ---------------------------------------------------------------------------

/// Fuse ultrasonic distance + digital IR trigger into an obstacle classification.
///
/// Strategy:
///   1. Ultrasonic provides the distance estimate (2–200 cm at 3.7V).
///   2. Digital IR acts as a close-range override / confirmation:
///      - If IR is triggered, obstacle is CLOSE regardless of ultrasonic reading.
///        This catches objects the ultrasonic might miss (thin, angled, ground-level).
///      - If IR is clear, classification is based solely on ultrasonic distance.
///   3. Ultrasonic is debounced (DEBOUNCE_COUNT consecutive reads).
///      IR has its own debounce in the sensor module (IR_DEBOUNCE_COUNT).
///
/// This is simpler than the v1 analog-IR fusion because we no longer have
/// two overlapping distance estimates to reconcile.

static ObstacleLevel fuse_and_classify(const SensorData& s) {
    bool us_ok  = s.ultrasonic.valid &&
                  (s.ultrasonic.filtered_cm <= US_VALID_MAX_CM);
    float us_cm = s.ultrasonic.filtered_cm;
    bool ir_hit = s.ir.triggered;

    // --- IR override: if IR says something is close, trust it ---
    // The FC-51 typically triggers at 2–30 cm (adjustable via pot).
    // If it fires, that's a confirmed close-range obstacle.
    if (ir_hit) {
        // Reset ultrasonic debounce counters to avoid stale state
        s_close_count = DEBOUNCE_COUNT;
        s_far_count   = 0;
        return ObstacleLevel::CLOSE;
    }

    // --- Ultrasonic-only classification ---
    if (!us_ok) {
        // No valid ultrasonic reading and IR is clear → no obstacle
        s_close_count = 0;
        s_far_count   = 0;
        return ObstacleLevel::NONE;
    }

    // Determine raw level from ultrasonic distance
    ObstacleLevel raw;
    if (us_cm <= OBSTACLE_CLOSE_CM) {
        raw = ObstacleLevel::CLOSE;
    } else if (us_cm <= OBSTACLE_FAR_CM) {
        raw = ObstacleLevel::FAR;
    } else {
        raw = ObstacleLevel::NONE;
    }

    // Debounce: require DEBOUNCE_COUNT consecutive reads to promote.
    // Downgrade to NONE is immediate (safety: clear the alert fast).
    switch (raw) {
        case ObstacleLevel::CLOSE:
            s_close_count = (s_close_count < 255) ? s_close_count + 1 : 255;
            s_far_count   = 0;
            if (s_close_count >= DEBOUNCE_COUNT) return ObstacleLevel::CLOSE;
            return s_state.obstacle;   // hold previous level during ramp-up

        case ObstacleLevel::FAR:
            s_far_count   = (s_far_count < 255) ? s_far_count + 1 : 255;
            s_close_count = 0;
            if (s_far_count >= DEBOUNCE_COUNT) return ObstacleLevel::FAR;
            return s_state.obstacle;

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

static FallState advance_fall_state(FallState current, bool fall_now,
                                    const GPSData& gps)
{
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
            // Immediately transition to grace period.
            // In v1 this triggered vibration — now it's just a state change.
            // FUTURE: notify mobile app here via BLE/Wi-Fi.
            return FallState::GRACE_PERIOD;

        case FallState::GRACE_PERIOD:
            // -------------------------------------------------------
            //  FUTURE: Check cancel button here.
            //  if (cancel_button_pressed()) {
            //      DEBUG_PRINTLN("[DECISION] Fall cancelled by user");
            //      return FallState::IDLE;
            //  }
            //
            //  FUTURE: Check mobile app cancel via BLE/Wi-Fi.
            // -------------------------------------------------------

            if (now - s_fall_detect_time >= FALL_GRACE_MS) {
                DEBUG_PRINTLN("[DECISION] Grace period expired — sending SMS alert");
                gsm_send_fall_alert(gps);
                return FallState::ALERT_SENT;
            }
            return FallState::GRACE_PERIOD;

        case FallState::ALERT_SENT:
            // Auto-reset after 30 seconds so the stick remains usable.
            // FUTURE: reset via mobile app acknowledgment instead.
            if (now - s_fall_detect_time >= FALL_GRACE_MS + 30000) {
                DEBUG_PRINTLN("[DECISION] Fall state auto-reset");
                return FallState::IDLE;
            }
            return FallState::ALERT_SENT;
    }

    return FallState::IDLE;
}

// ---------------------------------------------------------------------------
//  Public Functions
// ---------------------------------------------------------------------------

void decision_init() {
    // No motor pins to configure in v2

    s_state.obstacle       = ObstacleLevel::NONE;
    s_state.fall           = FallState::IDLE;
    s_state.us_distance_cm = US_MAX_CM;
    s_state.ir_triggered   = false;
    s_state.timestamp_ms   = 0;

    s_close_count = 0;
    s_far_count   = 0;

    DEBUG_PRINTLN("[DECISION] Initialised (no actuators — app feedback mode)");
}

void decision_update(const SensorData& sensors,
                     const MotionData& motion,
                     const GPSData&    gps)
{
    s_state.timestamp_ms   = millis();
    s_state.us_distance_cm = sensors.ultrasonic.filtered_cm;
    s_state.ir_triggered   = sensors.ir.triggered;

    // 1. Sensor fusion → obstacle classification
    s_state.obstacle = fuse_and_classify(sensors);

    // 2. Fall state machine
    s_state.fall = advance_fall_state(s_state.fall, motion.fall_detected, gps);

    // 3. No actuator output — state is available via decision_get_state()
    //    for future mobile app integration (BLE/Wi-Fi).

    DEBUG_PRINTF("[DECISION] us=%.1f cm  ir=%s  obstacle=%d  fall=%d\n",
                 s_state.us_distance_cm,
                 s_state.ir_triggered ? "HIT" : "clr",
                 static_cast<int>(s_state.obstacle),
                 static_cast<int>(s_state.fall));
}

DecisionState decision_get_state() {
    return s_state;
}
