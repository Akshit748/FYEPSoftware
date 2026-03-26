#ifndef DECISION_H
#define DECISION_H

#include "config.h"
#include "sensors.h"
#include "motion.h"
#include "gps_mod.h"

// ============================================================================
//  DECISION / SENSOR FUSION MODULE (v2)
// ============================================================================
//  Fuses ultrasonic distance + digital IR trigger into obstacle classification.
//  Manages fall state machine and triggers GSM alerts.
//
//  No actuator outputs — feedback will be delivered via mobile app.
//  The obstacle and fall state are exposed for the app to query.
// ============================================================================

// ---------------------------------------------------------------------------
//  Enums & Structs
// ---------------------------------------------------------------------------

/// Obstacle proximity classification.
enum class ObstacleLevel : uint8_t {
    NONE,       // no obstacle in range
    FAR,        // obstacle between CLOSE and FAR thresholds (ultrasonic)
    CLOSE       // obstacle within CLOSE threshold or IR triggered
};

/// Fall state machine states.
enum class FallState : uint8_t {
    IDLE,               // normal operation
    DETECTED,           // fall just detected — start grace period
    GRACE_PERIOD,       // waiting for cancellation (future: button)
    ALERT_SENT          // SMS sent — waiting for auto-reset
};

/// Snapshot of the decision engine's current state.
/// Exposed for debug logging and future mobile app integration.
struct DecisionState {
    ObstacleLevel obstacle;
    FallState     fall;
    float         us_distance_cm;     // ultrasonic filtered distance
    bool          ir_triggered;       // digital IR close-range flag
    uint32_t      timestamp_ms;
};

// ---------------------------------------------------------------------------
//  Public Interface
// ---------------------------------------------------------------------------

/// Initialise internal state. Call once in setup().
void decision_init();

/// Run one decision cycle.
///   - Fuses ultrasonic distance + IR binary trigger
///   - Advances the fall state machine
///   - Sends GSM alert when grace period expires
///   - No actuator output (motors removed)
void decision_update(const SensorData& sensors,
                     const MotionData& motion,
                     const GPSData&    gps);

/// Read-only access to current state (for debug / future app interface).
DecisionState decision_get_state();

#endif // DECISION_H
