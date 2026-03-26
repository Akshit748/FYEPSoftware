#ifndef DECISION_H
#define DECISION_H

#include "config.h"
#include "sensors.h"
#include "motion.h"
#include "gps_mod.h"

// ============================================================================
//  DECISION / SENSOR FUSION MODULE
// ============================================================================
//  Central logic that fuses distance sensors, handles the fall state machine,
//  and drives actuator outputs (vibration motors, GSM alerts).
// ============================================================================

// ---------------------------------------------------------------------------
//  Enums & Structs
// ---------------------------------------------------------------------------

/// Obstacle proximity classification.
enum class ObstacleLevel : uint8_t {
    NONE,       // no obstacle in range
    FAR,        // obstacle between CLOSE and FAR thresholds
    CLOSE       // obstacle within CLOSE threshold
};

/// Vibration pattern type.
enum class VibPattern : uint8_t {
    OFF,
    SHORT_PULSE,    // single 200 ms pulse (far obstacle)
    DOUBLE_PULSE,   // two quick pulses   (close obstacle)
    CONTINUOUS      // sustained vibration (fall)
};

/// Fall state machine states.
/// Structured for easy extension (e.g., add WAITING_CANCEL when button is wired).
enum class FallState : uint8_t {
    IDLE,               // normal operation
    DETECTED,           // fall just detected — start grace period
    GRACE_PERIOD,       // waiting for cancellation
    ALERT_SENT          // SMS sent — wait for reset
};

/// Snapshot of the decision engine's current state (useful for debug / UI).
struct DecisionState {
    ObstacleLevel obstacle;
    FallState     fall;
    VibPattern    vib_pattern;
    float         fused_distance_cm;    // effective obstacle distance
};

// ---------------------------------------------------------------------------
//  Public Interface
// ---------------------------------------------------------------------------

/// Initialise motor pins and internal state. Call once in setup().
void decision_init();

/// Run one decision cycle.
///   - Fuses sensor data into an obstacle classification.
///   - Advances the fall state machine.
///   - Drives vibration motors accordingly.
///   - Sends GSM alert when the grace period expires.
void decision_update(const SensorData& sensors,
                     const MotionData& motion,
                     const GPSData&    gps);

/// Read-only access to current state (for debug / display).
DecisionState decision_get_state();

#endif // DECISION_H
