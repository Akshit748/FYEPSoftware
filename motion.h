#ifndef MOTION_H
#define MOTION_H

#include "config.h"

// ============================================================================
//  MOTION / FALL DETECTION MODULE (MPU6050)
// ============================================================================

// ---------------------------------------------------------------------------
//  Data Structures
// ---------------------------------------------------------------------------

/// Raw accelerometer + gyroscope readings in physical units.
struct MotionRaw {
    float ax, ay, az;       // acceleration in g
    float gx, gy, gz;       // angular rate in deg/s
};

/// Processed motion state used by the decision layer.
struct MotionData {
    MotionRaw raw;
    float accel_magnitude;  // sqrt(ax² + ay² + az²)
    float tilt_deg;         // angle from vertical (0 = upright, 90 = horizontal)
    bool  fall_detected;    // true when both accel + tilt thresholds are exceeded
    uint32_t timestamp_ms;
};

// ---------------------------------------------------------------------------
//  Public Interface
// ---------------------------------------------------------------------------

/// Initialise MPU6050 over I2C. Call once in setup().
/// Returns true on success, false if the sensor is not responding.
bool motion_init();

/// Read accelerometer + gyroscope, compute magnitude and tilt,
/// and evaluate fall condition. Populates `out`.
void motion_read(MotionData& out);

#endif // MOTION_H
