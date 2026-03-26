#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

// ============================================================================
//  DISTANCE SENSORS MODULE (IR + Ultrasonic)
// ============================================================================

// ---------------------------------------------------------------------------
//  Data Structures
// ---------------------------------------------------------------------------

/// Raw + filtered reading from a single distance sensor.
struct DistanceReading {
    float raw_cm;           // most recent raw reading
    float filtered_cm;      // after median filter
    bool  valid;            // true if reading is within sensor's usable range
};

/// Combined output from both distance sensors.
struct SensorData {
    DistanceReading ir;
    DistanceReading ultrasonic;
    uint32_t        timestamp_ms;   // millis() when this sample was taken
};

// ---------------------------------------------------------------------------
//  Public Interface
// ---------------------------------------------------------------------------

/// Call once in setup(). Configures pins for HC-SR04 and IR.
void sensors_init();

/// Call each loop iteration. Reads both sensors, applies median filter,
/// and populates `out` with fresh data.
void sensors_read(SensorData& out);

#endif // SENSORS_H
