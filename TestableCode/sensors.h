#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

// ============================================================================
//  DISTANCE SENSORS MODULE (Ultrasonic + Digital IR)
// ============================================================================
//  HC-SR04 provides ranged distance measurement (2–200 cm at 3.7V).
//  FC-51 digital IR provides binary close-range obstacle detection.
// ============================================================================

// ---------------------------------------------------------------------------
//  Data Structures
// ---------------------------------------------------------------------------

/// Ultrasonic reading with median filtering.
struct UltrasonicReading {
    float raw_cm;           // most recent raw reading
    float filtered_cm;      // after median filter
    bool  valid;            // true if reading is within usable range
};

/// Digital IR obstacle reading with debounce.
struct IRReading {
    bool  raw_triggered;    // true if sensor pin is in active state right now
    bool  triggered;        // true after debounce confirms obstacle
    uint8_t consec_count;   // consecutive active reads (for debounce)
};

/// Combined output from both sensors.
struct SensorData {
    UltrasonicReading ultrasonic;
    IRReading         ir;
    uint32_t          timestamp_ms;
};

// ---------------------------------------------------------------------------
//  Public Interface
// ---------------------------------------------------------------------------

/// Call once in setup(). Configures pins for HC-SR04 and IR.
void sensors_init();

/// Call each loop iteration. Reads both sensors, applies filtering/debounce,
/// and populates `out` with fresh data.
void sensors_read(SensorData& out);

#endif // SENSORS_H
