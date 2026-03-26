#ifndef GPS_MOD_H
#define GPS_MOD_H

#include "config.h"

// ============================================================================
//  GPS MODULE (NEO-6M via TinyGPS++)
// ============================================================================
//  Header is named gps_mod.h to avoid collision with system "gps.h" on some
//  toolchains.
// ============================================================================

// ---------------------------------------------------------------------------
//  Data Structures
// ---------------------------------------------------------------------------

struct GPSData {
    double   latitude;
    double   longitude;
    bool     fix_valid;         // true if we have a current fix
    uint32_t fix_age_ms;        // age of the last fix (from TinyGPS++)
    uint32_t satellites;        // visible satellites
    uint32_t timestamp_ms;      // millis() when this was updated
};

// ---------------------------------------------------------------------------
//  Public Interface
// ---------------------------------------------------------------------------

/// Initialise hardware serial for GPS. Call once in setup().
void gps_init();

/// Feed available bytes from the GPS serial and update `out`.
/// This is non-blocking — returns immediately if no bytes are available.
void gps_read(GPSData& out);

/// Return the last known valid position. Useful for alert messages
/// when the current fix is stale.
GPSData gps_last_valid();

#endif // GPS_MOD_H
