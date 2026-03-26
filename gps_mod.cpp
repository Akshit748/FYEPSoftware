#include "gps_mod.h"
#include <TinyGPSPlus.h>

// ============================================================================
//  GPS — Implementation
// ============================================================================

// File-scoped state (no globals)
static TinyGPSPlus  s_gps;
static GPSData      s_last_valid;       // cached last-known-good position
static uint32_t     s_last_read_ms = 0;

// ESP32-S3 Hardware Serial1 object.
// We configure it in gps_init() with begin(baud, config, rx, tx).
static HardwareSerial& GpsSerial = Serial1;

// ---------------------------------------------------------------------------
//  Public Functions
// ---------------------------------------------------------------------------

void gps_init() {
    GpsSerial.begin(BAUD_GPS, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

    // Zero out the cached position
    s_last_valid = {};
    s_last_valid.fix_valid = false;

    DEBUG_PRINTLN("[GPS] Initialised on Serial1");
}

void gps_read(GPSData& out) {
    // Rate-limit the processing to avoid hogging CPU
    uint32_t now = millis();
    if (now - s_last_read_ms < GPS_READ_INTERVAL_MS) {
        // Return the cached data without re-parsing
        out = s_last_valid;
        out.timestamp_ms = now;
        return;
    }
    s_last_read_ms = now;

    // Feed all available bytes into TinyGPS++
    while (GpsSerial.available() > 0) {
        s_gps.encode(GpsSerial.read());
    }

    out.timestamp_ms = now;

    // Check for a valid location update
    if (s_gps.location.isValid() && s_gps.location.isUpdated()) {
        out.latitude   = s_gps.location.lat();
        out.longitude  = s_gps.location.lng();
        out.fix_valid  = true;
        out.fix_age_ms = s_gps.location.age();
        out.satellites = s_gps.satellites.value();

        // Cache as last-known-good
        s_last_valid = out;

        DEBUG_PRINTF("[GPS] Fix: lat=%.6f  lon=%.6f  sats=%u  age=%u ms\n",
                     out.latitude, out.longitude, out.satellites, out.fix_age_ms);
    } else {
        // No fresh fix — return the cached position with fix_valid = false
        out = s_last_valid;
        out.fix_valid    = false;
        out.timestamp_ms = now;

        DEBUG_PRINTF("[GPS] No fix (sats=%u, age=%u ms)\n",
                     s_gps.satellites.value(), s_gps.location.age());
    }
}

GPSData gps_last_valid() {
    return s_last_valid;
}
