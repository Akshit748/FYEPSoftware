#ifndef GSM_H
#define GSM_H

#include "config.h"
#include "gps_mod.h"

// ============================================================================
//  GSM MODULE (SIM800/900)
// ============================================================================
//  Provides functions for sending SMS and (optionally) HTTP requests.
//  Uses basic AT commands over Hardware Serial2.
// ============================================================================

// ---------------------------------------------------------------------------
//  Public Interface
// ---------------------------------------------------------------------------

/// Initialise hardware serial for the GSM module. Call once in setup().
void gsm_init();

/// Send an emergency SMS containing the fall alert message and GPS link.
/// `gps` should be the most recent valid GPS fix.
/// Returns true if the modem acknowledged the send command.
bool gsm_send_fall_alert(const GPSData& gps);

/// Generic SMS send: deliver `message` to `phone_number`.
/// Returns true if the modem acknowledged.
bool gsm_send_sms(const char* phone_number, const char* message);

/// Placeholder for HTTP POST alert (future use).
/// Returns true on success.
bool gsm_send_http_alert(const char* url, const char* payload);

/// Check if the modem is responsive (AT → OK).
bool gsm_is_ready();

#endif // GSM_H
