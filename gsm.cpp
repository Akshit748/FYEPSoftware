#include "gsm.h"
#include <stdio.h>

// ============================================================================
//  GSM — Implementation (SIM800/900 AT Commands)
// ============================================================================

static HardwareSerial& GsmSerial = Serial2;

// ---------------------------------------------------------------------------
//  Internal Helpers
// ---------------------------------------------------------------------------

/// Send an AT command string and wait for an expected response substring.
/// Returns true if `expected` was found within the timeout.
static bool at_send_wait(const char* cmd, const char* expected,
                         uint32_t timeout_ms = GSM_CMD_TIMEOUT_MS)
{
    // Flush any stale data
    while (GsmSerial.available()) GsmSerial.read();

    GsmSerial.println(cmd);
    DEBUG_PRINTF("[GSM] >> %s\n", cmd);

    uint32_t start = millis();
    String   response;
    response.reserve(128);

    while (millis() - start < timeout_ms) {
        while (GsmSerial.available()) {
            char c = GsmSerial.read();
            response += c;
        }
        if (response.indexOf(expected) >= 0) {
            DEBUG_PRINTF("[GSM] << %s\n", response.c_str());
            return true;
        }
        delay(10);  // small yield to avoid tight spin
    }

    DEBUG_PRINTF("[GSM] TIMEOUT waiting for '%s'. Got: %s\n",
                 expected, response.c_str());
    return false;
}

/// Build the SMS body string.  Uses a static buffer (no heap allocation).
static const char* build_fall_message(const GPSData& gps) {
    static char buf[200];

    if (gps.fix_valid || (gps.latitude != 0.0 && gps.longitude != 0.0)) {
        snprintf(buf, sizeof(buf),
                 "Emergency: Fall detected!\n"
                 "Location: https://maps.google.com/?q=%.6f,%.6f",
                 gps.latitude, gps.longitude);
    } else {
        snprintf(buf, sizeof(buf),
                 "Emergency: Fall detected!\n"
                 "Location: GPS unavailable");
    }
    return buf;
}

// ---------------------------------------------------------------------------
//  Public Functions
// ---------------------------------------------------------------------------

void gsm_init() {
    GsmSerial.begin(BAUD_GSM, SERIAL_8N1, PIN_GSM_RX, PIN_GSM_TX);
    delay(1000);  // allow modem boot

    // Basic handshake — retry a few times
    for (uint8_t i = 0; i < 3; i++) {
        if (at_send_wait("AT", "OK")) {
            DEBUG_PRINTLN("[GSM] Modem responded OK");
            break;
        }
        delay(500);
    }

    // Set SMS to text mode
    at_send_wait("AT+CMGF=1", "OK");

    // Optional: set character set to GSM
    at_send_wait("AT+CSCS=\"GSM\"", "OK");

    DEBUG_PRINTLN("[GSM] Initialised (text mode)");
}

bool gsm_is_ready() {
    return at_send_wait("AT", "OK", 2000);
}

bool gsm_send_sms(const char* phone_number, const char* message) {
    // Step 1: AT+CMGS="<number>"
    char cmd[50];
    snprintf(cmd, sizeof(cmd), "AT+CMGS=\"%s\"", phone_number);

    // Send the command; modem responds with '>' prompt
    if (!at_send_wait(cmd, ">", GSM_CMD_TIMEOUT_MS)) {
        DEBUG_PRINTLN("[GSM] Failed to get '>' prompt for SMS");
        return false;
    }

    // Step 2: Send the message body, terminated by Ctrl+Z (0x1A)
    GsmSerial.print(message);
    GsmSerial.write(0x1A);

    // Step 3: Wait for acknowledgment
    bool ok = at_send_wait("", "+CMGS:", 15000);  // SMS send can be slow
    if (ok) {
        DEBUG_PRINTLN("[GSM] SMS sent successfully");
    } else {
        DEBUG_PRINTLN("[GSM] SMS send FAILED or timed out");
    }
    return ok;
}

bool gsm_send_fall_alert(const GPSData& gps) {
    const char* message = build_fall_message(gps);
    DEBUG_PRINTF("[GSM] Sending fall alert to %s\n", GSM_TARGET_NUMBER);
    return gsm_send_sms(GSM_TARGET_NUMBER, message);
}

bool gsm_send_http_alert(const char* url, const char* payload) {
    // -----------------------------------------------------------------
    //  Placeholder for future HTTP POST via SIM800 GPRS.
    //  Typical flow:
    //    AT+SAPBR (bearer setup)
    //    AT+HTTPINIT
    //    AT+HTTPPARA="URL", url
    //    AT+HTTPDATA  → write payload
    //    AT+HTTPACTION=1  (POST)
    //    AT+HTTPTERM
    //  This requires GPRS configuration (APN, etc.) which is
    //  carrier-specific. Implement when needed.
    // -----------------------------------------------------------------
    DEBUG_PRINTF("[GSM] HTTP alert stub called (url=%s)\n", url);
    (void)payload;  // suppress unused warning
    return false;
}
