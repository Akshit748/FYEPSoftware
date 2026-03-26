#include "sensors.h"

// ============================================================================
//  DISTANCE SENSORS — Implementation (v2)
// ============================================================================
//  Changes from v1:
//    - IR sensor is now digital (FC-51): simple digitalRead + debounce
//    - Removed: analog read, power-law conversion, IR median filter
//    - Ultrasonic median filter retained unchanged
// ============================================================================

// ---------------------------------------------------------------------------
//  Median Filter (for ultrasonic only)
// ---------------------------------------------------------------------------

struct MedianFilter {
    float   buffer[MEDIAN_WINDOW];
    uint8_t index;
    uint8_t count;

    void reset() {
        index = 0;
        count = 0;
        for (uint8_t i = 0; i < MEDIAN_WINDOW; i++) buffer[i] = 0.0f;
    }

    float push(float value) {
        buffer[index] = value;
        index = (index + 1) % MEDIAN_WINDOW;
        if (count < MEDIAN_WINDOW) count++;

        // Insertion-sort on a small scratch array
        float sorted[MEDIAN_WINDOW];
        for (uint8_t i = 0; i < count; i++) sorted[i] = buffer[i];

        for (uint8_t i = 1; i < count; i++) {
            float key = sorted[i];
            int8_t j = i - 1;
            while (j >= 0 && sorted[j] > key) {
                sorted[j + 1] = sorted[j];
                j--;
            }
            sorted[j + 1] = key;
        }

        return sorted[count / 2];
    }
};

static MedianFilter s_us_filter;

// ---------------------------------------------------------------------------
//  Ultrasonic — HC-SR04
// ---------------------------------------------------------------------------

static float us_read_once() {
    digitalWrite(PIN_US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);

    float duration = static_cast<float>(
        pulseIn(PIN_US_ECHO, HIGH, static_cast<unsigned long>(US_TIMEOUT_US))
    );

    if (duration == 0.0f) return -1.0f;   // timeout — no object detected

    // Speed of sound ≈ 0.0343 cm/µs → distance = duration × 0.0343 / 2
    float cm = duration * 0.01715f;
    return cm;
}

static bool us_is_valid(float cm) {
    return (cm >= US_MIN_CM && cm <= US_MAX_CM);
}

// ---------------------------------------------------------------------------
//  IR — Digital Obstacle Sensor (FC-51 or similar)
// ---------------------------------------------------------------------------

/// Read the digital IR sensor and apply debounce.
/// The FC-51 outputs LOW when an obstacle is present (active-low by default).
/// We debounce by requiring IR_DEBOUNCE_COUNT consecutive active reads.
static void ir_read(IRReading& out) {
    bool pin_state = digitalRead(PIN_IR);
    out.raw_triggered = (pin_state == IR_ACTIVE_STATE);

    if (out.raw_triggered) {
        // Increment consecutive count, saturate at 255
        if (out.consec_count < 255) out.consec_count++;
    } else {
        // Reset immediately when sensor clears
        out.consec_count = 0;
    }

    // Debounced output: confirmed only after N consecutive active reads
    out.triggered = (out.consec_count >= IR_DEBOUNCE_COUNT);
}

// ---------------------------------------------------------------------------
//  Public Functions
// ---------------------------------------------------------------------------

void sensors_init() {
    // Ultrasonic pins
    pinMode(PIN_US_TRIG, OUTPUT);
    pinMode(PIN_US_ECHO, INPUT);

    // IR digital input
    // GPIO 34 is input-only on ESP32 (no internal pull-up available).
    // The FC-51 module has its own pull-up, so INPUT is fine.
    pinMode(PIN_IR, INPUT);

    // Reset ultrasonic filter
    s_us_filter.reset();

    DEBUG_PRINTLN("[SENSORS] Initialised (HC-SR04 + FC-51 digital IR)");
}

void sensors_read(SensorData& out) {
    out.timestamp_ms = millis();

    // --- Ultrasonic ---
    float us_raw = us_read_once();
    out.ultrasonic.raw_cm  = us_raw;
    out.ultrasonic.valid   = us_is_valid(us_raw);
    out.ultrasonic.filtered_cm = out.ultrasonic.valid
        ? s_us_filter.push(us_raw)
        : s_us_filter.push(US_MAX_CM);

    // --- IR (digital) ---
    ir_read(out.ir);

    DEBUG_PRINTF("[SENSORS] US: %.1f cm (filt %.1f, %s)  IR: %s (raw=%s, cnt=%u)\n",
                 us_raw, out.ultrasonic.filtered_cm,
                 out.ultrasonic.valid ? "OK" : "INV",
                 out.ir.triggered ? "OBSTACLE" : "clear",
                 out.ir.raw_triggered ? "active" : "idle",
                 out.ir.consec_count);
}
