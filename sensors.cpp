#include "sensors.h"

// ============================================================================
//  DISTANCE SENSORS — Implementation
// ============================================================================

// ---------------------------------------------------------------------------
//  Median Filter (fixed-size, no dynamic allocation)
// ---------------------------------------------------------------------------

/// Simple insertion-sort based median for a small fixed window.
/// Operates on a static circular buffer per sensor.
struct MedianFilter {
    float  buffer[MEDIAN_WINDOW];
    uint8_t index;
    uint8_t count;          // how many samples collected so far

    void reset() {
        index = 0;
        count = 0;
        for (uint8_t i = 0; i < MEDIAN_WINDOW; i++) buffer[i] = 0.0f;
    }

    /// Push a new sample and return the current median.
    float push(float value) {
        buffer[index] = value;
        index = (index + 1) % MEDIAN_WINDOW;
        if (count < MEDIAN_WINDOW) count++;

        // Copy into a scratch array and sort (small N — insertion sort is fine)
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

// One filter per sensor — file-scoped, not global.
static MedianFilter s_ir_filter;
static MedianFilter s_us_filter;

// ---------------------------------------------------------------------------
//  Ultrasonic — HC-SR04
// ---------------------------------------------------------------------------

/// Trigger a single ultrasonic ping and return distance in cm.
/// Returns -1.0 on timeout (no echo).
static float us_read_once() {
    // Send 10 µs trigger pulse
    digitalWrite(PIN_US_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_US_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_US_TRIG, LOW);

    // Measure echo duration (with timeout)
    float duration = static_cast<float>(
        pulseIn(PIN_US_ECHO, HIGH, static_cast<unsigned long>(US_TIMEOUT_US))
    );

    if (duration == 0.0f) return -1.0f;       // timeout → no object

    // Speed of sound ≈ 0.0343 cm/µs → distance = duration * 0.0343 / 2
    float cm = duration * 0.01715f;
    return cm;
}

// ---------------------------------------------------------------------------
//  IR — Analog Distance Sensor (Sharp GP2Y0A21YK0F-style)
// ---------------------------------------------------------------------------

/// Read the IR sensor analog voltage and convert to approximate cm.
/// The Sharp GP2Y0A21YK0F has a roughly inverse relationship:
///   distance ≈ 2786 / (analogValue - 10)   (empirical, 10-80 cm range)
/// Tune the constants for your specific sensor + ADC resolution.
static float ir_read_once() {
    int raw_adc = analogRead(PIN_IR);

    // ESP32 ADC: 12-bit → 0–4095, reference ~3.3 V
    // Avoid division by zero
    if (raw_adc < 20) return IR_MAX_CM + 1.0f;     // too close / noise → invalid

    // Empirical conversion (adjust these coefficients for your sensor)
    // Based on Sharp GP2Y0A21YK0F characteristic curve.
    float voltage = raw_adc * (3.3f / 4095.0f);
    if (voltage < 0.3f) return IR_MAX_CM + 1.0f;   // below usable range

    float cm = 29.988f * powf(voltage, -1.173f);    // power-law approximation
    return cm;
}

// ---------------------------------------------------------------------------
//  Validation Helpers
// ---------------------------------------------------------------------------

static bool us_is_valid(float cm) {
    return (cm >= US_MIN_CM && cm <= US_MAX_CM);
}

static bool ir_is_valid(float cm) {
    return (cm >= IR_MIN_CM && cm <= IR_MAX_CM);
}

// ---------------------------------------------------------------------------
//  Public Functions
// ---------------------------------------------------------------------------

void sensors_init() {
    // Ultrasonic pins
    pinMode(PIN_US_TRIG, OUTPUT);
    pinMode(PIN_US_ECHO, INPUT);

    // IR pin — analogRead doesn't strictly need pinMode on ESP32,
    // but being explicit doesn't hurt.
    pinMode(PIN_IR, INPUT);

    // Reset filters
    s_ir_filter.reset();
    s_us_filter.reset();

    DEBUG_PRINTLN("[SENSORS] Initialised (IR + Ultrasonic)");
}

void sensors_read(SensorData& out) {
    out.timestamp_ms = millis();

    // --- Ultrasonic ---
    float us_raw = us_read_once();
    out.ultrasonic.raw_cm  = us_raw;
    out.ultrasonic.valid   = us_is_valid(us_raw);
    out.ultrasonic.filtered_cm = out.ultrasonic.valid
        ? s_us_filter.push(us_raw)
        : s_us_filter.push(US_MAX_CM);   // feed a safe "no obstacle" value on invalid

    // --- IR ---
    float ir_raw = ir_read_once();
    out.ir.raw_cm  = ir_raw;
    out.ir.valid   = ir_is_valid(ir_raw);
    out.ir.filtered_cm = out.ir.valid
        ? s_ir_filter.push(ir_raw)
        : s_ir_filter.push(IR_MAX_CM);

    DEBUG_PRINTF("[SENSORS] US: %.1f cm (filt %.1f, %s)  IR: %.1f cm (filt %.1f, %s)\n",
                 us_raw, out.ultrasonic.filtered_cm, out.ultrasonic.valid ? "OK" : "INV",
                 ir_raw, out.ir.filtered_cm, out.ir.valid ? "OK" : "INV");
}
