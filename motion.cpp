#include "motion.h"
#include <Wire.h>
#include <math.h>

// ============================================================================
//  MPU6050 — Direct Register Interface
// ============================================================================
//  Using raw register reads keeps the dependency list at Wire.h only.
//  Register addresses from the MPU-6000/MPU-6050 Register Map (RM-MPU-6000A).
// ============================================================================

// Key register addresses
static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
static constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;
static constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;
static constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;  // start of 14-byte burst
static constexpr uint8_t REG_WHO_AM_I      = 0x75;

// Scale factors for default full-scale ranges
// Accel ±2 g → 16384 LSB/g,  Gyro ±250 °/s → 131 LSB/(°/s)
static constexpr float ACCEL_SCALE = 16384.0f;
static constexpr float GYRO_SCALE  = 131.0f;

// ---------------------------------------------------------------------------
//  Low-level I2C helpers
// ---------------------------------------------------------------------------

static void mpu_write_reg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

static uint8_t mpu_read_reg(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    return Wire.read();
}

/// Read a 16-bit signed value from two consecutive registers (big-endian).
static int16_t read_16bit(uint8_t* buf, uint8_t offset) {
    return static_cast<int16_t>(
        (static_cast<uint16_t>(buf[offset]) << 8) | buf[offset + 1]
    );
}

// ---------------------------------------------------------------------------
//  Public Functions
// ---------------------------------------------------------------------------

bool motion_init() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);  // 400 kHz Fast Mode

    // Check WHO_AM_I (should return 0x68)
    uint8_t id = mpu_read_reg(REG_WHO_AM_I);
    if (id != 0x68) {
        DEBUG_PRINTF("[MOTION] MPU6050 not found (WHO_AM_I = 0x%02X)\n", id);
        return false;
    }

    // Wake up (clear SLEEP bit)
    mpu_write_reg(REG_PWR_MGMT_1, 0x00);
    delay(50);  // brief stabilisation

    // Accel full-scale ±2 g (default, bits 4:3 = 00)
    mpu_write_reg(REG_ACCEL_CONFIG, 0x00);

    // Gyro full-scale ±250 °/s (default, bits 4:3 = 00)
    mpu_write_reg(REG_GYRO_CONFIG, 0x00);

    DEBUG_PRINTLN("[MOTION] MPU6050 initialised (±2 g, ±250 °/s)");
    return true;
}

void motion_read(MotionData& out) {
    out.timestamp_ms = millis();

    // --- Burst-read 14 bytes: accel(6) + temp(2) + gyro(6) ---
    uint8_t buf[14];
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)14);

    for (uint8_t i = 0; i < 14 && Wire.available(); i++) {
        buf[i] = Wire.read();
    }

    // Parse raw 16-bit values
    int16_t raw_ax = read_16bit(buf, 0);
    int16_t raw_ay = read_16bit(buf, 2);
    int16_t raw_az = read_16bit(buf, 4);
    // bytes 6-7 = temperature (skipped)
    int16_t raw_gx = read_16bit(buf, 8);
    int16_t raw_gy = read_16bit(buf, 10);
    int16_t raw_gz = read_16bit(buf, 12);

    // Convert to physical units
    out.raw.ax = raw_ax / ACCEL_SCALE;
    out.raw.ay = raw_ay / ACCEL_SCALE;
    out.raw.az = raw_az / ACCEL_SCALE;
    out.raw.gx = raw_gx / GYRO_SCALE;
    out.raw.gy = raw_gy / GYRO_SCALE;
    out.raw.gz = raw_gz / GYRO_SCALE;

    // --- Derived values ---

    // Total acceleration magnitude
    out.accel_magnitude = sqrtf(
        out.raw.ax * out.raw.ax +
        out.raw.ay * out.raw.ay +
        out.raw.az * out.raw.az
    );

    // Tilt angle from vertical.
    // When upright, az ≈ +1 g.  tilt = acos(az / magnitude) in degrees.
    // Clamp to avoid NaN from acos domain errors.
    float cos_angle = out.raw.az / out.accel_magnitude;
    cos_angle = constrain(cos_angle, -1.0f, 1.0f);
    out.tilt_deg = acosf(cos_angle) * (180.0f / M_PI);

    // --- Fall Detection Logic ---
    //
    //  A fall is declared when BOTH:
    //    1. Acceleration spike > FALL_ACCEL_G  (impact)
    //    2. Tilt angle > FALL_TILT_DEG         (not upright)
    //
    //  Future improvement: add a "stillness confirmation" phase after impact
    //  (low variance in accel for N ms → confirms the person is on the ground).
    //
    out.fall_detected = (out.accel_magnitude > FALL_ACCEL_G)
                     && (out.tilt_deg > FALL_TILT_DEG);

    DEBUG_PRINTF("[MOTION] mag=%.2fg  tilt=%.1f°  fall=%s\n",
                 out.accel_magnitude, out.tilt_deg,
                 out.fall_detected ? "YES" : "no");
}
