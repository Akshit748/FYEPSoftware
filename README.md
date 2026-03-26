# Smart Assistive Walking Stick — Firmware

ESP32-S3 firmware for a modular assistive walking stick with obstacle detection,
fall detection, GPS tracking, and GSM emergency alerts.

## Architecture

```
main.cpp                    ← setup() + loop() at 20 Hz
  │
  ├── sensors.h / .cpp      ← IR + Ultrasonic (median filter, range validation)
  ├── motion.h / .cpp       ← MPU6050 (accel/gyro, fall detection)
  ├── gps_mod.h / .cpp      ← NEO-6M via TinyGPS++ (lat/lon, last-valid cache)
  ├── gsm.h / .cpp          ← SIM800/900 AT commands (SMS, HTTP stub)
  └── decision.h / .cpp     ← Sensor fusion, debounce, fall state machine,
                               vibration patterns, GSM alert trigger
config.h                    ← All pins, thresholds, timing constants
```

### Data Flow (each loop cycle)

```
[HC-SR04] ──► sensors_read() ──┐
[IR Analog] ──► sensors_read() ──┤
[MPU6050] ──► motion_read() ────┼──► decision_update() ──► Motors / GSM
[NEO-6M] ──► gps_read() ───────┘
```

## Wiring

| Peripheral     | Signal    | ESP32-S3 GPIO |
|---------------|-----------|---------------|
| HC-SR04       | TRIG      | 5             |
| HC-SR04       | ECHO      | 18            |
| IR Sensor     | Analog    | 34            |
| MPU6050       | SDA       | 21            |
| MPU6050       | SCL       | 22            |
| GPS NEO-6M    | TX→RX     | 16            |
| GPS NEO-6M    | RX←TX     | 17            |
| GSM SIM800    | TX→RX     | 25            |
| GSM SIM800    | RX←TX     | 26            |
| Motor Left    | Base      | 32            |
| Motor Right   | Base      | 33            |

## Build (Arduino IDE)

1. Install **ESP32 board support** (Espressif Arduino core).
2. Install **TinyGPS++** library via Library Manager.
3. Set Board → `ESP32S3 Dev Module`.
4. Open `src/main.cpp` as the sketch.  Place all other `.h` / `.cpp` files
   in the same sketch folder (or use PlatformIO — see below).
5. Edit `config.h`:
   - Set `GSM_TARGET_NUMBER` to the emergency contact phone number.
   - Tune thresholds if needed.
6. Upload.

### PlatformIO (recommended)

```ini
; platformio.ini
[env:esp32s3]
platform  = espressif32
board     = esp32-s3-devkitc-1
framework = arduino
lib_deps  = mikalhart/TinyGPSPlus@^1.0.3
monitor_speed = 115200
```

## Key Design Decisions

- **Median filter** (window = 5) on both distance sensors rejects spike noise.
- **Debounce** (3 consecutive readings) prevents single-sample false obstacles.
- **Sensor fusion** uses range-aware combining: IR trusted <50 cm, ultrasonic
  trusted >80 cm, both cross-checked in the 50–80 cm overlap zone.
- **Fall state machine** (IDLE → DETECTED → GRACE_PERIOD → ALERT_SENT)
  has a 5-second grace period before SMS.  Structured for easy addition of a
  cancel button.
- **Non-blocking vibration** patterns driven by a time-based state machine
  inside `decision.cpp` — no `delay()` calls in the pattern driver.
- **GPS read rate-limiting** (1 Hz) reduces CPU load from NMEA parsing.

## Future Improvements

- [ ] Wire a cancel button to abort fall alerts during grace period
- [ ] Add stillness confirmation after impact (low accel variance for N ms)
- [ ] Implement HTTP POST alerts via GPRS (stub in `gsm.cpp`)
- [ ] Add deep sleep / power management for longer battery life
- [ ] Replace polling loop with FreeRTOS tasks for better timing
- [ ] Add battery voltage monitoring via ADC
