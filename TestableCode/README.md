# Smart Assistive Walking Stick — Firmware v2

ESP32-S3 firmware for a modular assistive walking stick with obstacle detection,
fall detection, GPS tracking, and GSM emergency alerts.

## Changes from v1

- **IR sensor**: analog Sharp GP2Y → digital FC-51 (binary obstacle trigger)
- **Vibration motors**: removed entirely (feedback via future mobile app)
- **Sensor fusion**: simplified — ultrasonic provides distance, IR provides
  binary close-range override
- **Power**: no external regulator, no boost converter. Raw VBAT from TP4056
  to ESP32 VIN pin. ESP32 onboard LDO provides 3.3V for MPU6050 + GPS.
- **HC-SR04 range**: reduced `US_VALID_MAX_CM` from 300 to 200 (running at 3.7V)

## Architecture

```
main.cpp                    ← setup() + loop() at 20 Hz
  │
  ├── sensors.h / .cpp      ← HC-SR04 (median filter) + FC-51 digital IR (debounce)
  ├── motion.h / .cpp       ← MPU6050 (accel/gyro, fall detection) [unchanged]
  ├── gps_mod.h / .cpp      ← NEO-6M via TinyGPS++ [unchanged]
  ├── gsm.h / .cpp          ← SIM800L AT commands (SMS alerts) [unchanged]
  └── decision.h / .cpp     ← Sensor fusion, fall state machine, GSM trigger
                               (no motor output)
config.h                    ← All pins, thresholds, timing constants
```

### Data Flow

```
[HC-SR04] → distance (cm) ──┐
[FC-51 IR] → obstacle (bool) ┤
[MPU6050] → fall (bool) ─────┼──► decision_update() ──► GSM alert (on fall)
[NEO-6M] → lat/lon ──────────┘         │
                                        └──► DecisionState (for future app)
```

## Wiring

| Peripheral     | Signal       | ESP32 GPIO | Power     | Notes                     |
|---------------|-------------|-----------|-----------|---------------------------|
| HC-SR04       | TRIG         | 5         | VBAT      |                           |
| HC-SR04       | ECHO         | 18        | VBAT      | Via 1k/2k voltage divider |
| FC-51 IR      | Signal (DO)  | 34        | VBAT      | Active-low (LOW=obstacle) |
| MPU6050       | SDA          | 21        | 3V3       |                           |
| MPU6050       | SCL          | 22        | 3V3       | 4.7k pull-ups if needed   |
| GPS NEO-6M    | TX→RX        | 16        | 3V3       |                           |
| GPS NEO-6M    | RX←TX        | 17        | 3V3       |                           |
| GSM SIM800L   | TX→RX        | 25        | VBAT      | Direct battery power      |
| GSM SIM800L   | RX←TX        | 26        | VBAT      |                           |

### Power Distribution

```
LiPo 3.7V → TP4056 → VBAT rail ──┬── ESP32 VIN (onboard LDO → 3V3 out)
                                   ├── HC-SR04 VCC
                                   ├── FC-51 IR VCC
                                   └── SIM800L VCC (direct, up to 2A peak)

ESP32 3V3 out ──┬── MPU6050 VCC
                └── NEO-6M VCC
```

## Build (Arduino IDE)

1. Install **ESP32 board support** (Espressif Arduino core).
2. Install **TinyGPS++** library via Library Manager.
3. Board → `ESP32S3 Dev Module`.
4. Edit `config.h`:
   - Set `GSM_TARGET_NUMBER` to the emergency contact number.
   - Verify `IR_ACTIVE_STATE` matches your FC-51 module (most are active-low).
   - Tune thresholds if needed.
5. Upload.

### PlatformIO

```ini
[env:esp32s3]
platform  = espressif32
board     = esp32-s3-devkitc-1
framework = arduino
lib_deps  = mikalhart/TinyGPSPlus@^1.0.3
monitor_speed = 115200
```

## Key Design Decisions

- **Ultrasonic median filter** (window=5) rejects spike noise.
- **Ultrasonic debounce** (3 consecutive reads) prevents false positives.
- **IR digital debounce** (3 consecutive reads in sensor module) confirms
  close-range obstacles before reporting.
- **IR as override**: if IR triggers, obstacle is CLOSE regardless of what
  the ultrasonic says. Catches thin/angled/ground-level objects.
- **Fall state machine** (IDLE → DETECTED → GRACE_PERIOD → ALERT_SENT)
  with 5-second grace period before SMS.
- **No actuator output**: decision state is exposed via `decision_get_state()`
  for future mobile app to query via BLE or Wi-Fi.

## FC-51 IR Sensor Notes

- The onboard potentiometer adjusts detection range (~2–30 cm).
- Most modules are active-low: OUT=LOW when obstacle detected.
- If your module is active-high, change `IR_ACTIVE_STATE` in `config.h` to `HIGH`.
- The sensor works at 3.3–5V; at 3.7V (VBAT) it operates fine.

## Future Improvements

- [ ] Mobile app integration (BLE/Wi-Fi) for obstacle + fall feedback
- [ ] Cancel button for fall alerts during grace period
- [ ] Stillness confirmation after fall impact
- [ ] HTTP POST alerts via GPRS (stub in gsm.cpp)
- [ ] Battery voltage monitoring via ADC
- [ ] Power management (deep sleep between alerts)
