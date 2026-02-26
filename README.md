Smart Modular IoT Assistive Stick – Alpha Firmware

What the Alpha Version Currently Does

The alpha firmware provides a working baseline for obstacle detection and basic fall detection using the ESP32-S3.

It reads data from:

1. An ultrasonic sensor for mid-range obstacle detection

2. An IR sensor for close-range obstacle detection

3. An MPU (accelerometer and gyroscope) for motion and tilt monitoring


The system applies simple median filtering to reduce sensor noise and rejects obviously invalid readings. It uses basic sensor fusion logic to determine whether an obstacle is confirmed, based on combined sensor input and movement data.

For fall detection, it checks for:

1. A sudden acceleration spike

2. A large tilt angle


If thresholds are exceeded, it flags a fall event.

 The main loop runs at approximately 20 Hz and outputs debug information through Serial. Alerts are currently represented through LED activation and serial messages.



The next development stage will focus on improving robustness and reliability. Planned improvements include:

1. Proper debounce counters to reduce false obstacle alerts

2. Two-stage fall detection with post-impact stillness confirmation

3. Non-blocking timing using millis() instead of delay()

4. A state machine architecture for better alert handling

5. BLE communication with acknowledgement handling

6. GSM emergency message implementation

7. Battery monitoring and power management

8. Directional vibration feedback integration


The alpha version establishes a functional foundation, and upcoming updates will transition the system toward a more stable, field-ready beta release.
