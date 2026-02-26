#include <Arduino.h>
#include "sensors.h"

#define IR_PIN      34
#define TRIG_PIN    5
#define ECHO_PIN    18

#define IR_SAMPLES  5
#define US_SAMPLES  3

// Bubble sort algorithm to find the median 
float median(float *arr, int size) {
    for (int i = 0; i < size - 1; i++) {
        for (int j = i + 1; j < size; j++) {
            if (arr[j] < arr[i]) {
                float t = arr[i];
                arr[i] = arr[j];
                arr[j] = t;
            }
        }
    }
    return arr[size / 2];
}

// Ultrasonic Sensor Reading 
float readUltrasonicRaw() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    if (duration == 0) return -1;

    return duration * 0.034 / 2.0;
}

float getUltrasonic() {
    float readings[US_SAMPLES];

    for (int i = 0; i < US_SAMPLES; i++) {
        readings[i] = readUltrasonicRaw();
        delay(5);
    }

    float value = median(readings, US_SAMPLES);

    if (value < 2 || value > 400) return -1;
    return value;
}

// ---------- Infrared Sensors Reading----------
float readIRRaw() {
    int val = analogRead(IR_PIN);
    float voltage = val * (3.3 / 4095.0);

    if (voltage < 0.2) return -1;

    float distance = 27.86 / (voltage - 0.42);
    return distance;
}

float getIR() {
    float readings[IR_SAMPLES];

    for (int i = 0; i < IR_SAMPLES; i++) {
        readings[i] = readIRRaw();
        delay(2);
    }

    float value = median(readings, IR_SAMPLES);

    if (value < 5 || value > 100) return -1;
    return value;
}

// Initialization + Main Section 
void initSensors() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(IR_PIN, INPUT);
}

SensorData readSensors() {
    SensorData data;
    data.irDistance = getIR();
    data.ultrasonicDistance = getUltrasonic();
    return data;
}
