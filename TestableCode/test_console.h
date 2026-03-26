#ifndef TEST_CONSOLE_H
#define TEST_CONSOLE_H

#include "config.h"

// ============================================================================
//  SERIAL TEST CONSOLE
// ============================================================================
//  Lightweight interactive test interface via Serial Monitor.
//  Type single-character commands to test individual subsystems.
//
//  Commands:
//    u  → Read ultrasonic once and print raw + filtered distance
//    i  → Read IR sensor state (triggered / clear)
//    m  → Read MPU6050 once (accel, tilt, fall status)
//    g  → Print current GPS fix (lat, lon, satellites, age)
//    s  → Send a test SMS (verifies GSM module is working)
//    f  → Simulate a fall event (triggers the state machine)
//    r  → Reset fall state machine back to IDLE
//    d  → Dump full system status (all sensors + decision state)
//    h  → Print help (list of commands)
//    ?  → Same as h
//
//  Usage: call test_console_init() in setup(), test_console_poll() in loop().
//  The console is passive — it only acts when you type a command.
//  Normal 20 Hz loop operation continues unaffected.
// ============================================================================

/// Call once in setup() after Serial.begin().
void test_console_init();

/// Call each loop iteration. Checks for serial input and executes commands.
/// Non-blocking — returns immediately if no input available.
void test_console_poll();

#endif // TEST_CONSOLE_H
