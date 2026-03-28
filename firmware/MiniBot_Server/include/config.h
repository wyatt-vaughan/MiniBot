#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Enable Joystick Mode (disable GUI mode when enabled)
#define ENABLE_JOYSTICK_MODE 0

// Random GPIO
#define STATUS_LED_WHITE_PIN 5
#define STATUS_LED_RED_PIN 4

// Electromagnet Configuration
#define NUM_ELECTROMAGNETS 2

// GPIO pins for electromagnets (A=forward drive, B=reverse drive)
// Forward: [A=HIGH, B=LOW]  Reverse: [A=LOW, B=HIGH]  Off: [A=LOW, B=LOW]
const uint8_t EMAG_PINS_A[NUM_ELECTROMAGNETS] = {
  6,
  8,
};
const uint8_t EMAG_PINS_B[NUM_ELECTROMAGNETS] = {
  7,
  9,
};

// Timing parameters (milliseconds)
#define EMAG_FRAME_LEN_MS            100    // Total frame length
#define EMAG_COUNT                   2      // Number of electromagnets in pattern
#define EMAG_FWD_ON_TIME_MS          6      // How long forward power is applied
#define EMAG_REV_ON_TIME_MS          6      // How long reverse power is applied
#define EMAG_GAP_TIME_MS             1      // Time between changing electromagnet states

// ============= Joystick Configuration =============
#if ENABLE_JOYSTICK_MODE

#define JOYSTICK_TARGET_ID 0xFF  // Broadcast ID for all robots

#define JOYSTICK_THROTTLE_PIN 1
#define JOYSTICK_STEERING_PIN 3

#define JOYSTICK_THROTTLE_CENTER 2190
#define JOYSTICK_STEERING_CENTER 2220

#define JOYSTICK_THROTTLE_INVERT -1
#define JOYSTICK_STEERING_INVERT 1

#define JOYSTICK_DEADZONE 50
#define JOYSTICK_MIN_TICK 50
#define JOYSTICK_MAX_TICK 4050

#define JOYSTICK_MAX_MOTOR_VELOCITY 127   // Max input value (int8)
#define JOYSTICK_STEERING_SCALE 0.8       // sets upper limit for steering velocity as a fraction of max velocity

// Reduces steering authority with higher throttle
#define JOYSTICK_STEERING_INFLUENCE_FACTOR 0.70

#define JOYSTICK_UPDATE_INTERVAL_MS 100
#define JOYSTICK_WATCHDOG_INTERVAL_MS 900 // Ensure the watchdog isn't tripped (1s timeout)

#endif // ENABLE_JOYSTICK_MODE

#endif // CONFIG_H
