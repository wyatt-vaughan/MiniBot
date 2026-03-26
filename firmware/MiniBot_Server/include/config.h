#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Enable Joystick Mode (disable GUI mode when enabled)
#define ENABLE_JOYSTICK_MODE 1

// Electromagnet Configuration
#define NUM_ELECTROMAGNETS 6

// Cycle frequency (Hz) - Target frequency for the pattern repetition
#define EMAG_CYCLE_FREQ_HZ 10.0

// GPIO pins for electromagnets
const uint8_t EMAG_PINS[NUM_ELECTROMAGNETS] = {
  32,
  33,
  25,
  26,
  27,
  14,
};

// Timing parameters (milliseconds)
#define EMAG_PULSE_COUNT 3
#define EMAG_PULSE_ON_MS 3
#define EMAG_PULSE_OFF_MS 3
#define EMAG_INDIVIDUAL_ON_MS 5
#define EMAG_DEADTIME_MS 10

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
