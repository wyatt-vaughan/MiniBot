#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

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

#endif // CONFIG_H
