#include "ElectromagnetTask.h"

// Task handle
TaskHandle_t emagTaskHandle = NULL;

// Control flag
volatile bool emagEnabled = false;

// Initialize electromagnets
void initElectromagnets() {
  // Calculate minimum cycle time based on timing parameters
  uint32_t phase1_time_ms = EMAG_PULSE_COUNT * (EMAG_PULSE_ON_MS + EMAG_PULSE_OFF_MS);
  uint32_t phase2_time_ms = NUM_ELECTROMAGNETS * EMAG_INDIVIDUAL_ON_MS;
  uint32_t phase3_time_ms = EMAG_DEADTIME_MS;
  uint32_t min_cycle_time_ms = phase1_time_ms + phase2_time_ms + phase3_time_ms;
  
  // Calculate maximum achievable frequency
  float max_freq_hz = 1000.0 / min_cycle_time_ms;
  
  // Validate that requested frequency is achievable
  if (EMAG_CYCLE_FREQ_HZ > max_freq_hz) {
    Serial.println("====================================");
    Serial.println("WARNING: ELECTROMAGNET FREQUENCY TOO HIGH!");
    Serial.printf("Requested: %.2f Hz, Maximum: %.2f Hz\n", EMAG_CYCLE_FREQ_HZ, max_freq_hz);
    Serial.println("Electromagnet task will be DISABLED.");
    Serial.println("====================================");
    
    // Disable electromagnet cycling due to invalid configuration
    emagEnabled = false;
  }
  
  // Configure all electromagnet pins as outputs
  for (int i = 0; i < NUM_ELECTROMAGNETS; i++) {
    pinMode(EMAG_PINS[i], OUTPUT);
    digitalWrite(EMAG_PINS[i], LOW);  // Start with all off
  }
  
  Serial.println("Electromagnets initialized");
}

// Set all electromagnets to a state
void setAllElectromagnets(bool state) {
  for (int i = 0; i < NUM_ELECTROMAGNETS; i++) {
    digitalWrite(EMAG_PINS[i], state ? HIGH : LOW);
  }
}

// FreeRTOS electromagnet task
void electromagnetTask(void *parameter) {
  Serial.println("Electromagnet Task started");
  
  // Calculate actual cycle time and additional delay needed to match frequency
  uint32_t phase1_time_ms = EMAG_PULSE_COUNT * (EMAG_PULSE_ON_MS + EMAG_PULSE_OFF_MS);
  uint32_t phase2_time_ms = NUM_ELECTROMAGNETS * EMAG_INDIVIDUAL_ON_MS;
  uint32_t phase3_time_ms = EMAG_DEADTIME_MS;
  uint32_t pattern_time_ms = phase1_time_ms + phase2_time_ms + phase3_time_ms;
  
  // Calculate target cycle time from frequency
  uint32_t target_cycle_ms = (uint32_t)(1000.0 / EMAG_CYCLE_FREQ_HZ);
  
  // Additional delay needed to achieve target frequency
  uint32_t additional_delay_ms = 0;
  if (target_cycle_ms > pattern_time_ms) {
    additional_delay_ms = target_cycle_ms - pattern_time_ms;
  }
  
  Serial.printf("Pattern execution time: %d ms\n", pattern_time_ms);
  Serial.printf("Target cycle time: %d ms (%.2f Hz)\n", target_cycle_ms, EMAG_CYCLE_FREQ_HZ);
  Serial.printf("Additional delay per cycle: %d ms\n", additional_delay_ms);
  
  while (1) {
    // Check if electromagnet cycling is enabled
    if (emagEnabled) {
      uint32_t cycleStart = millis();
      
      // Phase 1: Pulse all electromagnets 3 times
      for (int pulse = 0; pulse < EMAG_PULSE_COUNT; pulse++) {
        setAllElectromagnets(true);
        vTaskDelay(pdMS_TO_TICKS(EMAG_PULSE_ON_MS));
        
        setAllElectromagnets(false);
        vTaskDelay(pdMS_TO_TICKS(EMAG_PULSE_OFF_MS));
      }
      
      // Phase 2: Turn on each electromagnet individually in order
      for (int i = 0; i < NUM_ELECTROMAGNETS; i++) {
        uint8_t pin = EMAG_PINS[i];
        digitalWrite(pin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(EMAG_INDIVIDUAL_ON_MS));
        digitalWrite(pin, LOW);
      }
      
      // Phase 3: Deadtime - all off
      setAllElectromagnets(false);
      vTaskDelay(pdMS_TO_TICKS(EMAG_DEADTIME_MS));
      
      // Add additional delay to match target frequency
      if (additional_delay_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(additional_delay_ms));
      }
      
    } else {
      // If disabled, ensure all are off and wait
      setAllElectromagnets(false);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

// Enable/disable electromagnet cycling
void setElectromagnetEnabled(bool enabled) {
  emagEnabled = enabled;
  if (!enabled) {
    // Immediately turn off all electromagnets
    setAllElectromagnets(false);
  }
  Serial.printf("Electromagnet cycling %s\n", enabled ? "ENABLED" : "DISABLED");
}

// Get current state
bool getElectromagnetEnabled() {
  return emagEnabled;
}
