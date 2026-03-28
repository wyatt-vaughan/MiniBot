#include "ElectromagnetTask.h"
#include <esp_timer.h>

// Task handle
TaskHandle_t emagTaskHandle = NULL;

// Control flags
volatile bool emagEnabled = false;
volatile bool syncPulseRequested = false;

// Initialize electromagnets
void initElectromagnets() {
  if (EMAG_COUNT * (EMAG_FWD_ON_TIME_MS + EMAG_REV_ON_TIME_MS + (2 * EMAG_GAP_TIME_MS)) > EMAG_FRAME_LEN_MS) {
    Serial.println("WARNING: Electromagnet on-time exceeds frame length! Adjust timing parameters.");
  }
  
  // Configure all electromagnet pins as outputs, start disabled
  for (int i = 0; i < NUM_ELECTROMAGNETS; i++) {
    pinMode(EMAG_PINS_A[i], OUTPUT);
    pinMode(EMAG_PINS_B[i], OUTPUT);
    digitalWrite(EMAG_PINS_A[i], LOW);
    digitalWrite(EMAG_PINS_B[i], LOW);
  }
  
  Serial.println("Electromagnets initialized");
}

bool setElectromagnet(uint8_t emag_i, bool enabled, bool forward) {
  if (emag_i >= NUM_ELECTROMAGNETS) {
    Serial.printf("Invalid electromagnet index: %d\n", emag_i);
    return false;
  }
  
  if (!enabled) {
    digitalWrite(EMAG_PINS_A[emag_i], LOW);
    digitalWrite(EMAG_PINS_B[emag_i], LOW);
  } else if (forward) {
    digitalWrite(EMAG_PINS_A[emag_i], HIGH);
    digitalWrite(EMAG_PINS_B[emag_i], LOW);
  } else {
    digitalWrite(EMAG_PINS_A[emag_i], LOW);
    digitalWrite(EMAG_PINS_B[emag_i], HIGH);
  }
  return true;
}

// Set all electromagnets: disabled=[0,0], forward=[1,0], reverse=[0,1]
void setAllElectromagnets(bool enabled, bool forward) {
  for (int i = 0; i < NUM_ELECTROMAGNETS; i++) {
    setElectromagnet(i, enabled, forward);
  }
}

// Enable/disable electromagnet cycling
void setElectromagnetEnabled(bool enabled) {
  emagEnabled = enabled;
  if (!enabled) {
    setAllElectromagnets(false);
  }
  Serial.printf("Electromagnet cycling %s\n",
                enabled ? "ENABLED" : "DISABLED");
}

// Get current state
bool getElectromagnetEnabled() {
  return emagEnabled;
}

// Request a one-shot 3ms sync pulse at the start of the next emag frame
void triggerSyncPulse() {
  syncPulseRequested = true;
  Serial.println("Sync pulse requested");
}

// Wait until an absolute esp_timer_get_time() target (µs).
// Uses vTaskDelay to yield when more than 2ms remain, then busy-waits the tail.
static void waitUntilUs(int64_t targetUs) {
  int64_t sleepMs = (targetUs - esp_timer_get_time()) / 1000LL - 2;
  if (sleepMs > 0) {
    vTaskDelay(pdMS_TO_TICKS((uint32_t)sleepMs));
  }
  while (esp_timer_get_time() < targetUs) {}
}

// FreeRTOS electromagnet task
void electromagnetTask(void *parameter) {
  Serial.println("Electromagnet Task started");

  const int64_t frameLenUs = (int64_t)EMAG_FRAME_LEN_MS * 1000LL;
  int64_t nextFrameStartUs = esp_timer_get_time();

  while (1) {
    nextFrameStartUs += frameLenUs;
    waitUntilUs(nextFrameStartUs);

    // --- Frame start ---

    // Sync pulse: fire all emags for 3ms to sync frame start
    if (syncPulseRequested) {
      syncPulseRequested = false;
      setAllElectromagnets(true, true);  // forward pulse for sync
      waitUntilUs(nextFrameStartUs + 3000LL);
      setAllElectromagnets(false);
      Serial.println("Sync pulse fired (3ms)");
    }
    else if (emagEnabled) {
      const int64_t fwdOnUs = (int64_t)EMAG_FWD_ON_TIME_MS * 1000LL;
      const int64_t revOnUs = (int64_t)EMAG_REV_ON_TIME_MS * 1000LL;
      const int64_t gapUs   = (int64_t)EMAG_GAP_TIME_MS    * 1000LL;

      int64_t interFrameTimeUs = nextFrameStartUs;

      for (int i = 0; i < EMAG_COUNT && emagEnabled; i++) {
        // Forward ON
        setElectromagnet(i, true, true);
        interFrameTimeUs += fwdOnUs;
        waitUntilUs(interFrameTimeUs);

        // Wait
        setElectromagnet(i, false);
        interFrameTimeUs += gapUs;
        waitUntilUs(interFrameTimeUs);

        // Reverse ON
        setElectromagnet(i, true, false);
        interFrameTimeUs += revOnUs;
        waitUntilUs(interFrameTimeUs);

        // Wait
        setElectromagnet(i, false);
        interFrameTimeUs += gapUs;
        waitUntilUs(interFrameTimeUs);
      }
    }

    // Ensure all emags are off at the end of the frame
    setAllElectromagnets(false);
  }
}

