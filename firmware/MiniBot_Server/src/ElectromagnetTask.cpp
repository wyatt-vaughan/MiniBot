#include "ElectromagnetTask.h"
#include <esp_timer.h>

// Task handle
TaskHandle_t emagTaskHandle = NULL;

// Control flags
volatile bool emagEnabled = false;
volatile bool syncPulseRequested = false;
volatile int64_t nextFrameStartUs = 0;

// Initialize electromagnets
void initElectromagnets() {
  if (EMAG_COUNT * (EMAG_FWD_ON_TIME_MS + EMAG_REV_ON_TIME_MS) > EMAG_FRAME_LEN_MS) {
    DEBUG_PRINTLN("WARNING: Electromagnet on-time exceeds frame length! Adjust timing parameters.");
  }
  
  // Configure all electromagnet pins as outputs, start disabled
  for (int i = 0; i < EMAG_COUNT; i++) {
    pinMode(EMAG_PINS_A[i], OUTPUT);
    pinMode(EMAG_PINS_B[i], OUTPUT);
    digitalWrite(EMAG_PINS_A[i], LOW);
    digitalWrite(EMAG_PINS_B[i], LOW);
  }
  
  DEBUG_PRINTLN("Electromagnets initialized");
}

bool setElectromagnet(uint8_t emag_i, bool enabled, bool forward) {
  if (emag_i >= EMAG_COUNT) {
    DEBUG_PRINTF("Invalid electromagnet index: %d\n", emag_i);
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
  for (int i = 0; i < EMAG_COUNT; i++) {
    setElectromagnet(i, enabled, forward);
  }
}

// Enable/disable electromagnet cycling
void setElectromagnetEnabled(bool enabled) {
  emagEnabled = enabled;
  if (!enabled) {
    setAllElectromagnets(false);
  }
  DEBUG_PRINTF("Electromagnet cycling %s\n",
                enabled ? "ENABLED" : "DISABLED");
}

// Get current state
bool getElectromagnetEnabled() {
  return emagEnabled;
}

// Request a one-shot 3ms sync pulse at the start of the next emag frame
void triggerSyncPulse() {
  syncPulseRequested = true;
  DEBUG_PRINTLN("Sync pulse requested");
}

// Returns microseconds until the start of the next emag frame
uint32_t getTimeToNextFrameUs() {
  const int64_t frameLenUs = (int64_t)EMAG_FRAME_LEN_MS * 1000LL;
  int64_t timeToNext = nextFrameStartUs - esp_timer_get_time();
  if (timeToNext <= 0) timeToNext += frameLenUs;
  return (uint32_t)timeToNext;
}

// Wait until an absolute esp_timer_get_time() target (µs).
// Uses vTaskDelay to yield when more than 2ms remain, then busy-waits the tail.
// Returns false if vTaskDelay ran long and targetUs was already passed on wakeup.
static bool waitUntilUs(int64_t targetUs) {
  int64_t sleepMs = (targetUs - esp_timer_get_time()) / 1000LL - 2;
  if (sleepMs > 0) {
    vTaskDelay(pdMS_TO_TICKS((uint32_t)sleepMs));
    if (esp_timer_get_time() >= targetUs) {
      return false;
    }
  }
  while (esp_timer_get_time() < targetUs) {}
  return true;
}

// --- Emag timing benchmark ---
// Steps per frame: EMAG_COUNT * 4  (fwd ON, gap, rev ON, gap  x  each emag)
#define BENCH_STEPS  (EMAG_COUNT * 4)
#define BENCH_FRAMES 10

static int64_t benchMin[BENCH_STEPS];
static int64_t benchMax[BENCH_STEPS];
static int     benchFrameCount = 0;
static bool    benchInitialized = false;

// Record a timestamp (ns relative to frame start) for step index s.
static inline void benchRecord(int s, int64_t frameStartUs) {
  int64_t nowUs = esp_timer_get_time();
  int64_t relUs = nowUs - frameStartUs;
  if (!benchInitialized || relUs < benchMin[s]) benchMin[s] = relUs;
  if (!benchInitialized || relUs > benchMax[s]) benchMax[s] = relUs;
}

static void benchPrintAndReset() {
  DEBUG_PRINTF("\n=== Emag timing benchmark (last %d frames) ===\n", BENCH_FRAMES);
  for (int i = 0; i < EMAG_COUNT; i++) {
    int base = i * 4;
    DEBUG_PRINTF("  emag%d  fwd ON : min=%6lld µs  max=%6lld µs\n", i, benchMin[base+0], benchMax[base+0]);
    DEBUG_PRINTF("  emag%d  gap1   : min=%6lld µs  max=%6lld µs\n", i, benchMin[base+1], benchMax[base+1]);
    DEBUG_PRINTF("  emag%d  rev ON : min=%6lld µs  max=%6lld µs\n", i, benchMin[base+2], benchMax[base+2]);
    DEBUG_PRINTF("  emag%d  gap2   : min=%6lld µs  max=%6lld µs\n", i, benchMin[base+3], benchMax[base+3]);
  }
  DEBUG_PRINTLN("==============================================\n");
  benchFrameCount  = 0;
  benchInitialized = false;
}

// FreeRTOS electromagnet task
void electromagnetTask(void *parameter) {
  DEBUG_PRINTLN("Electromagnet Task started");

  const int64_t frameLenUs = (int64_t)EMAG_FRAME_LEN_MS * 1000LL;
  nextFrameStartUs = esp_timer_get_time();

  while (1) {
    nextFrameStartUs += frameLenUs;
    if (!waitUntilUs(nextFrameStartUs)) {
      DEBUG_PRINTLN("Frame skipped: overrun at frame start");
      continue;
    }

    // --- Frame start ---

    // Sync pulse: fire all emags for 3ms to sync frame start
    if (syncPulseRequested) {
      syncPulseRequested = false;
      setAllElectromagnets(true, true);  // forward pulse for sync
      waitUntilUs(nextFrameStartUs + 3000LL);
      setAllElectromagnets(false);
      DEBUG_PRINTLN("Sync pulse fired (3ms)");
    }
    else if (emagEnabled) {
      const int64_t fwdOnUs = (int64_t)EMAG_FWD_ON_TIME_MS * 1000LL;
      const int64_t revOnUs = (int64_t)EMAG_REV_ON_TIME_MS * 1000LL;

      int64_t interFrameTimeUs = nextFrameStartUs;

      for (int i = 0; i < EMAG_COUNT && emagEnabled; i++) {
        int base = i * 2;

        // Forward ON
        benchRecord(base + 0, nextFrameStartUs);
        setElectromagnet(i, true, true);
        interFrameTimeUs += fwdOnUs;
        waitUntilUs(interFrameTimeUs);

        // Reverse ON
        benchRecord(base + 1, nextFrameStartUs);
        setElectromagnet(i, true, false);
        interFrameTimeUs += revOnUs;
        waitUntilUs(interFrameTimeUs);

        // OFF
        setElectromagnet(i, false);
      }

      benchInitialized = true;
      benchFrameCount++;
      if (benchFrameCount >= BENCH_FRAMES) {
        benchPrintAndReset();
      }
    }

    // Ensure all emags are off at the end of the frame
    setAllElectromagnets(false);
  }
}

