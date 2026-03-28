#ifndef ELECTROMAGNET_TASK_H
#define ELECTROMAGNET_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"

// Task handle
extern TaskHandle_t emagTaskHandle;

// Control flags
extern volatile bool emagEnabled;
extern volatile bool syncPulseRequested;

// Initialize electromagnets
void initElectromagnets();

// FreeRTOS electromagnet task
void electromagnetTask(void *parameter);

// Control single electromagnet
bool setElectromagnet(uint8_t emag_i, bool enabled, bool forward = true);

// Control all electromagnets
void setAllElectromagnets(bool enabled, bool forward = true);

// Enable/disable electromagnet position cycle
void setElectromagnetEnabled(bool enabled);

// Get current state
bool getElectromagnetEnabled();

// Request a one-shot 3ms sync pulse at the start of the next emag frame
void triggerSyncPulse();

#endif // ELECTROMAGNET_TASK_H
