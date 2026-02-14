#ifndef ELECTROMAGNET_TASK_H
#define ELECTROMAGNET_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "config.h"

// Task handle
extern TaskHandle_t emagTaskHandle;

// Control flag
extern volatile bool emagEnabled;

// Initialize electromagnets
void initElectromagnets();

// FreeRTOS electromagnet task
void electromagnetTask(void *parameter);

// Enable/disable electromagnet cycling
void setElectromagnetEnabled(bool enabled);

// Get current state
bool getElectromagnetEnabled();

#endif // ELECTROMAGNET_TASK_H
