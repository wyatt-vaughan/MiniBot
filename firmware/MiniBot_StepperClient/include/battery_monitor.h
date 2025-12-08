#ifndef __BATTERY_MONITOR_H__
#define __BATTERY_MONITOR_H__

#include "robot.h"

/**
 * Battery Monitor Task
 * 
 * Priority: LOW
 * Responsible for:
 * - Periodically reading battery voltage via ADC
 * - Updating robot state with current voltage
 * - Detecting low battery conditions
 * - Triggering alerts when voltage drops below threshold
 * 
 * @param pvParameters Pointer to initialization parameters (unused)
 */
void BatteryMonitor_Task(void* pvParameters);

/**
 * Initialize the battery monitor
 * Should be called before starting the task
 * 
 * @param adc_pin GPIO pin connected to battery voltage divider
 * @return true on success, false on failure
 */
bool BatteryMonitor_Init(uint8_t adc_pin);

/**
 * Set the low battery threshold
 * @param threshold Voltage threshold in volts
 */
void BatteryMonitor_SetLowBatteryThreshold(float threshold);

/**
 * Get the low battery threshold
 * @return Current low battery threshold in volts
 */
float BatteryMonitor_GetLowBatteryThreshold(void);

#endif // __BATTERY_MONITOR_H__
