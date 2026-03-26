#ifndef __POSITION_ESTIMATOR_H__
#define __POSITION_ESTIMATOR_H__

#include "robot.h"

/**
 * Position Estimator — Sensor Task
 *
 * Priority: HIGH (3)
 * Core: 1
 * Responsible for:
 * - Communicating with the MMC5633 magnetometer at 2 kHz
 * - Detecting the frame start signal (3 short pulses)
 * - Using timing after the start signal to associate each sample
 *   with the correct electromagnet slot
 * - Posting a complete EmagFrameData to the internal queue after
 *   all 6 electromagnet slots have been sampled
 *
 * @param pvParameters Unused (pass NULL or Robot*)
 */
void PositionEstimator_SensorTask(void* pvParameters);

/**
 * Position Estimator — Calculation Task
 *
 * Priority: MEDIUM (2)
 * Core: 1
 * Responsible for:
 * - Blocking on the internal emag frame queue
 * - Computing robot position
 * - Updating the robot's true pose via robot->setTruePose()
 *
 * @param pvParameters Pointer to Robot instance
 */
void PositionEstimator_CalcTask(void* pvParameters);

/**
 * Initialize the position estimator
 * Should be called before starting the task
 * 
 * @return true on success, false on failure
 */
bool PositionEstimator_Init(void);

/**
 * Get the latest raw magnetometer field readings
 * Returns the averaged magnetic field values from the magnetometer
 * 
 * @param x Pointer to store field_x value in Gauss
 * @param y Pointer to store field_y value in Gauss
 * @param z Pointer to store field_z value in Gauss
 * @return true on success, false if magnetometer not initialized
 */
bool PositionEstimator_GetLatestMagneticField(float* x, float* y, float* z);

#endif // __POSITION_ESTIMATOR_H__
