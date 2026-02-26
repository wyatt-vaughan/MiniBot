#ifndef __POSITION_ESTIMATOR_H__
#define __POSITION_ESTIMATOR_H__

#include "robot.h"

/**
 * Position Estimator Task
 * 
 * Priority: MEDIUM
 * Responsible for:
 * - Estimating robot position based on wheel odometry
 * - Tracking step counts from stepper motors
 * - Continuously updating the robot's "true" position estimate
 * - Potentially fusing IMU or other sensor data (future enhancement)
 * 
 * Note: The position estimator always runs periodically and updates
 * the robot's true_x, true_y, and true_theta values. Other tasks can
 * call robot->updatePositionFromEstimate() to copy these values to
 * the active position when needed.
 * 
 * @param pvParameters Pointer to Robot instance
 */
void PositionEstimator_Task(void* pvParameters);

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
