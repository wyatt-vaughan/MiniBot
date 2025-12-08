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
 * - Updating robot state with calculated position
 * - Potentially fusing IMU or other sensor data (future enhancement)
 * 
 * @param pvParameters Pointer to initialization parameters (unused)
 */
void PositionEstimator_Task(void* pvParameters);

/**
 * Initialize the position estimator
 * Should be called before starting the task
 * 
 * @return true on success, false on failure
 */
bool PositionEstimator_Init(void);

#endif // __POSITION_ESTIMATOR_H__
