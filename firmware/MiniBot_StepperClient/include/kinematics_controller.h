#ifndef __KINEMATICS_CONTROLLER_H__
#define __KINEMATICS_CONTROLLER_H__

#include "robot.h"
#include "motion_queue.h"

/**
 * Kinematics Controller Task
 * 
 * Priority: HIGHEST
 * Responsible for:
 * - Consuming motion commands from the motion queue
 * - Computing inverse kinematics to convert Cartesian targets to motor commands
 * - Controlling stepper motors to reach target positions
 * - Executing smooth motion trajectories
 * 
 * @param pvParameters Pointer to initialization parameters (unused)
 */
void KinematicsController_Task(void* pvParameters);

/**
 * Initialize the kinematics controller
 * Should be called before starting the task
 * 
 * @param motion_queue MotionQueue handle
 * @return true on success, false on failure
 */
bool KinematicsController_Init(MotionQueue motion_queue);

#endif // __KINEMATICS_CONTROLLER_H__
