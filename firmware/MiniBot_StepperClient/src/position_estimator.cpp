#include "position_estimator.h"

bool PositionEstimator_Init(void) {
    return true;
}

void PositionEstimator_Task(void* pvParameters) {
    // Extract robot pointer from task parameters
    Robot* robot = (Robot*)pvParameters;
    
    // Task initialization
    if (robot == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
    int32_t prev_left_steps = 0;
    int32_t prev_right_steps = 0;
    
    while (1) {
        // Read current step counts from stepper drivers
        int32_t left_steps = robot->left_wheel.current_step_count;
        int32_t right_steps = robot->right_wheel.current_step_count;
        
        // Calculate step deltas since last update
        int32_t left_delta = left_steps - prev_left_steps;
        int32_t right_delta = right_steps - prev_right_steps;
        
        // TODO: Implement odometry calculations
        // Convert step deltas to distance traveled on each wheel
        // Calculate X, Y position change and orientation change
        // Update robot position
        
        // Example placeholder:
        // float left_distance = left_delta * STEPS_TO_MM;
        // float right_distance = right_delta * STEPS_TO_MM;
        // float robot_center_distance = (left_distance + right_distance) / 2.0f;
        // float robot_angle_change = (right_distance - left_distance) / WHEEL_DISTANCE;
        robot->updatePosition();
        
        prev_left_steps = left_steps;
        prev_right_steps = right_steps;
        
        // Update at 10 Hz
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
