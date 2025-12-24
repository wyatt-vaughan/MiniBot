#include "position_estimator.h"

bool PositionEstimator_Init(void) {
    return true;
}

void PositionEstimator_Task(void* pvParameters) {
    Robot* robot = (Robot*)pvParameters;
    
    if (robot == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
    int32_t prev_left_steps = 0;
    int32_t prev_right_steps = 0;
    
    while (1) {
        int32_t left_steps = robot->left_wheel.current_step_count;
        int32_t right_steps = robot->right_wheel.current_step_count;
        
        int32_t left_delta = left_steps - prev_left_steps;
        int32_t right_delta = right_steps - prev_right_steps;
        
        // TODO: Implement odometry calculations
        robot->updateTruePosition();
        
        prev_left_steps = left_steps;
        prev_right_steps = right_steps;
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
