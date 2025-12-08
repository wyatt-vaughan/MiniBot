#include "kinematics_controller.h"

// Static variables for task state
static MotionQueue* kinematics_queue = NULL;

bool KinematicsController_Init(MotionQueue* motion_queue) {
    if (motion_queue == NULL) {
        return false;
    }
    
    kinematics_queue = motion_queue;
    
    return true;
}

void KinematicsController_Task(void* pvParameters) {
    // Extract robot pointer from task parameters
    Robot* robot = (Robot*)pvParameters;
    
    // Task initialization
    if (robot == NULL || kinematics_queue == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
    MotionCommand current_command;
    
    while (1) {
        // Check if there are commands in the queue
        if (!MotionQueue_IsEmpty(kinematics_queue)) {
            // Dequeue next motion command
            Serial.println("Kinematics controller task = MESSAGE IS AVAILABLE");
            if (MotionQueue_Dequeue(kinematics_queue, &current_command)) {
                Serial.println("Successfully dequeued motion command");
                robot->setTargetPose(current_command);
            }
            else {
                Serial.println("Failed to dequeue motion command");
                continue;
            }
        }
        
        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
