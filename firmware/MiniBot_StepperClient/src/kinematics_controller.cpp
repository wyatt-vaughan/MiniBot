#include "kinematics_controller.h"

static MotionQueue* kinematics_queue = NULL;

bool KinematicsController_Init(MotionQueue* motion_queue) {
    if (motion_queue == NULL) {
        return false;
    }
    
    kinematics_queue = motion_queue;
    return true;
}

void KinematicsController_Task(void* pvParameters) {
    Robot* robot = (Robot*)pvParameters;
    
    if (robot == NULL || kinematics_queue == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
    MotionCommand cmd_buffer;
    
    while (1) {
        if (!MotionQueue_IsEmpty(kinematics_queue)) {
            Serial.println("Kinematics controller task = MESSAGE IS AVAILABLE");
            if (MotionQueue_Dequeue(kinematics_queue, &cmd_buffer)) {
                Serial.println("Successfully dequeued motion command");
                robot->setTargetPose(cmd_buffer);
            }
            else {
                Serial.println("Failed to dequeue motion command");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
