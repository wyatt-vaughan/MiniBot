#include "kinematics_controller.h"
#include <Arduino.h>

static MotionQueue kinematics_queue = NULL;

bool KinematicsController_Init(MotionQueue motion_queue) {
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
        // Block waiting for a command with 100ms timeout
        if (MotionQueue_Dequeue(kinematics_queue, &cmd_buffer, 100)) {
            Serial.println("Kinematics controller: Received motion command");
            robot->setTargetPose(cmd_buffer);
        }
    }
}
