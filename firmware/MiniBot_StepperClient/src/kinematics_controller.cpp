#include "kinematics_controller.h"
#include "esp_now_communicator.h"
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

    // debug cmds
    MotionCommand cmd0 = {
                100.0f,
                0.0f,
                0.0f,
                1500.0f
    };
    MotionCommand cmd1 = {
                100.0f,
                0.0f,
                1.57f,
                1000.0f
    };
    MotionCommand cmd2 = {
                0.0f,
                0.0f,
                -1.57f,
                1500.0f
    };
    MotionCommand cmd3 = {
                0.0f,
                0.0f,
                0.0f,
                1000.0f
    };
    // debug loop
    vTaskDelay(pdMS_TO_TICKS(3000));
    while (1) {
        robot->setTargetPose(cmd0);
        robot->setTargetPose(cmd1);
        robot->setTargetPose(cmd2);
        robot->setTargetPose(cmd3);
        vTaskDelay(pdMS_TO_TICKS(8000));
    }
    

    while (1) {
        // Block waiting for a command with 100ms timeout
        if (MotionQueue_Dequeue(kinematics_queue, &cmd_buffer, 100)) {
            Serial.println("Kinematics controller: Received motion command");

            // tbd if I want to move this before the dequeue or not
            if (robot->getBatteryCritical()) {
                Serial.println("WARNING: Battery voltage critical, ignoring motion command");
                if (!EspNowCommunicator_SendAlert(ERR_LOW_BATTERY)) {
                    Serial.println("Failed to send low battery alert");
                }
                continue;
            }
            robot->setTargetPose(cmd_buffer);
        }
    }
}
