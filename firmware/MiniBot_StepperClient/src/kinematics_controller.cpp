#include "kinematics_controller.h"
#include "esp_now_communicator.h"
#include "motor_test_queue.h"
#include "config.h"
#include <Arduino.h>

static MotionQueue kinematics_queue = NULL;
static MotorTestQueue motor_test_queue = NULL;

// Motor test state tracking
static float current_m0_velocity_rad_s = 0.0f;
static float current_m1_velocity_rad_s = 0.0f;
static uint32_t last_motor_test_command_us = 0;
static bool motor_test_active = false;

bool KinematicsController_Init(MotionQueue motion_queue, MotorTestQueue motor_test_q) {
    if (motion_queue == NULL || motor_test_q == NULL) {
        return false;
    }
    
    kinematics_queue = motion_queue;
    motor_test_queue = motor_test_q;
    return true;
}

void KinematicsController_Task(void* pvParameters) {
    Robot* robot = (Robot*)pvParameters;
    
    if (robot == NULL || kinematics_queue == NULL || motor_test_queue == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
    MotionCommand cmd_buffer;
    MotorTestRequest motor_test_buffer;

    // debug cmds
    MotionCommand cmd0 = {175.0f, 175.0f, 0.0f,   700.0f};
    MotionCommand cmd1 = {225.0f, 175.0f, 0.0f,   1000.0f};
    MotionCommand cmd2 = {225.0f, 175.0f, 1.571f, 700.0f};
    MotionCommand cmd3 = {225.0f, 225.0f, 1.571f, 1000.0f};
    MotionCommand cmd4 = {225.0f, 225.0f, 0.0f,   700.0f};
    MotionCommand cmd5 = {175.0f, 225.0f, 0.0f,   1000.0f};
    MotionCommand cmd6 = {175.0f, 225.0f, 1.571f, 700.0f};
    MotionCommand cmd7 = {175.0f, 175.0f, 1.571f, 1000.0f};

    // debug loop
    // vTaskDelay(pdMS_TO_TICKS(3000));
    // while (1) {
    //     robot->setTargetPose(cmd0);
    //     vTaskDelay(pdMS_TO_TICKS(3000));
    //     robot->setTargetPose(cmd1);
    //     robot->setTargetPose(cmd2);
    //     vTaskDelay(pdMS_TO_TICKS(3000));
    //     robot->setTargetPose(cmd3);
    //     robot->setTargetPose(cmd4);
    //     vTaskDelay(pdMS_TO_TICKS(3000));
    //     robot->setTargetPose(cmd5);
    //     robot->setTargetPose(cmd6);
    //     vTaskDelay(pdMS_TO_TICKS(3000));
    //     robot->setTargetPose(cmd7);
    // }
    

    while (1) {
        uint32_t current_us = micros();

        float true_x, true_y, true_theta;
        bool valid_pose = robot->getTruePose(&true_x, &true_y, &true_theta);
        if (valid_pose) {
            Serial.printf("TRUE POSE: X=%.1f mm\tY=%.1f mm\tθ=%.2f rad\n", true_x, true_y, true_theta);
        }

        if (MotorTestQueue_Dequeue(motor_test_queue, &motor_test_buffer, 10)) {
            last_motor_test_command_us = current_us;
            current_m0_velocity_rad_s = motor_test_buffer.m0_velocity_rad_s;
            current_m1_velocity_rad_s = motor_test_buffer.m1_velocity_rad_s;
            
            Serial.printf("Kinematics: Motor test command - M0=%.2f rad/s, M1=%.2f rad/s\n",
                        current_m0_velocity_rad_s, current_m1_velocity_rad_s);
            
            if (current_m0_velocity_rad_s == 0.0f && current_m1_velocity_rad_s == 0.0f) {
                motor_test_active = false;
                robot->stopMotorTest();
            } else {
                motor_test_active = true;
                robot->setMotorTestVelocity(current_m0_velocity_rad_s, current_m1_velocity_rad_s);
            }
        }
        
        // Check for motor test timeout
        if (motor_test_active && (current_us - last_motor_test_command_us) > (MOTOR_TEST_TIMEOUT_MS * 1000)) {
            Serial.println("Motor test timeout - stopping motors");
            motor_test_active = false;
            robot->stopMotorTest();
        }
        
        // Block waiting for motion command with 100ms timeout
        if (MotionQueue_Dequeue(kinematics_queue, &cmd_buffer, 100)) {
            Serial.println("Kinematics controller: Received motion command");

            // Stop motor test if motion command received
            if (motor_test_active) {
                motor_test_active = false;
                robot->stopMotorTest();
            }
            
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

        // yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
