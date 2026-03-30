/*
A FreeRTOS powered controller for a tiny 2 wheel robot.
*/

#include <Arduino.h>
#include "robot.h"
#include "motion_queue.h"
#include "motor_test_queue.h"
#include "kinematics_controller.h"
#include "esp_now_communicator.h"
#include "position_estimator.h"
#include "battery_monitor.h"
#include "led_status.h"
#include "device_id.h"

static Robot robot;
static MotionQueue motion_queue = NULL;
static MotorTestQueue motor_test_queue = NULL;

static TaskHandle_t kinematics_task_handle = NULL;
static TaskHandle_t communicator_task_handle = NULL;
static TaskHandle_t position_estimator_sensor_task_handle = NULL;
static TaskHandle_t position_estimator_calc_task_handle = NULL;
static TaskHandle_t battery_monitor_task_handle = NULL;
static TaskHandle_t led_status_task_handle = NULL;

static bool create_tasks(void) {
    BaseType_t task_created;
    
    task_created = xTaskCreatePinnedToCore(
        KinematicsController_Task,
        "KinematicsController",
        8192,
        (void*)&robot,
        4,
        &kinematics_task_handle,
        1
    );
    if (task_created != pdPASS) {
        Serial.println("ERROR: Failed to create Kinematics Controller task");
        return false;
    }
    Serial.println("Kinematics Controller task created successfully");
    
    task_created = xTaskCreatePinnedToCore(
        EspNowCommunicator_Task,
        "EspNowCommunicator",
        2048,
        (void*)&robot,
        2,
        &communicator_task_handle,
        0
    );
    if (task_created != pdPASS) {
        Serial.println("ERROR: Failed to create ESP-NOW Communicator task");
        return false;
    }
    Serial.println("ESP-NOW Communicator task created successfully");
    
    task_created = xTaskCreatePinnedToCore(
        PositionEstimator_SensorTask,
        "PositionEstimatorSensor",
        4096,
        (void*)&robot,
        3,
        &position_estimator_sensor_task_handle,
        0
    );
    if (task_created != pdPASS) {
        Serial.println("ERROR: Failed to create Position Estimator Sensor task");
        return false;
    }
    Serial.println("Position Estimator Sensor task created successfully");

    task_created = xTaskCreatePinnedToCore(
        PositionEstimator_CalcTask,
        "PositionEstimatorCalc",
        4096,
        (void*)&robot,
        2,
        &position_estimator_calc_task_handle,
        0
    );
    if (task_created != pdPASS) {
        Serial.println("ERROR: Failed to create Position Estimator Calc task");
        return false;
    }
    Serial.println("Position Estimator Calc task created successfully");
    
    task_created = xTaskCreatePinnedToCore(
        BatteryMonitor_Task,
        "BatteryMonitor",
        2048,
        (void*)&robot,
        1,
        &battery_monitor_task_handle,
        0
    );
    if (task_created != pdPASS) {
        Serial.println("ERROR: Failed to create Battery Monitor task");
        return false;
    }
    Serial.println("Battery Monitor task created successfully");
    
    // No Status LED on V3 boards, keep for future use
    // task_created = xTaskCreatePinnedToCore(
    //     LedStatus_Task,
    //     "LedStatus",
    //     1024,
    //     (void*)&robot,
    //     0,
    //     &led_status_task_handle,
    //     0
    // );
    // if (task_created != pdPASS) {
    //     Serial.println("ERROR: Failed to create LED Status task");
    //     return false;
    // }
    // Serial.println("LED Status task created successfully");
    
    return true;
}

static bool initialize_modules(void) {
    motion_queue = MotionQueue_Create(MOTION_QUEUE_SIZE);
    if (motion_queue == NULL) {
        Serial.println("ERROR: Failed to create motion queue");
        return false;
    }
    Serial.println("Motion queue created successfully");
    
    motor_test_queue = MotorTestQueue_Create(MOTION_QUEUE_SIZE);
    if (motor_test_queue == NULL) {
        Serial.println("ERROR: Failed to create motor test queue");
        return false;
    }
    Serial.println("Motor test queue created successfully");
    
    if (!robot.initialize()) {
        Serial.println("ERROR: Failed to initialize robot");
        return false;
    }
    Serial.println("Robot initialized successfully");
    
    if (!KinematicsController_Init(motion_queue, motor_test_queue)) {
        Serial.println("ERROR: Failed to initialize kinematics controller");
        return false;
    }
    Serial.println("Kinematics controller initialized successfully");
    
    if (!EspNowCommunicator_Init(motion_queue, motor_test_queue)) {
        Serial.println("ERROR: Failed to initialize ESP-NOW communicator");
        return false;
    }
    Serial.println("ESP-NOW communicator initialized successfully");
    
    if (!PositionEstimator_Init()) {
        Serial.println("ERROR: Failed to initialize position estimator");
        return false;
    }
    Serial.println("Position estimator initialized successfully");
    
    if (!BatteryMonitor_Init(BATTERY_VOLTAGE_PIN)) {
        Serial.println("ERROR: Failed to initialize battery monitor");
        return false;
    }
    Serial.println("Battery monitor initialized successfully");
    
    // if (!LedStatus_Init(2)) {
    //     Serial.println("ERROR: Failed to initialize LED status");
    //     return false;
    // }
    // Serial.println("LED status initialized successfully");
    
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n\n=== MiniBot Stepper Client ===");

    // Write device ID to NVS, ony do this one time. Persistant across reflashes.
    // TODO create a function to set ID based on current position on chess board
    // setDeviceID(0x04);
    
    uint8_t device_id = getDeviceID();
    if (device_id == 0xFF) {
        Serial.println("WARNING: Device ID not configured!");
        Serial.println("Set ID by calling: setDeviceID(id) where id is 0x01-0xFE");
    } else {
        Serial.printf("Device ID: 0x%02X\n", device_id);
    }
    
    Serial.println("Initializing FreeRTOS framework...");
    
    if (!initialize_modules()) {
        Serial.println("FATAL: Module initialization failed");
        while (1) {
            delay(1000);
        }
    }
    
    Serial.println("Modules initialized successfully");
    Serial.println("\nCreating FreeRTOS tasks...");
    
    if (!create_tasks()) {
        Serial.println("FATAL: Task creation failed");
        while (1) {
            delay(1000);
        }
    }
    
    Serial.println("\nAll tasks created successfully");
    Serial.println("Core 0: ESP-NOW, Position Estimator, Battery Monitor, LED Status");
    Serial.println("Core 1: Kinematics Controller");

    setCpuFrequencyMhz(80);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
