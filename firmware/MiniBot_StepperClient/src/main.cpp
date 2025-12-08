/*
A FreeRTOS powered controller for a 2 wheel robot.

Tasks | Descending priority order:
- Kinematics Controller (Priority 4) - highest
- ESP-NOW Communicator (Priority 3)
- Position Estimator (Priority 2)
- Battery Monitor (Priority 1)
- LED Status Indicator (Priority 0) - lowest

More info on each task within their header files.

*/

#include <Arduino.h>

// Headers for all modules
#include "robot.h"
#include "motion_queue.h"
#include "kinematics_controller.h"
#include "esp_now_communicator.h"
#include "position_estimator.h"
#include "battery_monitor.h"
#include "led_status.h"

// Global robot state and motion queue
static Robot robot;
static MotionQueue* motion_queue = NULL;

// Task handles
static TaskHandle_t kinematics_task_handle = NULL;
static TaskHandle_t communicator_task_handle = NULL;
static TaskHandle_t position_estimator_task_handle = NULL;
static TaskHandle_t battery_monitor_task_handle = NULL;
static TaskHandle_t led_status_task_handle = NULL;

/**
 * Create all FreeRTOS tasks with core affinity for dual-core ESP32-C3
 * 
 * Core Assignment:
 * - Core 0: WiFi/Communication tasks (ESP-NOW Communicator)
 *           System tasks (LED, Battery Monitor, Position Estimator)
 * - Core 1: Kinematics Controller (motion execution) - isolated for real-time performance
 */
static bool create_tasks(void) {
    BaseType_t task_created;
    
    // ========================================================================
    // Kinematics Controller - Priority 4 (CORE 1 - Isolated for real-time motion)
    // ========================================================================
    task_created = xTaskCreatePinnedToCore(
        KinematicsController_Task,
        "KinematicsController",
        8192,  // Stack size in words (increased from 2048 to prevent stack overflow)
        (void*)&robot,  // Pass robot pointer
        4,     // Priority (highest)
        &kinematics_task_handle,
        1      // Core 1 (dedicated to kinematics/motion control)
    );
    if (task_created != pdPASS) {
        Serial.println("ERROR: Failed to create Kinematics Controller task");
        return false;
    }
    Serial.println("✓ Kinematics Controller pinned to Core 1");
    
    // ========================================================================
    // ESP-NOW Communicator - Priority 3 (CORE 0 - WiFi core)
    // ========================================================================
    task_created = xTaskCreatePinnedToCore(
        EspNowCommunicator_Task,
        "EspNowCommunicator",
        2048,
        (void*)&robot,  // Pass robot pointer
        3,
        &communicator_task_handle,
        0      // Core 0 (WiFi/communication core)
    );
    if (task_created != pdPASS) {
        Serial.println("ERROR: Failed to create ESP-NOW Communicator task");
        return false;
    }
    Serial.println("✓ ESP-NOW Communicator pinned to Core 0");
    
    // ========================================================================
    // Position Estimator - Priority 2 (CORE 0 - System core)
    // ========================================================================
    task_created = xTaskCreatePinnedToCore(
        PositionEstimator_Task,
        "PositionEstimator",
        2048,
        (void*)&robot,  // Pass robot pointer
        2,
        &position_estimator_task_handle,
        0      // Core 0 (system core)
    );
    if (task_created != pdPASS) {
        Serial.println("ERROR: Failed to create Position Estimator task");
        return false;
    }
    Serial.println("✓ Position Estimator pinned to Core 0");
    
    // ========================================================================
    // Battery Monitor - Priority 1 (CORE 0 - System core)
    // ========================================================================
    task_created = xTaskCreatePinnedToCore(
        BatteryMonitor_Task,
        "BatteryMonitor",
        1024,
        (void*)&robot,  // Pass robot pointer
        1,
        &battery_monitor_task_handle,
        0      // Core 0 (system core)
    );
    if (task_created != pdPASS) {
        Serial.println("ERROR: Failed to create Battery Monitor task");
        return false;
    }
    Serial.println("✓ Battery Monitor pinned to Core 0");
    
    // ========================================================================
    // LED Status Indicator - Priority 0 (CORE 0 - System core, lowest priority)
    // ========================================================================
    task_created = xTaskCreatePinnedToCore(
        LedStatus_Task,
        "LedStatus",
        1024,
        (void*)&robot,  // Pass robot pointer
        0,
        &led_status_task_handle,
        0      // Core 0 (system core)
    );
    if (task_created != pdPASS) {
        Serial.println("ERROR: Failed to create LED Status task");
        return false;
    }
    Serial.println("✓ LED Status Indicator pinned to Core 0");
    
    return true;
}

/**
 * Initialize all modules
 */
static bool initialize_modules(void) {
    // Initialize motion queue
    motion_queue = MotionQueue_Create(50);
    if (motion_queue == NULL) {
        Serial.println("ERROR: Failed to create motion queue");
        return false;
    }
    
    // Initialize robot hardware using class method
    if (!robot.initialize()) {
        Serial.println("ERROR: Failed to initialize robot");
        return false;
    }
    
    // Initialize kinematics controller motion queue
    if (!KinematicsController_Init(motion_queue)) {
        Serial.println("ERROR: Failed to initialize kinematics controller");
        return false;
    }
    
    // Initialize ESP-NOW communicator motion queue
    if (!EspNowCommunicator_Init(motion_queue)) {
        Serial.println("ERROR: Failed to initialize ESP-NOW communicator");
        return false;
    }
    
    // Initialize position estimator
    if (!PositionEstimator_Init()) {
        Serial.println("ERROR: Failed to initialize position estimator");
        return false;
    }
    
    // Initialize battery monitor (ADC pin 0 on ESP32-C3)
    if (!BatteryMonitor_Init(0)) {
        Serial.println("ERROR: Failed to initialize battery monitor");
        return false;
    }
    
    // Initialize LED status (GPIO pin 2)
    if (!LedStatus_Init(2)) {
        Serial.println("ERROR: Failed to initialize LED status");
        return false;
    }
    
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\n\n=== MiniBot Stepper Client ===");
    Serial.println("Initializing FreeRTOS framework...");
    Serial.println("Dual-core configuration: Core 0 (WiFi/System), Core 1 (Kinematics)");
    
    // Initialize all modules
    if (!initialize_modules()) {
        Serial.println("FATAL: Module initialization failed");
        while (1) {
            delay(1000);
        }
    }
    
    Serial.println("Modules initialized successfully");
    Serial.println("\nCreating FreeRTOS tasks with core affinity...");
    
    // Create FreeRTOS tasks with core pinning
    if (!create_tasks()) {
        Serial.println("FATAL: Task creation failed");
        while (1) {
            delay(1000);
        }
    }
    
    Serial.println("\nAll tasks created successfully");
    Serial.println("Task allocation:");
    Serial.println("  Core 0: ESP-NOW, Position Estimator, Battery Monitor, LED Status");
    Serial.println("  Core 1: Kinematics Controller (isolated)");
    Serial.println("\nStarting FreeRTOS scheduler...");
    
    // Note: vTaskStartScheduler() is called automatically by the Arduino framework
    // for ESP32, so the loop() function will not be called once FreeRTOS starts
}

void loop() {
    // Loop is not used when FreeRTOS scheduler is running
    // All application logic is handled by FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}
