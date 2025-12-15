#ifndef __CONFIG_H__
#define __CONFIG_H__

// ============================================================================
// GPIO Pin Definitions
// ============================================================================

// Left wheel stepper motor
#define L_WHEEL_STEP_PIN     10
#define L_WHEEL_DIR_PIN      4

// Right wheel stepper motor
#define R_WHEEL_STEP_PIN     5
#define R_WHEEL_DIR_PIN      6

// Stepper driver enable (shared by both motors)
#define STEPPER_EN_PIN       7
#define STEPPER_RST_PIN      3

// Battery monitoring
#define BATTERY_VOLTAGE_PIN  1
#define BATTERY_VOLTAGE_DIVIDER_RATIO 2.0f

// I2C pins (used for mag sensor and maybe future stuff)
#define SDA_PIN              RX
#define SCL_PIN              TX

// ============================================================================
// Robot Physical Configuration
// ============================================================================

// Wheel dimensions
#define WHEEL_RADIUS_MM      6.25f      // Wheel radius in millimeters
#define WHEEL_SPACING_MM     23.3f      // Distance between wheel centers

// Stepper motor configuration
#define STEPS_PER_REVOLUTION 128.0f     // Total microsteps per revolution

// Motor reversal (set to true to reverse motor direction)
#define L_WHEEL_REVERSE      false     // Reverse left wheel motor
#define R_WHEEL_REVERSE      false     // Reverse right wheel motor

// ============================================================================
// Motion Control Limits
// ============================================================================

// Linear motion constraints
#define ROBOT_MAX_VELOCITY_MM_S    100.0f   // Maximum linear velocity (mm/s)
#define ROBOT_MAX_ACCEL_MM_S2      100.0f    // Maximum linear acceleration (mm/s²)

// Rotational motion constraints
#define MAX_ROT_VEL_RAD_S          2.0f     // Maximum angular velocity (rad/s)
#define MAX_ROT_ACCEL_RAD_S2       10.0f    // Maximum angular acceleration (rad/s²)

// Stepper motor safety limit
#define STEPPER_MAX_VELOCITY_MM_S  200.0f   // Stepper motor maximum velocity

// ============================================================================
// Motion Queue Configuration
// ============================================================================

#define MOTION_QUEUE_SIZE          16       // Maximum pending motion commands

// ============================================================================
// ESP-NOW Network Configuration
// ============================================================================

#define DEVICE_ID                  0x22     // Unique ID for this robot
#define WIFI_CHANNEL               6        // WiFi channel for ESP-NOW
#define WIFI_POWER                 WIFI_POWER_11dBm  // WiFi TX power

// ============================================================================
// Task Configuration
// ============================================================================

// FreeRTOS Core1 Config
#define KINEMATICS_TASK_PRIORITY   5        // High priority for motion control
#define KINEMATICS_TASK_STACK_SIZE 8192     // Stack size in words

// FreeRTOS Core0 Config
#define BATTERY_MONITOR_PRIORITY   3        // Medium priority
#define LED_STATUS_PRIORITY        3        // Medium priority
#define POSITION_ESTIMATOR_PRIORITY 3       // Medium priority
#define ESP_NOW_COMM_PRIORITY      4        // Higher priority for comms

#endif // __CONFIG_H__
