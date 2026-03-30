#ifndef __CONFIG_H__
#define __CONFIG_H__

// ============================================================================
// GPIO Pin Definitions
// ============================================================================

// Left wheel stepper motor
#define L_WHEEL_STEP_PIN     10
#define L_WHEEL_DIR_PIN      4

// Right wheel stepper motor
#define R_WHEEL_STEP_PIN     6
#define R_WHEEL_DIR_PIN      5

// Stepper driver enable (shared by both motors)
#define STEPPER_EN_PIN       7
#define STEPPER_RST_PIN      3

// Battery monitoring
#define BATTERY_VOLTAGE_PIN  1
#define BATTERY_VOLTAGE_DIVIDER_RATIO 1.84f
#define BATTERY_CRITICAL_VOLTAGE 3.2f
#define BATTERY_POLL_INTERVAL_MS 500
#define BATTERY_AVG_WINDOW_SIZE 20

// I2C pins (used for mag sensor and maybe future stuff)
#define SDA_PIN              RX
#define SCL_PIN              TX

// ============================================================================
// Robot Physical Configuration
// ============================================================================

// Wheel dimensions
#define WHEEL_RADIUS_MM      5.25f      // Wheel radius in millimeters
#define WHEEL_SPACING_MM     23.4f      // Distance between wheel centers

// Stepper motor configuration
#define STEPS_PER_REVOLUTION 160.0f     // Total microsteps per revolution. Motors are 20 full steps/rev

// Microstepping config
    // STSPIN220 microstepping table (MS1=HIGH, MS2=LOW hardwired):
    // STEP=0 DIR=0 -> 1/128, STEP=0 DIR=1 -> 1/256
    // STEP=1 DIR=0 -> 1/2, STEP=1 DIR=1 -> 1/8
#define MSET_STEP_LVL      true
#define MSET_DIR_LVL       true

// Motor reversal (set to true to reverse motor direction)
#define L_WHEEL_REVERSE      false
#define R_WHEEL_REVERSE      true

// ============================================================================
// Motion Control Limits
// ============================================================================

// Linear motion constraints
#define ROBOT_MAX_VELOCITY_MM_S    200.0f   // Maximum linear velocity (mm/s) NOT USED
#define ROBOT_MAX_ACCEL_MM_S2      200.0f    // Maximum linear acceleration (mm/s²)

// Rotational motion constraints
#define MAX_ROT_VEL_RAD_S          2.0f     // Maximum angular velocity (rad/s)
#define MAX_ROT_ACCEL_RAD_S2       5.0f    // Maximum angular acceleration (rad/s²)

// Stepper motor safety limit
#define STEPPER_MAX_VELOCITY_MM_S  250.0f   // Stepper motor maximum velocity

// Motor test command configuration
#define MOTOR_TEST_TIMEOUT_MS      1000     // Timeout for motor test commands (ms)
#define MOTOR_TEST_ACCEL_RAD_S2    MAX_ROT_ACCEL_RAD_S2  // Motor test acceleration limit

#define POSITION_TOLERANCE_MM 2.0f          // Position error tolerance
#define ANGLE_TOLERANCE_RAD 0.05f           // ~3 degrees angle tolerance
#define MIN_ARC_RADIUS_MM 5.0f              // Minimum arc radius before fallback

// ============================================================================
// Motion Queue Configuration
// ============================================================================

#define MOTION_QUEUE_SIZE          8       // Maximum pending motion commands

// ============================================================================
// ESP-NOW Network Configuration
// ============================================================================

// Device ID functions are in device_id.h
#define WIFI_CHANNEL               6        // WiFi channel for ESP-NOW
#define WIFI_POWER                 WIFI_POWER_8_5dBm

// ============================================================================
// Task Configuration
// ============================================================================

// FreeRTOS Core1 Config
#define KINEMATICS_TASK_PRIORITY   5        // High priority for motion control

// FreeRTOS Core0 Config
#define BATTERY_MONITOR_PRIORITY    2       // Low priority
#define LED_STATUS_PRIORITY         2       // Low priority
#define POSITION_ESTIMATOR_PRIORITY 3       // Medium priority
#define ESP_NOW_COMM_PRIORITY       4       // High priority

// ============================================================================
// Debug Configuration
// ============================================================================

#define MOTION_DEBUG_LOGGING        true    // Enable detailed motion control debug output

// ============================================================================
// Electromagnet Positioning System Configuration
// ============================================================================

// Electromagnet positions (x, y) in mm relative to the platform origin
// Add one entry per electromagnet; array length must match EMAG_COUNT
#define EMAG_POSITIONS_MM            { {0.0f, 0.0f}, {125.0f, 0.0f} }

// Electromagnet frame timing setup
#define EMAG_FRAME_LEN_MS            100    // Total frame length
#define EMAG_COUNT                   2      // Number of electromagnets in platform
#define EMAG_FWD_ON_TIME_MS          8      // How long forward power is applied
#define EMAG_REV_ON_TIME_MS          8      // How long reverse power is applied
#define EMAG_GAP_TIME_MS             1      // Time between changing electromagnet states
#define EMAG_TRIM_MS                 2      // Samples closer than this to state changes are ignored

// Sampling
#define EMAG_MIN_SAMPLE_PERIOD_US    900   // 1 kHz sampling period, set faster to avoid missed readings (µs)
#define MAX_SAMPLES_PER_EMAG         ((EMAG_FWD_ON_TIME_MS + EMAG_REV_ON_TIME_MS) * 1000 / EMAG_MIN_SAMPLE_PERIOD_US)

// Detection thresholds
#define FIELD_THRESHOLD_GAUSS        0.5f   // Minimum field magnitude to detect emag active

// Other stuff :)
#define TRUE_POSE_LPF_CUTOFF_HZ      1.0f
#define TRUE_POSE_STALE_TIMEOUT_MS   1000   // Max age of true pose before it is considered stale
#define EMAG_MIN_SIGNAL_GAUSS        0.5f   // Minimum fwd-rev differential magnitude to consider an emag reading valid

#endif // __CONFIG_H__
