#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "pins.h"
#include "motion_queue.h"
#include "esp_now_communicator.h"
#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include <functional>

/**
 * Stepper driver for controlling individual stepper motors
 */
class StepperDriver {
public:
    // Pin configuration
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t enable_pin;
    uint8_t reset_pin;
    
    // State
    bool enabled;
    int32_t current_step_count;
    int32_t target_step_count;
    
    /**
     * Initialize stepper driver with GPIO pins
     * @param step GPIO pin for step signal
     * @param dir GPIO pin for direction signal
     * @param enable GPIO pin for enable signal
     * @param reset GPIO pin for reset signal
     * @return true on success, false on failure
     */
    bool initialize(uint8_t step, uint8_t dir, uint8_t enable, uint8_t reset);
    
    /**
     * Enable the stepper driver
     * @return true on success, false on failure
     */
    bool enable();
    
    /**
     * Disable the stepper driver
     * @return true on success, false on failure
     */
    bool disable();
    
    /**
     * Set stepper direction
     * @param direction true for forward, false for reverse
     * @return true on success, false on failure
     */
    bool setDirection(bool direction);
    
    /**
     * Generate a single step pulse
     * @return true on success, false on failure
     */
    bool step();
};

/**
 * Robot state and control class
 */
class Robot {
public:
    // Motion profile calculation struct
    struct MotionProfile {
        float accel_time_s;
        float accel_phase_ms;
        bool full_profile;
    };

private:
    // Robot config
    float wheel_radius_mm;
    float wheel_spacing_mm;

    // Robot pose
    float positionX;
    float positionY;
    float orientation;
    float battery_voltage;
    uint8_t system_status;

public:
    // Motor and motion control constants. All commands should be within these bounds otherwise timing may be off.
    static constexpr float STEPS_PER_REVOLUTION = 64.0f;
    static constexpr float ROBOT_MAX_VELOCITY_MM_S = 150.0f;
    static constexpr float ROBOT_MAX_ACCEL_MM_S2 = 50.0f;
    static constexpr float MAX_ROT_VEL_RAD_S = 2.0f;
    static constexpr float MAX_ROT_ACCEL_RAD_S2 = 10.0f;
    
    StepperDriver left_wheel;
    StepperDriver right_wheel;
    
    /**
     * Initialize the robot with default hardware configuration
     * @return true on success, false on failure
     */
    bool initialize();
    
    /**
     * Update robot position based on wheel step counts
     * Uses odometry to calculate position change
     */
    void updatePosition();
    
    /**
     * Get current robot position
     * @param x Pointer to store X position
     * @param y Pointer to store Y position
     * @param orientation Pointer to store orientation
     */
    void getPosition(float* x, float* y, float* orientation);
    
    /**
     * Set robot position (for external updates, e.g., from position estimator)
     * @param x X position
     * @param y Y position
     * @param orientation Robot orientation
     */
    void setPosition(float x, float y, float orientation);

    /**
     * Set target position and orientation, always absolute coordinates
     * @param target MotionCommand struct with target pose
     */
    void setTargetPose(MotionCommand target);

    /**
     * Set target position and orientation, always absolute coordinates
     * @param target MotionCommand struct with target pose
     */
    void setTestState(MotTestCommand target);
    
    /**
     * Set battery voltage (from battery monitor task)
     * @param voltage Battery voltage in volts
     */
    void setBatteryVoltage(float voltage);
    
private:
    MotionProfile calculateMotionProfile(float distance_mm, float max_velocity_mm_s);
    
    float calculateVelocityAtTime(float elapsed_ms, float remaining_ms, float accel_phase_ms,
                                   float max_velocity_mm_s, bool full_profile);
    
    void executeMotionLoop(int32_t total_steps, float base_step_time_us,
                           float max_velocity_mm_s, const MotionProfile& profile,
                           TickType_t start_tick, TickType_t target_end_tick,
                           std::function<void(int32_t)> step_callback);
    
    void executeRotationMotion(float angle_rad, float steps_per_mm, float move_duration_ms);
    
    void executeStraightMotion(float distance, float steps_per_mm, float max_velocity,
                               TickType_t start_tick, TickType_t end_tick);
    
    void executeArcMotion(float dx, float dy, float linear_distance, float orientation_delta,
                          float steps_per_mm, float max_velocity, TickType_t start_tick, TickType_t end_tick);
    
public:
    /**
     * Set system status
     * @param status Status code
     */
    void setStatus(uint8_t status);
    
    /**
     * Get system status
     * @return Current system status code
     */
    uint8_t getStatus();
};

#endif // __ROBOT_H__