#ifndef __ROBOT_H__
#define __ROBOT_H__

#include "config.h"
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
    
private:
    bool reverse_motor;  // If true, reverses the direction signal
    
public:
    /**
     * Initialize stepper driver with GPIO pins
     * @param step GPIO pin for step signal
     * @param dir GPIO pin for direction signal
     * @param enable GPIO pin for enable signal
     * @param reset GPIO pin for reset signal
     * @param reverse If true, reverse the motor direction
     * @return true on success, false on failure
     */
    bool initialize(uint8_t step, uint8_t dir, uint8_t enable, uint8_t reset, bool reverse = false);

    /**
     * Set microstepping mode. Requires driver reset after setting.
     * Remember the reset and en pins are shared, so set both drivers
     * first before resetting.
     * @param step_lvl Logic level for STEP pin
     * @param dir_lvl Logic level for DIR pin
     * @return true on success, false on failure
     */
    bool setMicrostepping(bool step_lvl, bool dir_lvl);

    /**
     * Reset the stepper driver
     * @return true on success, false on failure
     */
    bool resetDriver();
    
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

    // Robot pose (active/commanded position)
    float positionX;
    float positionY;
    float orientation;
    
    // True position estimate (continuously updated by position estimator)
    float true_x;
    float true_y;
    float true_theta;
    
    float battery_voltage;
    uint8_t system_status;

    // Motor and motion control constants - initialized from config.h but modifiable
    float steps_per_revolution;
    float robot_max_velocity_mm_s;
    float robot_max_accel_mm_s2;
    float max_rot_vel_rad_s;
    float max_rot_accel_rad_s2;
    float stepper_max_velocity_mm_s;

public:
    StepperDriver left_wheel;
    StepperDriver right_wheel;
    
    /**
     * Initialize the robot with default hardware configuration
     * @return true on success, false on failure
     */
    bool initialize();
    
    /**
     * Update the true position estimate based on wheel step counts
     * Uses odometry to calculate position change
     * This is called continuously by the position estimator task
     */
    void updateTruePosition();
    
    /**
     * Copy the true position estimate to the active position
     * This should be called by tasks that need to sync the active position
     * with the latest position estimate (e.g., after motion completion)
     */
    void updatePositionFromEstimate();
    
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
    
    /**
     * Get battery voltage
     * @return Battery voltage in volts
     */
    float getBatteryVoltage() const { return battery_voltage; }
    
    // Motion control configuration getters
    float getStepsPerRevolution() const { return steps_per_revolution; }
    float getRobotMaxVelocity() const { return robot_max_velocity_mm_s; }
    float getRobotMaxAccel() const { return robot_max_accel_mm_s2; }
    float getMaxRotVel() const { return max_rot_vel_rad_s; }
    float getMaxRotAccel() const { return max_rot_accel_rad_s2; }
    float getStepperMaxVelocity() const { return stepper_max_velocity_mm_s; }
    
    // Motion control configuration setters
    void setStepsPerRevolution(float steps) { steps_per_revolution = steps; }
    void setRobotMaxVelocity(float vel) { robot_max_velocity_mm_s = vel; }
    void setRobotMaxAccel(float accel) { robot_max_accel_mm_s2 = accel; }
    void setMaxRotVel(float vel) { max_rot_vel_rad_s = vel; }
    void setMaxRotAccel(float accel) { max_rot_accel_rad_s2 = accel; }
    void setStepperMaxVelocity(float vel) { stepper_max_velocity_mm_s = vel; }
    
private:
    MotionProfile calculateMotionProfile(float distance_mm, float max_velocity_mm_s);
    
    float calculateVelocityAtTime(float elapsed_ms, float remaining_ms, float accel_phase_ms,
                                   float max_velocity_mm_s, bool full_profile);
    
    float calculateMaxVelocityForTime(float distance_mm, float target_time_s, float max_accel);
    
    void executeMotionLoop(int32_t total_steps, float base_step_time_us,
                           float max_velocity_mm_s, const MotionProfile& profile,
                           TickType_t start_tick, TickType_t target_end_tick,
                           std::function<void(int32_t)> step_callback);
    
    void executeRotationMotion(float angle_rad, float steps_per_mm, float move_duration_ms);
    
    void executeStraightMotion(float distance, float steps_per_mm, float max_velocity,
                               TickType_t start_tick, TickType_t end_tick, bool move_backward = false);
    
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