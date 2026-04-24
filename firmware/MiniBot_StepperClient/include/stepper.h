#ifndef __STEPPER_H__
#define __STEPPER_H__

#include <Arduino.h>
#include <stdint.h>
#include <driver/rmt.h>

class StepperDriver {
public:
    // Pin configuration
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t enable_pin;
    uint8_t reset_pin;
    
private:
    // State
    bool enabled;
    int8_t current_direction;  // 1 for forward, -1 for reverse
    int32_t current_step_count;

    bool reverse_motor;  // If true, reverses the direction signal
    bool rmt_running = false;
    bool rmt_initialized = false;
    uint32_t step_interval_us = 0;
    int64_t rmt_start_time_us = 0;
    rmt_channel_t rmt_channel;
    
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

    /**
    * Set the RMT channel for this stepper (for control at constant velocity)
    * @param incoming_rmt_channel RMT channel to use for stepping
    * @return true on success, false on failure
    */
    bool setRMTchannel(rmt_channel_t incoming_rmt_channel);

    /**
     * Configure RMT for a specific velocity
     * @param velocity_rad_s Desired velocity in rad/s (used to calculate step interval)
     * @param steps_per_rad Steps per radian for the wheel (used to calculate step interval)
     * @return true on success, false on failure
     */
    bool configureRMT(float velocity_rad_s, float steps_per_rad);

    /**
     * Start RMT transmission for continuous stepping
     * @return true on success, false on failure
     */
    bool startRMT();

    /**
     * Stop RMT transmission
     * @return true on success, false on failure
     */
    bool stopRMT();

    bool getRMTRunning() const { return rmt_running; }

};

#endif // __STEPPER_H__