#include "robot.h"
#include <Arduino.h>
#include <cmath>

// ============================================================================
// StepperDriver Implementation
// ============================================================================

bool StepperDriver::initialize(uint8_t step, uint8_t dir, uint8_t enable, uint8_t reset, bool reverse) {
    step_pin = step;
    dir_pin = dir;
    enable_pin = enable;
    reset_pin = reset;
    reverse_motor = reverse;
    enabled = false;
    current_step_count = 0;
    target_step_count = 0;
    
    // Configure pins as outputs
    pinMode(step_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    pinMode(enable_pin, OUTPUT);
    pinMode(reset_pin, OUTPUT);
    
    digitalWrite(enable_pin, LOW);
    digitalWrite(reset_pin, LOW);
    digitalWrite(step_pin, LOW);
    digitalWrite(dir_pin, LOW);

    return true;
}

bool StepperDriver::setMicrostepping(bool step_lvl, bool dir_lvl) {
    // MS1 hardwired low, MS2 hardwired high
    // step 0 dir 0 = 1/32
    // step 0 dir 1 = 1/4
    // step 1 dir 0 = 1/256
    // step 1 dir 1 = 1/64
    digitalWrite(step_pin, step_lvl);
    digitalWrite(dir_pin, dir_lvl);
    return true;
}

bool StepperDriver::resetDriver() {
    disable();
    digitalWrite(reset_pin, LOW);
    delay(5);
    digitalWrite(reset_pin, HIGH);
    delay(5);
    return true;
}

bool StepperDriver::enable() {
    digitalWrite(enable_pin, HIGH);
    enabled = true;
    return true;
}

bool StepperDriver::disable() {
    digitalWrite(enable_pin, LOW);
    enabled = false;
    return true;
}

bool StepperDriver::setDirection(bool direction) {
    // Apply reversal if configured
    bool actual_direction = reverse_motor ? !direction : direction;
    digitalWrite(dir_pin, actual_direction);
    return true;
}

bool StepperDriver::step() {
    if (!enabled) {
        return false;
    }
    
    // Generate step pulse (pulse high then low)
    // STSPIN220 requires >100ns high pulse
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(1);
    digitalWrite(step_pin, LOW);
    
    current_step_count++;
    
    return true;
}

// ============================================================================
// Robot Implementation
// ============================================================================

bool Robot::initialize() {
    // Initialize stepper drivers with default pinout for ESP32-C3
    // Left wheel: step=6, dir=7, enable=8, reset=9
    // Right wheel: step=10, dir=11, enable=12, reset=13
    if (!left_wheel.initialize(L_WHEEL_STEP_PIN, L_WHEEL_DIR_PIN, STEPPER_EN_PIN, STEPPER_RST_PIN, L_WHEEL_REVERSE)) {
        return false;
    }
    
    if (!right_wheel.initialize(R_WHEEL_STEP_PIN, R_WHEEL_DIR_PIN, STEPPER_EN_PIN, STEPPER_RST_PIN, R_WHEEL_REVERSE)) {
        return false;
    }

    // Set microstepping to 1/4 for both drivers
    // left_wheel.setMicrostepping(false, true);
    // right_wheel.setMicrostepping(false, true);

    // Set microstepping to 1/8 for both drivers
    left_wheel.setMicrostepping(true, false);
    right_wheel.setMicrostepping(true, false);

    // Reset drivers to apply microstepping. Only need to do on one since reset and enable are shared.
    left_wheel.resetDriver();
    
    // Set default robot configuration from config.h
    wheel_radius_mm = WHEEL_RADIUS_MM;
    wheel_spacing_mm = WHEEL_SPACING_MM;
    
    // Initialize motion control constants from config.h
    steps_per_revolution = STEPS_PER_REVOLUTION;
    robot_max_velocity_mm_s = ROBOT_MAX_VELOCITY_MM_S;
    robot_max_accel_mm_s2 = ROBOT_MAX_ACCEL_MM_S2;
    max_rot_vel_rad_s = MAX_ROT_VEL_RAD_S;
    max_rot_accel_rad_s2 = MAX_ROT_ACCEL_RAD_S2;
    stepper_max_velocity_mm_s = STEPPER_MAX_VELOCITY_MM_S;
    
    // Initialize position and state
    positionX = 0.0f;
    positionY = 0.0f;
    orientation = 0.0f;
    battery_voltage = 0.0f;
    system_status = 0;
    
    return true;
}

void Robot::updatePosition() {
    // TODO: Implement odometry calculations based on step counts
    // This will convert stepper motor step counts to X,Y position and orientation
}

void Robot::getPosition(float* x, float* y, float* orientation) {
    if (x != NULL) *x = positionX;
    if (y != NULL) *y = positionY;
    if (orientation != NULL) *orientation = this->orientation;
}

void Robot::setPosition(float x, float y, float orientation) {
    // Allows for position correction from magnetometer task
    positionX = x;
    positionY = y;
    this->orientation = orientation;
}

// ============================================================================
// Motion execution helper functions
// ============================================================================

Robot::MotionProfile Robot::calculateMotionProfile(float distance_mm, float max_velocity_mm_s) {
    float accel_time_s = max_velocity_mm_s / robot_max_accel_mm_s2;
    float accel_distance_mm = 0.5f * robot_max_accel_mm_s2 * accel_time_s * accel_time_s;
    bool full_profile = (distance_mm >= 2.0f * accel_distance_mm);
    
    return {accel_time_s, accel_time_s * 1000.0f, full_profile};
}

float Robot::calculateVelocityAtTime(float elapsed_ms, float remaining_ms, float accel_phase_ms, 
                                      float max_velocity_mm_s, bool full_profile) {
    if (!full_profile) return max_velocity_mm_s;
    
    if (elapsed_ms < accel_phase_ms) {
        return robot_max_accel_mm_s2 * (elapsed_ms / 1000.0f);
    } else if (remaining_ms < accel_phase_ms) {
        return robot_max_accel_mm_s2 * (remaining_ms / 1000.0f);
    }
    return max_velocity_mm_s;
}

void Robot::executeMotionLoop(int32_t total_steps, float base_step_time_us, 
                               float max_velocity_mm_s, const MotionProfile& profile,
                               TickType_t start_tick, TickType_t target_end_tick,
                               std::function<void(int32_t)> step_callback) {
    Serial.printf("    executeMotionLoop: total_steps=%ld, base_step_time=%.2f us, max_vel=%.2f mm/s\n", 
                  total_steps, base_step_time_us, max_velocity_mm_s);
    Serial.printf("    Profile: accel_phase=%.2f ms, full_profile=%s\n", 
                  profile.accel_phase_ms, profile.full_profile ? "YES" : "NO");
    
    TickType_t current_tick;
    float elapsed_ms, remaining_ms, current_velocity_mm_s;
    uint32_t step_delay_us;
    TickType_t step_delay_ticks;
    
    // Print velocity at key steps
    int32_t print_interval = (total_steps > 100) ? (total_steps / 10) : 1;
    
    for (int32_t step_index = 0; step_index < total_steps; step_index++) {
        current_tick = xTaskGetTickCount();
        elapsed_ms = (float)(current_tick - start_tick) * portTICK_PERIOD_MS;
        remaining_ms = (float)(target_end_tick - current_tick) * portTICK_PERIOD_MS;
        
        current_velocity_mm_s = calculateVelocityAtTime(elapsed_ms, remaining_ms, profile.accel_phase_ms, 
                                                         max_velocity_mm_s, profile.full_profile);
        
        step_delay_us = (uint32_t)fmax(1.0f, base_step_time_us * max_velocity_mm_s / current_velocity_mm_s);
        step_delay_ticks = pdMS_TO_TICKS(fmax(1, (uint32_t)(step_delay_us / 1000)));
        
        // Periodic debug output
        if (step_index % print_interval == 0 || step_index == total_steps - 1) {
            Serial.printf("      Step %ld/%ld: vel=%.2f mm/s, delay=%lu us (elapsed=%.1f ms, remaining=%.1f ms)\n",
                          step_index, total_steps, current_velocity_mm_s, step_delay_us, elapsed_ms, remaining_ms);
        }
        
        step_callback(step_index);
        
        if (step_delay_ticks > 0) {
            vTaskDelay(step_delay_ticks);
        } else {
            vTaskDelay(1);
        }
    }
}

// ============================================================================
// Main motion control
// ============================================================================

void Robot::setTargetPose(MotionCommand target) {
    float target_x = target.target_position_x_mm;
    float target_y = target.target_position_y_mm;
    float target_theta = target.target_orientation_rad;
    float move_duration_ms = target.move_duration_ms;

    Serial.printf("\n========================================\n");
    Serial.printf("Robot::setTargetPose CALLED\n");
    Serial.printf("  Target: (%.2f, %.2f, %.4f rad) over %.2f ms\n", target_x, target_y, target_theta, move_duration_ms);

    // Enable motors and give time for them to settle
    left_wheel.enable();
    right_wheel.enable();
    
    // Extract current position
    float current_x = positionX;
    float current_y = positionY;
    float current_theta = orientation;
    
    Serial.printf("  Current: (%.2f, %.2f, %.4f rad)\n", current_x, current_y, current_theta);
    
    // Calculate deltas
    float dx = target_x - current_x;
    float dy = target_y - current_y;
    float linear_distance = sqrt(dx*dx + dy*dy);
    
    float orientation_delta = target_theta - current_theta;
    while (orientation_delta > M_PI) orientation_delta -= 2.0f*M_PI;
    while (orientation_delta < -M_PI) orientation_delta += 2.0f*M_PI;
    
    Serial.printf("  Delta: dx=%.2f mm, dy=%.2f mm\n", dx, dy);
    Serial.printf("  Linear distance: %.2f mm\n", linear_distance);
    Serial.printf("  Orientation delta: %.4f rad (%.2f deg)\n", orientation_delta, orientation_delta * 180.0f / M_PI);
    
    // Calculate motion parameters (common to all move types)
    const float POSITION_TOLERANCE = 5.0f;
    const float STEPS_PER_MM = steps_per_revolution / (2.0f * M_PI * wheel_radius_mm);
    
    Serial.printf("  Steps per mm: %.4f\n", STEPS_PER_MM);
    Serial.printf("  Wheel radius: %.2f mm, spacing: %.2f mm\n", wheel_radius_mm, wheel_spacing_mm);
    
    float max_velocity = (move_duration_ms > 0) 
        ? fmin((linear_distance / (move_duration_ms / 1000.0f)) * 1.5f, robot_max_velocity_mm_s)
        : 100.0f;
    
    Serial.printf("  Max velocity: %.2f mm/s\n", max_velocity);
    
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t end_tick = start_tick + pdMS_TO_TICKS((uint32_t)move_duration_ms);
    
    // Determine motion type and execute
    bool is_rotation_only = (linear_distance < POSITION_TOLERANCE);
    bool is_straightline = (fabs(orientation_delta) < 0.01f);
    
    Serial.printf("  Motion classification:\n");
    Serial.printf("    Rotation only: %s (distance < %.2f mm)\n", is_rotation_only ? "YES" : "NO", POSITION_TOLERANCE);
    Serial.printf("    Straight line: %s (|orientation_delta| < 0.01 rad)\n", is_straightline ? "YES" : "NO");
    Serial.printf("    Arc motion: %s\n", (!is_rotation_only && !is_straightline) ? "YES" : "NO");
    
    uint32_t move_starttime = millis();
    if (is_rotation_only) {
        executeRotationMotion(orientation_delta, STEPS_PER_MM, move_duration_ms);
    } else if (is_straightline) {
        // For straight line motion, check if we need to rotate first to face the target
        // Calculate the required heading to reach the target
        float required_heading = atan2(dy, dx);
        float heading_error = required_heading - current_theta;
        
        // Normalize heading error to [-pi, pi]
        while (heading_error > M_PI) heading_error -= 2.0f*M_PI;
        while (heading_error < -M_PI) heading_error += 2.0f*M_PI;
        
        Serial.printf("  Straight line alignment check:\n");
        Serial.printf("    Required heading: %.4f rad (%.2f deg)\n", required_heading, required_heading * 180.0f / M_PI);
        Serial.printf("    Current heading: %.4f rad (%.2f deg)\n", current_theta, current_theta * 180.0f / M_PI);
        Serial.printf("    Heading error: %.4f rad (%.2f deg)\n", heading_error, heading_error * 180.0f / M_PI);
        
        const float HEADING_TOLERANCE = 0.1f;  // ~5.7 degrees tolerance
        
        // Check if we can move backward (heading error ~180 deg) which might be shorter
        float forward_rotation = heading_error;
        float backward_rotation = heading_error + ((heading_error > 0) ? -M_PI : M_PI);
        
        // Normalize backward rotation
        while (backward_rotation > M_PI) backward_rotation -= 2.0f*M_PI;
        while (backward_rotation < -M_PI) backward_rotation += 2.0f*M_PI;
        
        Serial.printf("    Forward rotation needed: %.4f rad (%.2f deg)\n", forward_rotation, forward_rotation * 180.0f / M_PI);
        Serial.printf("    Backward rotation needed: %.4f rad (%.2f deg)\n", backward_rotation, backward_rotation * 180.0f / M_PI);
        
        bool move_backward = false;
        float rotation_needed = forward_rotation;
        
        // Choose the shorter rotation (forward or backward)
        if (fabs(backward_rotation) < fabs(forward_rotation)) {
            rotation_needed = backward_rotation;
            move_backward = true;
            Serial.printf("    -> Choosing BACKWARD motion (shorter rotation)\n");
        } else {
            Serial.printf("    -> Choosing FORWARD motion\n");
        }
        
        // If rotation needed exceeds tolerance, rotate first
        if (fabs(rotation_needed) > HEADING_TOLERANCE) {
            Serial.printf("  Executing pre-rotation: %.4f rad (%.2f deg)\n", rotation_needed, rotation_needed * 180.0f / M_PI);
            executeRotationMotion(rotation_needed, STEPS_PER_MM, 0);  // No time constraint for alignment
            
            // Update robot's stored orientation after rotation
            orientation = current_theta + rotation_needed;
            while (orientation > M_PI) orientation -= 2.0f*M_PI;
            while (orientation < -M_PI) orientation += 2.0f*M_PI;
            
            Serial.printf("  Orientation after pre-rotation: %.4f rad\n", orientation);
        } else {
            Serial.printf("  No pre-rotation needed (within %.2f rad tolerance)\n", HEADING_TOLERANCE);
        }
        
        // Execute straight line motion (forward or backward)
        executeStraightMotion(linear_distance, STEPS_PER_MM, max_velocity, start_tick, end_tick, move_backward);
    } else {
        executeArcMotion(dx, dy, linear_distance, orientation_delta, STEPS_PER_MM, max_velocity, start_tick, end_tick);
    }

    uint32_t actual_move_time = millis() - move_starttime;
    Serial.printf("  Actual move time: %lu ms (commanded: %.2f ms)\n", actual_move_time, move_duration_ms);
    
    // Update final position
    positionX = target_x;
    positionY = target_y;
    orientation = target_theta;
    
    Serial.printf("  Updated position to: (%.2f, %.2f, %.4f rad)\n", positionX, positionY, orientation);
    
    left_wheel.disable();
    right_wheel.disable();
    Serial.printf("Robot::Move complete\n");
    Serial.printf("========================================\n\n");
}

void Robot::executeRotationMotion(float angle_rad, float steps_per_mm, float move_duration_ms) {
    Serial.println("  Motion type: ROTATION_ONLY");
    
    float wheel_arc = (wheel_spacing_mm / 2.0f) * fabs(angle_rad);
    int32_t num_steps = (int32_t)round(wheel_arc * steps_per_mm);
    Serial.printf("    Angle: %.4f rad, Steps: %ld\n", angle_rad, num_steps);
    
    // Set rotation direction
    bool cw = (angle_rad > 0);
    left_wheel.setDirection(!cw);
    right_wheel.setDirection(cw);
    
    // Calculate required angular velocity accounting for acceleration time
    // Try to fit the motion within move_duration_ms, accounting for accel/decel phases
    float target_rot_vel_rad_s = this->max_rot_vel_rad_s;
    float move_duration_s = move_duration_ms / 1000.0f;
    
    if (move_duration_ms > 0) {
        // Iteratively find the max velocity that fits in the time budget
        // Start with the required velocity and back off if acceleration takes too long
        float required_vel = fabs(angle_rad) / move_duration_s;
        
        for (int attempts = 0; attempts < 5; attempts++) {
            float test_vel = fmin(required_vel, this->max_rot_vel_rad_s);
            float accel_time = test_vel / max_rot_accel_rad_s2;
            float accel_angle = 0.5f * max_rot_accel_rad_s2 * accel_time * accel_time;
            
            float actual_time;
            if (fabs(angle_rad) >= 2.0f * accel_angle) {
                // Full profile fits
                float cruise_angle = fabs(angle_rad) - 2.0f * accel_angle;
                actual_time = 2.0f * accel_time + (cruise_angle / test_vel);
            } else {
                // Triangular profile
                actual_time = 2.0f * sqrt(fabs(angle_rad) / max_rot_accel_rad_s2);
            }
            
            if (actual_time <= move_duration_s) {
                target_rot_vel_rad_s = test_vel;
                break;
            }
            // Reduce velocity and try again
            required_vel *= 0.9f;
        }
    }
    
    Serial.printf("    Commanded duration: %.2f ms, Max velocity: %.4f rad/s\n", move_duration_ms, target_rot_vel_rad_s);
    
    // Calculate rotational acceleration profile with the determined velocity
    float accel_time_s = target_rot_vel_rad_s / max_rot_accel_rad_s2;
    float accel_angle_rad = 0.5f * max_rot_accel_rad_s2 * accel_time_s * accel_time_s;
    bool full_profile = (fabs(angle_rad) >= 2.0f * accel_angle_rad);
    float accel_phase_s = accel_time_s;
    
    // Calculate actual motion time based on profile
    float total_rotation_time_s;
    if (full_profile) {
        // Time = accel + cruise + decel
        float cruise_angle = fabs(angle_rad) - 2.0f * accel_angle_rad;
        float cruise_time = cruise_angle / target_rot_vel_rad_s;
        total_rotation_time_s = 2.0f * accel_time_s + cruise_time;
    } else {
        // Triangular profile: t_total = 2*sqrt(distance / accel)
        total_rotation_time_s = 2.0f * sqrt(fabs(angle_rad) / max_rot_accel_rad_s2);
    }
    
    Serial.printf("    Accel phase: %.2f s, Full profile: %s, Total time: %.2f s\n", accel_phase_s, full_profile ? "YES" : "NO", total_rotation_time_s);
    
    // Base timing at max velocity
    float rotation_linear_vel = target_rot_vel_rad_s * (wheel_spacing_mm / 2.0f);
    float base_step_time_us = (2.0f * M_PI * wheel_radius_mm / steps_per_revolution) / rotation_linear_vel * 1000000.0f;
    
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t end_tick = start_tick + pdMS_TO_TICKS((uint32_t)(total_rotation_time_s * 1000.0f));
    
    TickType_t current_tick;
    float elapsed_s, remaining_s, current_rot_vel_rad_s;
    uint32_t step_delay_us;
    TickType_t step_delay_ticks;
    const float MIN_ROT_VEL = 0.1f;  // Minimum velocity to prevent infinite delays (rad/s)
    
    for (int32_t step_index = 0; step_index < num_steps; step_index++) {
        current_tick = xTaskGetTickCount();
        elapsed_s = (float)(current_tick - start_tick) * portTICK_PERIOD_MS / 1000.0f;
        remaining_s = (float)(end_tick - current_tick) * portTICK_PERIOD_MS / 1000.0f;
        
        current_rot_vel_rad_s = target_rot_vel_rad_s;
        
        if (full_profile) {
            if (elapsed_s < accel_phase_s) {
                // Acceleration phase: ramp from min to max velocity
                current_rot_vel_rad_s = fmax(MIN_ROT_VEL, max_rot_accel_rad_s2 * elapsed_s);
            } else if (remaining_s < accel_phase_s) {
                // Deceleration phase: ramp from max to min velocity
                current_rot_vel_rad_s = fmax(MIN_ROT_VEL, max_rot_accel_rad_s2 * remaining_s);
            }
        }
        
        // Convert current rotational velocity to step delay
        float current_linear_vel = current_rot_vel_rad_s * (wheel_spacing_mm / 2.0f);
        step_delay_us = (uint32_t)fmax(1.0f, base_step_time_us * rotation_linear_vel / current_linear_vel);
        step_delay_us = fmin(step_delay_us, 100000);  // Cap at 100ms to prevent very long delays
        step_delay_ticks = (step_delay_us > 10000) ? pdMS_TO_TICKS(step_delay_us / 1000) : 0;
        
        left_wheel.step();
        right_wheel.step();
        
        if (step_delay_ticks > 0) {
            vTaskDelay(step_delay_ticks);
        } else {
            delayMicroseconds(step_delay_us);
        }
    }
    
    Serial.println("  Rotation complete");
}

void Robot::executeStraightMotion(float distance, float steps_per_mm, float max_velocity,
                                   TickType_t start_tick, TickType_t end_tick, bool move_backward) {
    Serial.println("  Motion type: STRAIGHT_LINE");
    Serial.printf("    Direction: %s\n", move_backward ? "BACKWARD" : "FORWARD");
    
    int32_t num_steps = (int32_t)round(distance * steps_per_mm);
    Serial.printf("    Distance: %.2f mm, Steps: %ld\n", distance, num_steps);
    Serial.printf("    Steps per mm: %.4f\n", steps_per_mm);
    Serial.printf("    Max velocity: %.2f mm/s\n", max_velocity);
    
    // Set direction based on move_backward flag
    left_wheel.setDirection(!move_backward);
    right_wheel.setDirection(!move_backward);
    Serial.printf("    Direction set: %s (both motors)\n", move_backward ? "REVERSE" : "FORWARD");
    
    MotionProfile profile = calculateMotionProfile(distance, max_velocity);
    Serial.printf("    Motion profile calculated: accel_time=%.4f s, accel_phase=%.2f ms, full=%s\n",
                  profile.accel_time_s, profile.accel_phase_ms, profile.full_profile ? "YES" : "NO");
    
    // Calculate actual motion time based on profile
    float total_motion_time_s;
    if (profile.full_profile) {
        // Time = accel + cruise + decel
        float accel_distance = 0.5f * robot_max_accel_mm_s2 * profile.accel_time_s * profile.accel_time_s;
        float cruise_distance = distance - 2.0f * accel_distance;
        float cruise_time = cruise_distance / max_velocity;
        total_motion_time_s = 2.0f * profile.accel_time_s + cruise_time;
        Serial.printf("    Full profile: accel_dist=%.2f mm, cruise_dist=%.2f mm, cruise_time=%.4f s\n",
                      accel_distance, cruise_distance, cruise_time);
    } else {
        // Triangular profile: t_total = 2*sqrt(distance / accel)
        total_motion_time_s = 2.0f * sqrt(distance / robot_max_accel_mm_s2);
        Serial.printf("    Triangular profile (no cruise phase)\n");
    }
    
    // Override end_tick with corrected time
    end_tick = start_tick + pdMS_TO_TICKS((uint32_t)(total_motion_time_s * 1000.0f));
    
    Serial.printf("    Total motion time: %.4f s (%.2f ms)\n", total_motion_time_s, total_motion_time_s * 1000.0f);
    Serial.printf("    Robot accel: %.2f mm/s^2\n", robot_max_accel_mm_s2);
    
    float step_time_us = (2.0f * M_PI * wheel_radius_mm / steps_per_revolution) / max_velocity * 1000000.0f;
    Serial.printf("    Base step time: %.2f us\n", step_time_us);
    
    executeMotionLoop(num_steps, step_time_us, max_velocity, profile, start_tick, end_tick,
        [this](int32_t idx) {
            left_wheel.step();
            right_wheel.step();
        });
    
    Serial.println("  Straight line complete");
}

void Robot::executeArcMotion(float dx, float dy, float linear_distance, float orientation_delta,
                              float steps_per_mm, float max_velocity, TickType_t start_tick, TickType_t end_tick) {
    Serial.println("  Motion type: ARC_MOTION");
    
    // Calculate arc geometry
    float arc_radius;
    if (fabs(orientation_delta) > 0.01f) {
        float sin_half = sin(fabs(orientation_delta) / 2.0f);
        arc_radius = (sin_half > 0.001f) ? (linear_distance / (2.0f * sin_half)) : 1e6f;
    } else {
        arc_radius = 1e6f;
    }
    
    float outer_distance = linear_distance * (arc_radius + wheel_spacing_mm/2.0f) / arc_radius;
    float inner_distance = linear_distance * (arc_radius - wheel_spacing_mm/2.0f) / arc_radius;
    
    if (orientation_delta < 0) {
        float temp = outer_distance;
        outer_distance = -inner_distance;
        inner_distance = -temp;
    }
    
    int32_t outer_steps = (int32_t)round(fabs(outer_distance) * steps_per_mm);
    int32_t inner_steps = (int32_t)round(fabs(inner_distance) * steps_per_mm);
    
    bool left_is_outer = (orientation_delta > 0);
    int32_t left_steps = left_is_outer ? outer_steps : inner_steps;
    int32_t right_steps = left_is_outer ? inner_steps : outer_steps;
    int32_t max_steps = (left_steps > right_steps) ? left_steps : right_steps;
    
    Serial.printf("    Arc radius: %.2f mm, Left steps: %ld, Right steps: %ld\n", arc_radius, left_steps, right_steps);

    left_wheel.setDirection(outer_distance >= 0);
    right_wheel.setDirection(inner_distance >= 0);
    
    MotionProfile profile = calculateMotionProfile(linear_distance, max_velocity);
    
    // Calculate actual motion time based on outer wheel distance and profile
    // The outer wheel travels the furthest distance in arc motion
    float total_motion_time_s;
    if (profile.full_profile) {
        // For arc motion, use outer wheel distance to calculate cruise distance
        float accel_distance = 0.5f * robot_max_accel_mm_s2 * profile.accel_time_s * profile.accel_time_s;
        float cruise_distance = fabs(outer_distance) - 2.0f * accel_distance;
        float cruise_time = cruise_distance / max_velocity;
        total_motion_time_s = 2.0f * profile.accel_time_s + cruise_time;
    } else {
        // Triangular profile based on outer wheel distance
        total_motion_time_s = 2.0f * sqrt(fabs(outer_distance) / robot_max_accel_mm_s2);
    }
    
    // Override end_tick with corrected time
    end_tick = start_tick + pdMS_TO_TICKS((uint32_t)(total_motion_time_s * 1000.0f));
    
    float step_time_us = (fabs(outer_distance) / outer_steps) / max_velocity * 1000000.0f;
    
    Serial.printf("    Accel phase: %.2f ms, Full profile: %s, Total time: %.2f s\n", profile.accel_phase_ms, profile.full_profile ? "YES" : "NO", total_motion_time_s);
    
    int32_t left_count = 0, right_count = 0;
    
    executeMotionLoop(max_steps, step_time_us, max_velocity, profile, start_tick, end_tick,
        [this, left_steps, right_steps, max_steps, &left_count, &right_count](int32_t idx) {
            float left_progress = (left_steps > 0) ? ((float)idx / max_steps) : 0.0f;
            float right_progress = (right_steps > 0) ? ((float)idx / max_steps) : 0.0f;
            
            int32_t left_target = (int32_t)round(left_progress * left_steps);
            int32_t right_target = (int32_t)round(right_progress * right_steps);
            
            while (left_count < left_target) {
                left_wheel.step();
                left_count++;
            }
            while (right_count < right_target) {
                right_wheel.step();
                right_count++;
            }
        });
    
    Serial.println("  Arc motion complete");
}

void Robot::setTestState(MotTestCommand test_cmd) {
    if (!test_cmd.enabled) {
        left_wheel.disable();
        right_wheel.disable();
        return;
    }
    
    const float MOTION_DURATION_S = 5.0f;
    
    left_wheel.enable();
    right_wheel.enable();
    
    // Convert rad/s to linear velocity (mm/s)
    float m0_linear_velocity_mm_s = test_cmd.m0_vel * wheel_radius_mm;
    float m1_linear_velocity_mm_s = test_cmd.m1_vel * wheel_radius_mm;
    
    // Distance traveled in 5 seconds
    float m0_distance_mm = fabs(m0_linear_velocity_mm_s * MOTION_DURATION_S);
    float m1_distance_mm = fabs(m1_linear_velocity_mm_s * MOTION_DURATION_S);
    
    // Convert distance to steps
    float steps_per_mm = steps_per_revolution / (2.0f * M_PI * wheel_radius_mm);
    int32_t m0_steps = (int32_t)round(m0_distance_mm * steps_per_mm);
    int32_t m1_steps = (int32_t)round(m1_distance_mm * steps_per_mm);
    
    // Set motor directions
    left_wheel.setDirection(test_cmd.m0_vel >= 0);
    right_wheel.setDirection(test_cmd.m1_vel >= 0);
    
    // Calculate step timing: delay_ms = 1000 / steps_per_second
    float m0_steps_per_second = fabs(test_cmd.m0_vel) * steps_per_revolution / (2.0f * M_PI);
    float m1_steps_per_second = fabs(test_cmd.m1_vel) * steps_per_revolution / (2.0f * M_PI);
    
    uint32_t m0_delay_ms = (m0_steps_per_second > 0) ? (uint32_t)fmax(1, 1000.0f / m0_steps_per_second) : 1000;
    uint32_t m1_delay_ms = (m1_steps_per_second > 0) ? (uint32_t)fmax(1, 1000.0f / m1_steps_per_second) : 1000;
    
    TickType_t m0_delay_ticks = pdMS_TO_TICKS(m0_delay_ms);
    TickType_t m1_delay_ticks = pdMS_TO_TICKS(m1_delay_ms);
    
    int32_t m0_step_count = 0;
    int32_t m1_step_count = 0;
    
    int32_t max_steps = (m0_steps > m1_steps) ? m0_steps : m1_steps;
    
    for (int32_t step_index = 0; step_index < max_steps; step_index++) {
        // Synchronized stepping with linear interpolation
        float m0_progress = (m0_steps > 0) ? ((float)step_index / max_steps) : 0.0f;
        float m1_progress = (m1_steps > 0) ? ((float)step_index / max_steps) : 0.0f;
        
        int32_t m0_target = (int32_t)round(m0_progress * m0_steps);
        int32_t m1_target = (int32_t)round(m1_progress * m1_steps);
        
        // Step motor 0 to target
        while (m0_step_count < m0_target) {
            left_wheel.step();
            m0_step_count++;
        }
        
        // Step motor 1 to target
        while (m1_step_count < m1_target) {
            right_wheel.step();
            m1_step_count++;
        }
        
        // Delay with average of both motor timings
        TickType_t avg_delay = (m0_delay_ticks + m1_delay_ticks) / 2;
        vTaskDelay(fmax(1, avg_delay));
    }
    
    left_wheel.disable();
    right_wheel.disable();
}

void Robot::setBatteryVoltage(float voltage) {
    battery_voltage = voltage;
}

void Robot::setStatus(uint8_t status) {
    system_status = status;
}

uint8_t Robot::getStatus() {
    return system_status;
}
