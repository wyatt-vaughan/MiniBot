#undef LOG_LOCAL_LEVEL
#define LOG_LOCAL_LEVEL LOG_LEVEL_ROBOT
#include "esp_log.h"
#include "config.h"
#include "stepper.h"

static const char* TAG = "ROBOT";

bool StepperDriver::initialize(uint8_t step, uint8_t dir, uint8_t enable, uint8_t reset, bool reverse) {
    step_pin = step;
    dir_pin = dir;
    enable_pin = enable;
    reset_pin = reset;
    reverse_motor = reverse;
    enabled = false;
    current_direction = 1;
    current_step_count = 0;
    
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
    // See config.h for microstepping settings
    digitalWrite(step_pin, step_lvl);
    digitalWrite(dir_pin, dir_lvl);
    return true;
}

bool StepperDriver::resetDriver() {
    disable();
    digitalWrite(reset_pin, LOW);
    vTaskDelay(pdMS_TO_TICKS(5));
    digitalWrite(reset_pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(5));
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
    current_direction = direction ? 1 : -1;
    bool actual_direction = reverse_motor ? !direction : direction;
    digitalWrite(dir_pin, actual_direction);
    return true;
}

bool StepperDriver::step() {
    if (!enabled) {
        return false;
    }
    
    digitalWrite(step_pin, HIGH);
    delayMicroseconds(1);  // STSPIN220 requires >100ns pulse width
    digitalWrite(step_pin, LOW);
    
    current_step_count += current_direction;
    
    return true;
}

bool StepperDriver::stepISR() {
    return step();
}


bool StepperDriver::configureTimer(float velocity_rad_s, float steps_per_rad) {
    if (!timer_initialized) {
        ESP_LOGE(TAG, "Timer not initialized. Set timer first.");
        return false;
    }

    // Calculate step interval in microseconds
    step_interval_us = (uint32_t)fmax(1, (uint32_t)round(1000000.0f / (velocity_rad_s * steps_per_rad)));
    step_interval_us = fmin(step_interval_us, 100000);  // Cap at 100ms to avoid shenanigans

    // Stop any existing transmission
    if (timer_running) {
        stopTimer();
    }
    
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT, // Select the default clock source
    .direction = GPTIMER_COUNT_UP,      // Counting direction is up
    .resolution_hz = 1 * 1000 * 1000,   // Resolution is 1 MHz, i.e., 1 tick equals 1 microsecond
};
    
    esp_err_t err = rmt_write_items(rmt_channel, &pulse, 1, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Timer config failed: %d", err);
        return false;
    }
    return true;
}

void StepperDriver::startTimer() {
    if (!timer_initialized) return;
    // Disconnect step pin from RMT and reclaim as a plain GPIO output

}

void StepperDriver::stopTimer() {
    if (!timer_initialized) return;

}
