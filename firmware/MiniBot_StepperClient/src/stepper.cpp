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

bool StepperDriver::setRMTchannel(rmt_channel_t incoming_rmt_channel) {
    rmt_channel = incoming_rmt_channel;

    // Configure RMT channel
    rmt_config_t rmt_cfg = {};
    rmt_cfg.channel = rmt_channel;
    rmt_cfg.gpio_num = (gpio_num_t)step_pin;
    rmt_cfg.clk_div = 80;  // 80MHz / 80 = 1MHz = 1us per tick
    rmt_cfg.mem_block_num = 1;
    rmt_cfg.tx_config.loop_en = true;
    
    esp_err_t err = rmt_config(&rmt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT config failed: %d", err);
        return false;
    }

    err = rmt_driver_install(rmt_channel, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT driver install failed: %d", err);
        return false;
    }
    rmt_initialized = true;
    return true;
}

bool StepperDriver::configureRMT(float velocity_rad_s, float steps_per_rad) {
    if (!rmt_initialized) {
        ESP_LOGE(TAG, "RMT channel not initialized. Set channel first.");
        return false;
    }

    // Calculate step interval in microseconds (capped at max RMT duration ~32.7ms)
    step_interval_us = (uint32_t)fmax(1, (uint32_t)round(1000000.0f / (velocity_rad_s * steps_per_rad)));
    step_interval_us = fmin(step_interval_us, 32000);  // Cap at RMT max duration

    // Stop any existing transmission
    if (rmt_running) {
        stopRMT();
    }
    
    // Create pulse: HIGH for 1us, LOW for (interval-1) us
    rmt_item32_t pulse = {};
    pulse.level0 = 1;
    pulse.duration0 = 1;  // 1us high
    pulse.level1 = 0;
    pulse.duration1 = fmax(1, step_interval_us - 1);  // Low for rest of interval
    
    esp_err_t err = rmt_write_items(rmt_channel, &pulse, 1, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT write_items failed: %d", err);
        return false;
    }
    return true;
}

bool StepperDriver::startRMT() {
    esp_err_t err = rmt_tx_start(rmt_channel, true);
    if (err != ESP_OK) {
        return false;
    }
    rmt_start_time_us = esp_timer_get_time();
    rmt_running = true;
    return true;
}

bool StepperDriver::stopRMT() {
    esp_err_t err = rmt_tx_stop(rmt_channel);
    if (err != ESP_OK) {
        return false;
    }
    int64_t elapsed_us = esp_timer_get_time() - rmt_start_time_us;
    int64_t steps_taken = elapsed_us / step_interval_us;
    current_step_count += steps_taken * current_direction;

    rmt_running = false;
    rmt_start_time_us = 0;
    return true;
}
