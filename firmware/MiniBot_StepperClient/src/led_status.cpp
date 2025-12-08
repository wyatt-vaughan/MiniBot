#include "led_status.h"

// Static variables for task state
static uint8_t led_pin = 0;
static LedStatus current_led_status = LED_STATUS_STARTUP;

bool LedStatus_Init(uint8_t pin) {
    led_pin = pin;
    
    // Configure LED pin as output
    pinMode(led_pin, OUTPUT);
    digitalWrite(led_pin, LOW);
    
    return true;
}

void LedStatus_SetStatus(LedStatus status) {
    current_led_status = status;
}

LedStatus LedStatus_GetStatus(void) {
    return current_led_status;
}

/**
 * Blink pattern helpers
 */
static void blink_slow(uint8_t pin) {
    digitalWrite(pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(pin, LOW);
    vTaskDelay(pdMS_TO_TICKS(500));
}

static void blink_fast(uint8_t pin) {
    digitalWrite(pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(250));
    digitalWrite(pin, LOW);
    vTaskDelay(pdMS_TO_TICKS(250));
}

static void blink_pattern(uint8_t pin, uint8_t count, uint16_t on_ms, uint16_t off_ms) {
    for (uint8_t i = 0; i < count; i++) {
        digitalWrite(pin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        digitalWrite(pin, LOW);
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

void LedStatus_Task(void* pvParameters) {
    // Extract robot pointer from task parameters (not used, but available if needed)
    Robot* robot = (Robot*)pvParameters;
    
    // Task initialization
    if (led_pin == 0) {
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        // Update LED pattern based on current system status
        switch (current_led_status) {
            case LED_STATUS_STARTUP:
                // Rapid blinks during startup
                blink_fast(led_pin);
                break;
                
            case LED_STATUS_READY:
                // Slow steady blink
                blink_slow(led_pin);
                break;
                
            case LED_STATUS_MOVING:
                // Quick double blinks
                blink_pattern(led_pin, 2, 100, 100);
                vTaskDelay(pdMS_TO_TICKS(200));
                break;
                
            case LED_STATUS_ERROR:
                // Three rapid blinks
                blink_pattern(led_pin, 3, 100, 100);
                vTaskDelay(pdMS_TO_TICKS(500));
                break;
                
            case LED_STATUS_LOW_BATTERY:
                // Alternating on/off with longer off time
                blink_pattern(led_pin, 1, 200, 1000);
                break;
                
            default:
                digitalWrite(led_pin, LOW);
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }
    }
}
