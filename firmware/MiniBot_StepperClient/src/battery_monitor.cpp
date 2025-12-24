#include "battery_monitor.h"

static uint8_t battery_adc_pin = 0;
static float low_battery_threshold = 2.8f;

bool BatteryMonitor_Init(uint8_t adc_pin) {
    battery_adc_pin = adc_pin;
    pinMode(adc_pin, INPUT);
    return true;
}

void BatteryMonitor_SetLowBatteryThreshold(float threshold) {
    if (threshold > 0.0f) {
        low_battery_threshold = threshold;
    }
}

float BatteryMonitor_GetLowBatteryThreshold(void) {
    return low_battery_threshold;
}

void BatteryMonitor_Task(void* pvParameters) {
    Robot* robot = (Robot*)pvParameters;
    
    if (robot == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        uint16_t adc_raw = analogRead(battery_adc_pin);
        
        // Convert 12-bit ADC (0-4095) to voltage: (adc/4095) * 3.3V * 2.0 (voltage divider ratio)
        float batt_voltage = (adc_raw / 4095.0f) * 3.3f * BATTERY_VOLTAGE_DIVIDER_RATIO;
        
        robot->setBatteryVoltage(batt_voltage);
        
        if (batt_voltage < low_battery_threshold) {
            // TODO: Trigger low battery alert
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
