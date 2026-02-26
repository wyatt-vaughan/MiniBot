#include "battery_monitor.h"
#include "esp_now_communicator.h"

static uint8_t battery_adc_pin = 0;
static RollingAverage<BATTERY_AVG_WINDOW_SIZE> battery_avg_v;
static bool low_battery_alerted = false;

bool BatteryMonitor_Init(uint8_t adc_pin) {
    battery_adc_pin = adc_pin;
    pinMode(battery_adc_pin, INPUT);
    return true;
}

void BatteryMonitor_RecordVoltage() {
    uint16_t adc_raw = analogRead(battery_adc_pin);
    float batt_voltage = (adc_raw / 4095.0f) * 3.3f * BATTERY_VOLTAGE_DIVIDER_RATIO;
    battery_avg_v.add(batt_voltage);
}

void BatteryMonitor_Task(void* pvParameters) {
    Robot* robot = (Robot*)pvParameters;
    
    if (robot == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
    while (1) {
        BatteryMonitor_RecordVoltage();
        float current_voltage = battery_avg_v.avg();
        robot->setBatteryVoltage(current_voltage);
        
        // Check for low battery condition and send alert
        if (robot->getBatteryCritical()) {
            if (!low_battery_alerted) {
                Serial.printf("LOW BATTERY DETECTED: %.2fV\n", 
                              current_voltage);
                if (EspNowCommunicator_SendAlert(ERR_LOW_BATTERY)) {
                    Serial.println("Low battery alert sent successfully");
                    low_battery_alerted = true;
                } else {
                    Serial.println("Failed to send low battery alert");
                }
            }
        } else {
            low_battery_alerted = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(BATTERY_POLL_INTERVAL_MS));
    }
}
