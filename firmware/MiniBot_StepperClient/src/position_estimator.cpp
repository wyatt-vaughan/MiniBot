#include "position_estimator.h"
#include "mmc5633.h"
#include "utils.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>

struct EmagReading {
    float x[MAX_SAMPLES_PER_EMAG];
    float y[MAX_SAMPLES_PER_EMAG];
    float z[MAX_SAMPLES_PER_EMAG];
    uint16_t count;
};

struct EmagFrameData {
    EmagReading readings[EMAG_COUNT];
    float bg_x;
    float bg_y;
    float bg_z;
};

enum PositionEstState {
    STATE_WAITING_PAUSE,
    STATE_START_PULSES,
    STATE_MEASURING,
    STATE_IDLE
};

enum PulseDetectState {
    PULSE_IDLE,
    PULSE_HIGH,
    PULSE_LOW
};

static MMC5633NJL mag;
static RollingAverage<10> avgX;
static RollingAverage<10> avgY;
static RollingAverage<10> avgZ;
static bool mag_initialized = false;

static EmagReading emag_readings[EMAG_COUNT];
static PositionEstState current_state = STATE_WAITING_PAUSE;
static uint32_t state_start_time = 0;
static uint8_t current_emag_index = 0;
static uint8_t start_pulse_count = 0;

static PulseDetectState pulse_state = PULSE_IDLE;
static uint32_t pulse_transition_time = 0;
static uint8_t detected_pulse_count = 0;

static QueueHandle_t emag_frame_queue = NULL;

bool PositionEstimator_Init(void) {
    emag_frame_queue = xQueueCreate(1, sizeof(EmagFrameData));
    if (emag_frame_queue == NULL) {
        Serial.println("ERROR: Failed to create emag frame queue");
        return false;
    }

    if (!mag.begin(SDA_PIN, SCL_PIN)) {
        Serial.println("ERROR: MMC5633NJL init failed");
        return false;
    }
    
    Serial.println("MMC5633NJL magnetometer ready");
    
    if (!mag.runSelfTest()) {
        Serial.println("WARNING: Magnetometer self-test failed");
    } else {
        Serial.println("Magnetometer self-test passed");
    }

    mag_initialized = true;
    return true;
}

bool PositionEstimator_GetLatestMagneticField(float* x, float* y, float* z) {
    if (!mag_initialized || x == NULL || y == NULL || z == NULL) {
        return false;
    }
    
    *x = avgX.avg();
    *y = avgY.avg();
    *z = avgZ.avg();
    return true;
}

static void trimAndAverageSamples(const EmagReading* reading, float* avg_x, float* avg_y, float* avg_z) {
    uint16_t trim_samples = (EMAG_TRIM_MS * 1000) / EMAG_SAMPLE_PERIOD_US;
    
    if (reading->count < (trim_samples * 2)) {
        *avg_x = 0.0f;
        *avg_y = 0.0f;
        *avg_z = 0.0f;
        return;
    }
    
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    uint16_t valid_samples = 0;
    
    for (uint16_t i = trim_samples; i < (reading->count - trim_samples); i++) {
        sum_x += reading->x[i];
        sum_y += reading->y[i];
        sum_z += reading->z[i];
        valid_samples++;
    }
    
    if (valid_samples > 0) {
        *avg_x = sum_x / valid_samples;
        *avg_y = sum_y / valid_samples;
        *avg_z = sum_z / valid_samples;
    } else {
        *avg_x = 0.0f;
        *avg_y = 0.0f;
        *avg_z = 0.0f;
    }
}

static bool computePositionFromFrameData(const EmagFrameData* frame, Robot* robot) {
    Serial.println("Computing position from frame data...TODO :)");
    return false;
}

static bool detectStartPulse(float mx, float my, float mz, uint32_t current_time) {
    float mx_cal = mx - avgX.avg();
    float my_cal = my - avgY.avg();
    float mz_cal = mz - avgZ.avg();
    
    float field_magnitude = sqrtf(mx_cal * mx_cal + my_cal * my_cal + mz_cal * mz_cal);
    bool field_high = field_magnitude > FIELD_THRESHOLD_GAUSS;
    
    switch (pulse_state) {
        case PULSE_IDLE:
            if (field_high) {
                pulse_state = PULSE_HIGH;
                pulse_transition_time = current_time;
                detected_pulse_count = 0;
            }
            break;
            
        case PULSE_HIGH: {
            uint32_t elapsed = current_time - pulse_transition_time;
            
            if (!field_high) {
                if (elapsed >= PULSE_ON_MIN_MS && elapsed <= PULSE_ON_MAX_MS) {
                    pulse_state = PULSE_LOW;
                    pulse_transition_time = current_time;
                    detected_pulse_count++;
                } else {
                    pulse_state = PULSE_IDLE;
                    detected_pulse_count = 0;
                }
            } else if (elapsed > PULSE_ON_MAX_MS) {
                pulse_state = PULSE_IDLE;
                detected_pulse_count = 0;
            }
            break;
        }
            
        case PULSE_LOW: {
            uint32_t elapsed = current_time - pulse_transition_time;
            
            if (field_high) {
                if (elapsed >= PULSE_OFF_MIN_MS && elapsed <= PULSE_OFF_MAX_MS) {
                    if (detected_pulse_count >= EMAG_START_PULSE_COUNT) {
                        pulse_state = PULSE_IDLE;
                        detected_pulse_count = 0;
                        return true;
                    } else {
                        pulse_state = PULSE_HIGH;
                        pulse_transition_time = current_time;
                    }
                } else {
                    pulse_state = PULSE_IDLE;
                    detected_pulse_count = 0;
                }
            } else if (elapsed > PULSE_OFF_MAX_MS) {
                pulse_state = PULSE_IDLE;
                detected_pulse_count = 0;
            }
            break;
        }
    }
    
    return false;
}

void PositionEstimator_SensorTask(void* pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(100));

    if (emag_frame_queue == NULL) {
        Serial.println("ERROR: emag_frame_queue not initialized");
        vTaskDelete(NULL);
        return;
    }

    if (!mag.enableContinuousMode()) {
        Serial.println("ERROR: Failed to enable continuous mode");
        vTaskDelete(NULL);
        return;
    }

    current_state = STATE_WAITING_PAUSE;
    uint32_t last_sample_time = micros();
    uint32_t pause_quiet_start = 0;
    bool pause_was_quiet = false;

    while (1) {
        uint32_t current_time = millis();
        uint32_t current_micros = micros();
        uint32_t elapsed_micros = current_micros - last_sample_time;

        // If within 100us of next sample time, do a non-yielding wait
        if ((elapsed_micros < EMAG_SAMPLE_PERIOD_US) && (elapsed_micros >= (EMAG_SAMPLE_PERIOD_US - 100))) {
            delayMicroseconds(EMAG_SAMPLE_PERIOD_US - elapsed_micros);
            current_micros = micros();
        }

        if ((current_micros - last_sample_time) >= EMAG_SAMPLE_PERIOD_US) {
            last_sample_time = current_micros;

            if (mag_initialized && mag.readMeasurement()) {
                float mx = mag.getFieldGaussX();
                float my = mag.getFieldGaussY();
                float mz = mag.getFieldGaussZ();

                avgX.add(mx);
                avgY.add(my);
                avgZ.add(mz);

                float mx_cal = mx - avgX.avg();
                float my_cal = my - avgY.avg();
                float mz_cal = mz - avgZ.avg();
                float field_magnitude = sqrtf(mx_cal * mx_cal + my_cal * my_cal + mz_cal * mz_cal);

                Serial.printf("%lu\t%.3f\t%.3f\t%.3f\t%.3f\n", current_micros, mx, my, mz, field_magnitude);

                // switch (current_state) {

                //     case STATE_WAITING_PAUSE: {
                //         if (field_magnitude < FIELD_THRESHOLD_GAUSS) {
                //             if (!pause_was_quiet) {
                //                 pause_was_quiet = true;
                //                 pause_quiet_start = current_time;
                //             } else if ((current_time - pause_quiet_start) >= EMAG_PAUSE_TIME_MS) {
                //                 pulse_state = PULSE_IDLE;
                //                 detected_pulse_count = 0;
                //                 current_state = STATE_START_PULSES;
                //             }
                //         } else {
                //             pause_was_quiet = false;
                //         }
                //         break;
                //     }

                //     case STATE_START_PULSES: {
                //         if (detectStartPulse(mx, my, mz, current_time)) {
                //             for (uint8_t i = 0; i < EMAG_COUNT; i++) {
                //                 emag_readings[i].count = 0;
                //             }
                //             state_start_time = current_time;
                //             current_state = STATE_MEASURING;
                //         }
                //         break;
                //     }

                //     case STATE_MEASURING: {
                //         uint32_t elapsed_ms = current_time - state_start_time;
                //         const uint32_t slot_duration = EMAG_ON_TIME_MS + EMAG_GAP_TIME_MS;
                //         uint8_t emag_index = (uint8_t)(elapsed_ms / slot_duration);
                //         bool within_on_window = (elapsed_ms % slot_duration) < EMAG_ON_TIME_MS;

                //         if (emag_index < EMAG_COUNT && within_on_window) {
                //             EmagReading* r = &emag_readings[emag_index];
                //             if (r->count < MAX_SAMPLES_PER_EMAG) {
                //                 r->x[r->count] = mx;
                //                 r->y[r->count] = my;
                //                 r->z[r->count] = mz;
                //                 r->count++;
                //             }
                //         }

                //         if (emag_index >= EMAG_COUNT) {
                //             EmagFrameData frame;
                //             memcpy(frame.readings, emag_readings, sizeof(emag_readings));
                //             frame.bg_x = avgX.avg();
                //             frame.bg_y = avgY.avg();
                //             frame.bg_z = avgZ.avg();
                //             xQueueOverwrite(emag_frame_queue, &frame);

                //             for (uint8_t i = 0; i < EMAG_COUNT; i++) {
                //                 emag_readings[i].count = 0;
                //             }
                //             pause_was_quiet = false;
                //             current_state = STATE_WAITING_PAUSE;
                //         }
                //         break;
                //     }

                //     case STATE_IDLE:
                //     default:
                //         current_state = STATE_WAITING_PAUSE;
                //         break;
                // }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    mag.disableContinuousMode();
}

void PositionEstimator_CalcTask(void* pvParameters) {
    Robot* robot = (Robot*)pvParameters;

    if (robot == NULL || emag_frame_queue == NULL) {
        Serial.println("ERROR: PositionEstimator_CalcTask bad parameters");
        vTaskDelete(NULL);
        return;
    }

    EmagFrameData frame;

    while (1) {
        if (xQueueReceive(emag_frame_queue, &frame, portMAX_DELAY) == pdTRUE) {
            if (!computePositionFromFrameData(&frame, robot)) {
                Serial.println("WARNING: Position computation failed for frame");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
