#include "position_estimator.h"
#include "position_lut.h"
#include "mmc5633.h"
#include "config.h"
#include <Wire.h>

// Electromagnet positioning system timing parameters (all in milliseconds)
#define EMAG_PAUSE_TIME_MS           100
#define EMAG_START_PULSE_COUNT       3
#define EMAG_START_PULSE_ON_MS       3
#define EMAG_START_PULSE_OFF_MS      3
#define EMAG_START_TOTAL_MS          (EMAG_START_PULSE_COUNT * (EMAG_START_PULSE_ON_MS + EMAG_START_PULSE_OFF_MS))
#define EMAG_MEASUREMENT_PHASE_MS    60
#define EMAG_OFF_TIME_BETWEEN_MS     18
#define EMAG_TOTAL_FRAME_MS          (EMAG_PAUSE_TIME_MS + EMAG_START_TOTAL_MS + EMAG_MEASUREMENT_PHASE_MS + EMAG_OFF_TIME_BETWEEN_MS)

#define EMAG_COUNT                   6
#define EMAG_ON_TIME_MS              12
#define EMAG_GAP_TIME_MS             3
#define EMAG_TRIM_MS                 1
#define EMAG_SAMPLE_PERIOD_US        500  // 2kHz = 500us period

#define MAX_SAMPLES_PER_EMAG         ((EMAG_ON_TIME_MS * 1000) / EMAG_SAMPLE_PERIOD_US)

struct EmagReading {
    float x[MAX_SAMPLES_PER_EMAG];
    float y[MAX_SAMPLES_PER_EMAG];
    float z[MAX_SAMPLES_PER_EMAG];
    uint16_t count;
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

#define FIELD_THRESHOLD_GAUSS        0.5f  // Threshold for detecting electromagnet on
#define PULSE_ON_MIN_MS              2
#define PULSE_ON_MAX_MS              4
#define PULSE_OFF_MIN_MS             2
#define PULSE_OFF_MAX_MS             4

static MMC5633NJL mag;
static RollingAverage<200> avgX;
static RollingAverage<200> avgY;
static RollingAverage<200> avgZ;
static bool mag_initialized = false;

static EmagReading emag_readings[EMAG_COUNT];
static PositionEstState current_state = STATE_WAITING_PAUSE;
static uint32_t state_start_time = 0;
static uint8_t current_emag_index = 0;
static uint8_t start_pulse_count = 0;

static PulseDetectState pulse_state = PULSE_IDLE;
static uint32_t pulse_transition_time = 0;
static uint8_t detected_pulse_count = 0;

bool PositionEstimator_Init(void) {
    Wire.begin(SDA_PIN, SCL_PIN);
    
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

static void trimAndAverageSamples(EmagReading* reading, float* avg_x, float* avg_y, float* avg_z) {
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

static bool computePositionFromReadings(Robot* robot) {
    float measured_intensities[EMAG_AXES];
    
    for (uint8_t i = 0; i < EMAG_COUNT; i++) {
        float avg_x, avg_y, avg_z;
        trimAndAverageSamples(&emag_readings[i], &avg_x, &avg_y, &avg_z);
        
        float mx_cal = avg_x - avgX.avg();
        float my_cal = avg_y - avgY.avg();
        float mz_cal = avg_z - avgZ.avg();
        
        measured_intensities[i] = sqrtf(mx_cal * mx_cal + my_cal * my_cal + mz_cal * mz_cal);
    }
    
    PositionEstimate estimate;
    if (PositionLUT_ComputePosition(measured_intensities, &estimate)) {
        robot->setTruePose(estimate.x_mm, estimate.y_mm, estimate.theta_rad);
        
        Serial.print("Position: (");
        Serial.print(estimate.x_mm);
        Serial.print(", ");
        Serial.print(estimate.y_mm);
        Serial.print(", ");
        Serial.print(estimate.theta_rad);
        Serial.print(") confidence: ");
        Serial.println(estimate.confidence);
        
        return true;
    }
    
    return false;
}

static void processEmagFrame(Robot* robot) {
    if (computePositionFromReadings(robot)) {
        Serial.println("Position update computed from electromagnet data");
    }
    
    for (uint8_t i = 0; i < EMAG_COUNT; i++) {
        emag_readings[i].count = 0;
    }
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

void PositionEstimator_Task(void* pvParameters) {
    Robot* robot = (Robot*)pvParameters;
    
    if (robot == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
    state_start_time = millis();
    current_state = STATE_IDLE;
    
    uint32_t last_sample_time = micros();
    
    while (1) {
        uint32_t current_time = millis();
        uint32_t current_micros = micros();
        uint32_t elapsed_ms = current_time - state_start_time;
        
        if ((current_micros - last_sample_time) >= EMAG_SAMPLE_PERIOD_US) {
            last_sample_time = current_micros;
            
            if (mag_initialized && mag.readMeasurement()) {
                float mx = mag.getFieldGaussX();
                float my = mag.getFieldGaussY();
                float mz = mag.getFieldGaussZ();
                
                switch (current_state) {
                    case STATE_IDLE:
                        avgX.add(mx);
                        avgY.add(my);
                        avgZ.add(mz);
                        
                        if (detectStartPulse(mx, my, mz, current_time)) {
                            current_state = STATE_WAITING_PAUSE;
                            state_start_time = current_time;
                            Serial.println("Start pulse pattern detected!");
                        }
                        break;
                        
                    case STATE_WAITING_PAUSE:
                        if (elapsed_ms >= EMAG_PAUSE_TIME_MS) {
                            current_state = STATE_START_PULSES;
                            state_start_time = current_time;
                            start_pulse_count = 0;
                            Serial.println("Detected start pulses");
                        }
                        break;
                        
                    case STATE_START_PULSES:
                        if (elapsed_ms >= EMAG_START_TOTAL_MS) {
                            current_state = STATE_MEASURING;
                            state_start_time = current_time;
                            current_emag_index = 0;
                            
                            for (uint8_t i = 0; i < EMAG_COUNT; i++) {
                                emag_readings[i].count = 0;
                            }
                            Serial.println("Starting measurement phase");
                        }
                        break;
                        
                    case STATE_MEASURING: {
                        uint32_t phase_time = elapsed_ms % (EMAG_ON_TIME_MS + EMAG_GAP_TIME_MS);
                        uint32_t total_phase = elapsed_ms / (EMAG_ON_TIME_MS + EMAG_GAP_TIME_MS);
                        
                        if (total_phase >= EMAG_COUNT) {
                            processEmagFrame(robot);
                            current_state = STATE_IDLE;
                            state_start_time = current_time;
                            Serial.println("Measurement frame complete");
                        } else if (phase_time < EMAG_ON_TIME_MS && total_phase == current_emag_index) {
                            if (emag_readings[current_emag_index].count < MAX_SAMPLES_PER_EMAG) {
                                uint16_t idx = emag_readings[current_emag_index].count;
                                emag_readings[current_emag_index].x[idx] = mx;
                                emag_readings[current_emag_index].y[idx] = my;
                                emag_readings[current_emag_index].z[idx] = mz;
                                emag_readings[current_emag_index].count++;
                            }
                        } else if (phase_time >= EMAG_ON_TIME_MS && total_phase > current_emag_index) {
                            current_emag_index = total_phase;
                        }
                        break;
                    }
                }
            }
        }
        
        delayMicroseconds(100);
    }
}
