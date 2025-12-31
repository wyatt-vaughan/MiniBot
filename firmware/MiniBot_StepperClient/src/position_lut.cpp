#include "position_lut.h"
#include <Arduino.h>
#include <math.h>
#include <float.h>

// Hardcoded LUT: 20x20 grid from (0,0) to (500,500) mm
// Only stores 0° rotation - rotation extracted during matching
// Format: {x_mm, y_mm, {emag0, emag1, emag2, emag3, emag4, emag5}}
// Replace these placeholder values with actual calibrated measurements
static const PositionLUTEntry position_lut[LUT_GRID_X_SIZE * LUT_GRID_Y_SIZE] = {
    // Row 0: Y = 0.0mm
    {0.0f, 0.0f, {1.0f, 1.2f, 1.5f, 2.0f, 1.8f, 1.3f}},
    {26.3f, 0.0f, {1.1f, 1.3f, 1.6f, 1.9f, 1.7f, 1.2f}},
    {52.6f, 0.0f, {1.2f, 1.4f, 1.7f, 1.8f, 1.6f, 1.1f}},
    {78.9f, 0.0f, {1.3f, 1.5f, 1.8f, 1.7f, 1.5f, 1.0f}},
    {105.3f, 0.0f, {1.4f, 1.6f, 1.9f, 1.6f, 1.4f, 0.9f}},
    {131.6f, 0.0f, {1.5f, 1.7f, 2.0f, 1.5f, 1.3f, 0.8f}},
    {157.9f, 0.0f, {1.6f, 1.8f, 2.1f, 1.4f, 1.2f, 0.7f}},
    {184.2f, 0.0f, {1.7f, 1.9f, 2.2f, 1.3f, 1.1f, 0.6f}},
    {210.5f, 0.0f, {1.8f, 2.0f, 2.3f, 1.2f, 1.0f, 0.5f}},
    {236.8f, 0.0f, {1.9f, 2.1f, 2.4f, 1.1f, 0.9f, 0.4f}},
    {263.2f, 0.0f, {2.0f, 2.2f, 2.5f, 1.0f, 0.8f, 0.3f}},
    {289.5f, 0.0f, {1.9f, 2.1f, 2.4f, 1.1f, 0.9f, 0.4f}},
    {315.8f, 0.0f, {1.8f, 2.0f, 2.3f, 1.2f, 1.0f, 0.5f}},
    {342.1f, 0.0f, {1.7f, 1.9f, 2.2f, 1.3f, 1.1f, 0.6f}},
    {368.4f, 0.0f, {1.6f, 1.8f, 2.1f, 1.4f, 1.2f, 0.7f}},
    {394.7f, 0.0f, {1.5f, 1.7f, 2.0f, 1.5f, 1.3f, 0.8f}},
    {421.1f, 0.0f, {1.4f, 1.6f, 1.9f, 1.6f, 1.4f, 0.9f}},
    {447.4f, 0.0f, {1.3f, 1.5f, 1.8f, 1.7f, 1.5f, 1.0f}},
    {473.7f, 0.0f, {1.2f, 1.4f, 1.7f, 1.8f, 1.6f, 1.1f}},
    {500.0f, 0.0f, {1.1f, 1.3f, 1.6f, 1.9f, 1.7f, 1.2f}},
    
    // Row 1: Y = 26.3mm
    {0.0f, 26.3f, {1.0f, 1.1f, 1.4f, 1.9f, 1.9f, 1.4f}},
    {26.3f, 26.3f, {1.1f, 1.2f, 1.5f, 1.8f, 1.8f, 1.3f}},
    {52.6f, 26.3f, {1.2f, 1.3f, 1.6f, 1.7f, 1.7f, 1.2f}},
    {78.9f, 26.3f, {1.3f, 1.4f, 1.7f, 1.6f, 1.6f, 1.1f}},
    {105.3f, 26.3f, {1.4f, 1.5f, 1.8f, 1.5f, 1.5f, 1.0f}},
    {131.6f, 26.3f, {1.5f, 1.6f, 1.9f, 1.4f, 1.4f, 0.9f}},
    {157.9f, 26.3f, {1.6f, 1.7f, 2.0f, 1.3f, 1.3f, 0.8f}},
    {184.2f, 26.3f, {1.7f, 1.8f, 2.1f, 1.2f, 1.2f, 0.7f}},
    {210.5f, 26.3f, {1.8f, 1.9f, 2.2f, 1.1f, 1.1f, 0.6f}},
    {236.8f, 26.3f, {1.9f, 2.0f, 2.3f, 1.0f, 1.0f, 0.5f}},
    {263.2f, 26.3f, {2.0f, 2.1f, 2.4f, 0.9f, 0.9f, 0.4f}},
    {289.5f, 26.3f, {1.9f, 2.0f, 2.3f, 1.0f, 1.0f, 0.5f}},
    {315.8f, 26.3f, {1.8f, 1.9f, 2.2f, 1.1f, 1.1f, 0.6f}},
    {342.1f, 26.3f, {1.7f, 1.8f, 2.1f, 1.2f, 1.2f, 0.7f}},
    {368.4f, 26.3f, {1.6f, 1.7f, 2.0f, 1.3f, 1.3f, 0.8f}},
    {394.7f, 26.3f, {1.5f, 1.6f, 1.9f, 1.4f, 1.4f, 0.9f}},
    {421.1f, 26.3f, {1.4f, 1.5f, 1.8f, 1.5f, 1.5f, 1.0f}},
    {447.4f, 26.3f, {1.3f, 1.4f, 1.7f, 1.6f, 1.6f, 1.1f}},
    {473.7f, 26.3f, {1.2f, 1.3f, 1.6f, 1.7f, 1.7f, 1.2f}},
    {500.0f, 26.3f, {1.1f, 1.2f, 1.5f, 1.8f, 1.8f, 1.3f}},
    
    // Rows 2-18: TODO - Add remaining 360 entries
    // Pattern continues with Y incrementing by 26.3mm each row
    // For now, using symmetric pattern as placeholder
    
    // Row 19: Y = 500.0mm
    {0.0f, 500.0f, {2.0f, 1.8f, 1.3f, 1.0f, 1.2f, 1.5f}},
    {26.3f, 500.0f, {1.9f, 1.7f, 1.2f, 1.1f, 1.3f, 1.6f}},
    {52.6f, 500.0f, {1.8f, 1.6f, 1.1f, 1.2f, 1.4f, 1.7f}},
    {78.9f, 500.0f, {1.7f, 1.5f, 1.0f, 1.3f, 1.5f, 1.8f}},
    {105.3f, 500.0f, {1.6f, 1.4f, 0.9f, 1.4f, 1.6f, 1.9f}},
    {131.6f, 500.0f, {1.5f, 1.3f, 0.8f, 1.5f, 1.7f, 2.0f}},
    {157.9f, 500.0f, {1.4f, 1.2f, 0.7f, 1.6f, 1.8f, 2.1f}},
    {184.2f, 500.0f, {1.3f, 1.1f, 0.6f, 1.7f, 1.9f, 2.2f}},
    {210.5f, 500.0f, {1.2f, 1.0f, 0.5f, 1.8f, 2.0f, 2.3f}},
    {236.8f, 500.0f, {1.1f, 0.9f, 0.4f, 1.9f, 2.1f, 2.4f}},
    {263.2f, 500.0f, {1.0f, 0.8f, 0.3f, 2.0f, 2.2f, 2.5f}},
    {289.5f, 500.0f, {1.1f, 0.9f, 0.4f, 1.9f, 2.1f, 2.4f}},
    {315.8f, 500.0f, {1.2f, 1.0f, 0.5f, 1.8f, 2.0f, 2.3f}},
    {342.1f, 500.0f, {1.3f, 1.1f, 0.6f, 1.7f, 1.9f, 2.2f}},
    {368.4f, 500.0f, {1.4f, 1.2f, 0.7f, 1.6f, 1.8f, 2.1f}},
    {394.7f, 500.0f, {1.5f, 1.3f, 0.8f, 1.5f, 1.7f, 2.0f}},
    {421.1f, 500.0f, {1.6f, 1.4f, 0.9f, 1.4f, 1.6f, 1.9f}},
    {447.4f, 500.0f, {1.7f, 1.5f, 1.0f, 1.3f, 1.5f, 1.8f}},
    {473.7f, 500.0f, {1.8f, 1.6f, 1.1f, 1.2f, 1.4f, 1.7f}},
    {500.0f, 500.0f, {1.9f, 1.7f, 1.2f, 1.1f, 1.3f, 1.6f}},
};

static const uint16_t LUT_SIZE = sizeof(position_lut) / sizeof(PositionLUTEntry);

static float computeRotationInvariantDistance(const float measured[EMAG_AXES], 
                                               const float reference[EMAG_AXES],
                                               float* rotation_offset) {
    float min_distance = FLT_MAX;
    float best_rotation = 0.0f;
    
    // Try all possible rotational alignments (6 positions for 6 emags)
    for (int rot_step = 0; rot_step < EMAG_AXES; rot_step++) {
        float dist_sum = 0.0f;
        
        for (uint8_t i = 0; i < EMAG_AXES; i++) {
            int rotated_idx = (i + rot_step) % EMAG_AXES;
            float diff = measured[rotated_idx] - reference[i];
            dist_sum += diff * diff;
        }
        
        if (dist_sum < min_distance) {
            min_distance = dist_sum;
            best_rotation = (2.0f * 3.14159265359f * rot_step) / EMAG_AXES;
        }
    }
    
    if (rotation_offset != NULL) {
        *rotation_offset = best_rotation;
    }
    
    return sqrtf(min_distance);
}

static void findNearestNeighbors(const float measured[EMAG_AXES], 
                                  uint16_t* indices, 
                                  float* distances,
                                  float* rotations,
                                  uint8_t k) {
    for (uint8_t i = 0; i < k; i++) {
        indices[i] = 0;
        distances[i] = FLT_MAX;
        rotations[i] = 0.0f;
    }
    
    for (uint16_t lut_idx = 0; lut_idx < LUT_SIZE; lut_idx++) {
        float rotation = 0.0f;
        float dist = computeRotationInvariantDistance(measured, 
                                                       position_lut[lut_idx].field_intensities,
                                                       &rotation);
        
        for (uint8_t i = 0; i < k; i++) {
            if (dist < distances[i]) {
                for (uint8_t j = k - 1; j > i; j--) {
                    distances[j] = distances[j - 1];
                    indices[j] = indices[j - 1];
                    rotations[j] = rotations[j - 1];
                }
                distances[i] = dist;
                indices[i] = lut_idx;
                rotations[i] = rotation;
                break;
            }
        }
    }
}

static float wrapAngle(float angle) {
    while (angle > 3.14159265359f) angle -= 6.28318530718f;
    while (angle < -3.14159265359f) angle += 6.28318530718f;
    return angle;
}

bool PositionLUT_ComputePosition(const float measured_intensities[EMAG_AXES], PositionEstimate* result) {
    if (result == NULL) {
        return false;
    }
    
    const uint8_t K_NEIGHBORS = 4;
    uint16_t neighbor_indices[K_NEIGHBORS];
    float neighbor_distances[K_NEIGHBORS];
    float neighbor_rotations[K_NEIGHBORS];
    
    findNearestNeighbors(measured_intensities, neighbor_indices, neighbor_distances, neighbor_rotations, K_NEIGHBORS);
    
    if (neighbor_distances[0] == FLT_MAX) {
        return false;
    }
    
    float total_weight = 0.0f;
    float weighted_x = 0.0f;
    float weighted_y = 0.0f;
    float weighted_sin_theta = 0.0f;
    float weighted_cos_theta = 0.0f;
    
    for (uint8_t i = 0; i < K_NEIGHBORS; i++) {
        if (neighbor_distances[i] == FLT_MAX) {
            break;
        }
        
        float weight = 1.0f / (neighbor_distances[i] + 0.001f);
        total_weight += weight;
        
        const PositionLUTEntry* entry = &position_lut[neighbor_indices[i]];
        weighted_x += weight * entry->x_mm;
        weighted_y += weight * entry->y_mm;
        // Use detected rotation offset for orientation
        weighted_sin_theta += weight * sinf(neighbor_rotations[i]);
        weighted_cos_theta += weight * cosf(neighbor_rotations[i]);
    }
    
    if (total_weight > 0.0f) {
        result->x_mm = weighted_x / total_weight;
        result->y_mm = weighted_y / total_weight;
        result->theta_rad = atan2f(weighted_sin_theta / total_weight, weighted_cos_theta / total_weight);
        result->confidence = 1.0f / (1.0f + neighbor_distances[0]);
    } else {
        result->x_mm = 0.0f;
        result->y_mm = 0.0f;
        result->theta_rad = 0.0f;
        result->confidence = 0.0f;
        return false;
    }
    
    return true;
}

void PositionLUT_RunBenchmark() {
    Serial.println("\n===== Position LUT Benchmark =====");
    Serial.print("LUT Size: ");
    Serial.print(LUT_SIZE);
    Serial.println(" entries");
    Serial.print("LUT Memory: ");
    Serial.print(sizeof(position_lut));
    Serial.println(" bytes");
    
    const uint16_t NUM_TESTS = 50;
    uint32_t total_time_us = 0;
    uint32_t min_time_us = UINT32_MAX;
    uint32_t max_time_us = 0;
    uint16_t successful_lookups = 0;
    
    // Test with various intensity patterns
    for (uint16_t test = 0; test < NUM_TESTS; test++) {
        // Generate test pattern based on LUT entry
        uint16_t lut_test_idx = (test * LUT_SIZE) / NUM_TESTS;
        float test_intensities[EMAG_AXES];
        
        // Use LUT entry and add small noise
        for (uint8_t i = 0; i < EMAG_AXES; i++) {
            test_intensities[i] = position_lut[lut_test_idx].field_intensities[i];
            // Add 5% random variation
            float noise = (float)(random(-50, 50)) / 1000.0f;
            test_intensities[i] *= (1.0f + noise);
        }
        
        PositionEstimate result;
        uint32_t start_time = micros();
        bool success = PositionLUT_ComputePosition(test_intensities, &result);
        uint32_t elapsed = micros() - start_time;
        
        if (success) {
            successful_lookups++;
            total_time_us += elapsed;
            if (elapsed < min_time_us) min_time_us = elapsed;
            if (elapsed > max_time_us) max_time_us = elapsed;
            
            // Print first few results for verification
            if (test < 3) {
                Serial.print("Test ");
                Serial.print(test);
                Serial.print(": (");
                Serial.print(result.x_mm, 1);
                Serial.print(", ");
                Serial.print(result.y_mm, 1);
                Serial.print(", ");
                Serial.print(result.theta_rad, 2);
                Serial.print(") conf=");
                Serial.print(result.confidence, 3);
                Serial.print(" in ");
                Serial.print(elapsed);
                Serial.println("us");
            }
        }
    }
    
    Serial.println("\n----- Benchmark Results -----");
    Serial.print("Successful lookups: ");
    Serial.print(successful_lookups);
    Serial.print("/");
    Serial.println(NUM_TESTS);
    
    if (successful_lookups > 0) {
        Serial.print("Average time: ");
        Serial.print(total_time_us / successful_lookups);
        Serial.println(" us");
        
        Serial.print("Min time: ");
        Serial.print(min_time_us);
        Serial.println(" us");
        
        Serial.print("Max time: ");
        Serial.print(max_time_us);
        Serial.println(" us");
        
        float lookups_per_sec = 1000000.0f / (float)(total_time_us / successful_lookups);
        Serial.print("Throughput: ");
        Serial.print(lookups_per_sec, 1);
        Serial.println(" lookups/sec");
    }
    
    Serial.println("=================================\n");
}
