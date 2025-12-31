#ifndef POSITION_LUT_H
#define POSITION_LUT_H

#include <stdint.h>

#define LUT_GRID_X_SIZE   20
#define LUT_GRID_Y_SIZE   20
#define LUT_GRID_THETA_SIZE 1

#define LUT_X_MIN_MM      0.0f
#define LUT_X_MAX_MM      500.0f
#define LUT_Y_MIN_MM      0.0f
#define LUT_Y_MAX_MM      500.0f

#define EMAG_AXES         6

struct PositionLUTEntry {
    float x_mm;
    float y_mm;
    float field_intensities[EMAG_AXES];
};

struct PositionEstimate {
    float x_mm;
    float y_mm;
    float theta_rad;
    float confidence;
};

bool PositionLUT_ComputePosition(const float measured_intensities[EMAG_AXES], PositionEstimate* result);

void PositionLUT_RunBenchmark();

#endif
