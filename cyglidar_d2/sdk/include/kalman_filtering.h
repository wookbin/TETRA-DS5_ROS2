#pragma once

#include <cstdint>
#include <cmath>

#include "cygbot_constant.h"

typedef struct KalmanFilterValues {
    float x;
    float K;
    float P;
} KalmanValues_t;

class KalmanFiltering
{
    const float A = 1.0;
    const float H = 1.0;
    const float Q = 50.0;
    const float R = 200.0;
    const float TRANSPOSE_OF_A = 1.0;
    const float TRANSPOSE_OF_H = 1.0;

    public:
        KalmanFiltering();
        ~KalmanFiltering();

        uint16_t applyKalmanFiltering(uint16_t buffer_index, uint16_t raw_data);

    private:
        void initKalmanFilter();
        void reinitKalmanFilter(uint16_t index, uint16_t raw_data);
        void clearKalmanBuffer();
        uint16_t runKalmanFiltering(uint16_t index, uint16_t raw_data);

        KalmanValues_t* _kalman_buffer;
};
