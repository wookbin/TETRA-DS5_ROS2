#include "kalman_filtering.h"

KalmanFiltering::KalmanFiltering()
{
    initKalmanFilter();
}

KalmanFiltering::~KalmanFiltering()
{
    delete _kalman_buffer;

    _kalman_buffer = nullptr;
}

uint16_t KalmanFiltering::applyKalmanFiltering(uint16_t buffer_index, uint16_t raw_data)
{
    uint16_t result;

    if (raw_data < D2_Const::INTERFERENCE_3D)
    {
        uint16_t kalman_data = runKalmanFiltering(buffer_index, raw_data);

        if (abs(kalman_data - raw_data) < 40)
        {
            result = kalman_data;
        }
        else
        {
            reinitKalmanFilter(buffer_index, raw_data);
            result = raw_data;
        }
    }
    else
    {
        result = raw_data;
    }

    return result;
}

void KalmanFiltering::initKalmanFilter()
{
    _kalman_buffer = new KalmanValues_t[D2_Const::IMAGE_WIDTH * D2_Const::IMAGE_HEIGHT];

    clearKalmanBuffer();
}

void KalmanFiltering::reinitKalmanFilter(uint16_t index, uint16_t raw_data)
{
    _kalman_buffer[index].x = raw_data;
}

uint16_t KalmanFiltering::runKalmanFiltering(uint16_t index, uint16_t raw_data)
{
    float z  = static_cast<float>(raw_data);
    float xp =  A * _kalman_buffer[index].x;
    float Pp = (A * _kalman_buffer[index].P * TRANSPOSE_OF_A) + Q;

    _kalman_buffer[index].x = xp + (_kalman_buffer[index].K * (z - (H * xp)));
    _kalman_buffer[index].K = (Pp * TRANSPOSE_OF_H) / ((H * Pp * TRANSPOSE_OF_H) + R);
    _kalman_buffer[index].P = Pp - (_kalman_buffer[index].K * H * Pp);

    return static_cast<uint16_t>(_kalman_buffer[index].x);
}

void KalmanFiltering::clearKalmanBuffer()
{
    for (uint16_t i = 0; i < D2_Const::IMAGE_WIDTH * D2_Const::IMAGE_HEIGHT; i++)
    {
        _kalman_buffer[i].x = 0.0;
        _kalman_buffer[i].K = 0.0;
        _kalman_buffer[i].P = 20.0;
    }
}
