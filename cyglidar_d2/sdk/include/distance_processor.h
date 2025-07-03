#pragma once

#include <cstdio>
#include <cstdint>
#include <vector>

#include "cygbot_constant.h"
#include "kalman_filtering.h"

const uint16_t DATA_LENGTH_2D = 161;
const uint16_t DATA_LENGTH_3D = D2_Const::IMAGE_WIDTH * D2_Const::IMAGE_HEIGHT;

const uint16_t PACKET_LENGTH_2D = DATA_LENGTH_2D * sizeof(uint16_t) + sizeof(uint8_t) * 4;
const uint16_t PACKET_LENGTH_DISTANCE_3D  = DATA_LENGTH_3D * 1.5 + sizeof(uint8_t) * 4;
const uint16_t PACKET_LENGTH_AMPLITUDE_3D = DATA_LENGTH_3D * 1.5 + DATA_LENGTH_3D * 1 + sizeof(uint8_t) * 4;

class DistanceProcessor
{
    public:
        void getDistanceArray2D(uint8_t* received_buffer_2d, uint16_t* distance_2d);
        void getDistanceArray3D(uint8_t* received_buffer_3d, uint16_t* distance_3d, bool enable_kalman_filter);
        void getDistanceAndAmpliutdeArray3D(uint8_t* received_buffer_3d, uint16_t* distance_3d, uint8_t* amplitude_3d, bool enable_kalman_filter);

        uint16_t setTimeStamp2D();
        uint16_t setTimeStamp3D();
        int16_t  setTemperature(uint8_t run_mode);

    private:
		KalmanFiltering _kalman_filter;

        uint16_t _timestamp_mode_2d;
        uint16_t _timestamp_mode_3d;
        int16_t  _temperature_mode_2d;
        int16_t  _temperature_mode_3d;
};
