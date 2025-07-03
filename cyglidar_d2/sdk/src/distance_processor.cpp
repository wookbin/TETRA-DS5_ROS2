#include "distance_processor.h"

// Convert 2D, 3D rawdata to measured distance appending both separated data (LSB and MSB)
void DistanceProcessor::getDistanceArray2D(uint8_t* received_buffer_2d, uint16_t* distance_2d)
{
    uint8_t buffer_count_2d = 0;

    _timestamp_mode_2d   = ((received_buffer_2d[D2_Const::TIMESTAMP_MSB]   & 0xFF) << 8) | received_buffer_2d[D2_Const::TIMESTAMP_LSB];
    _temperature_mode_2d = ((received_buffer_2d[D2_Const::TEMPERATURE_MSB] & 0xFF) << 8) | received_buffer_2d[D2_Const::TEMPERATURE_LSB];

    for (uint16_t data_length = 4; data_length < PACKET_LENGTH_2D; data_length += 2)
    {
        uint8_t data_lsb = received_buffer_2d[data_length];
        uint8_t data_msb = received_buffer_2d[data_length + 1];

        distance_2d[buffer_count_2d++] = static_cast<uint16_t>((data_msb << 8) | data_lsb);
    }
}

void DistanceProcessor::getDistanceArray3D(uint8_t* received_buffer_3d, uint16_t* distance_3d, bool enable_kalman_filter)
{
    uint16_t buffer_count_3d = 0;

    _timestamp_mode_3d   = ((received_buffer_3d[D2_Const::TIMESTAMP_MSB]   & 0xFF) << 8) | received_buffer_3d[D2_Const::TIMESTAMP_LSB];
    _temperature_mode_3d = ((received_buffer_3d[D2_Const::TEMPERATURE_MSB] & 0xFF) << 8) | received_buffer_3d[D2_Const::TEMPERATURE_LSB];

    for (uint16_t data_length = 4; data_length < PACKET_LENGTH_DISTANCE_3D; data_length += 3)
    {
        uint8_t first  = received_buffer_3d[data_length];
        uint8_t second = received_buffer_3d[data_length + 1];
        uint8_t third  = received_buffer_3d[data_length + 2];

        uint16_t data1 = (first << 4) | (second >> 4);
        uint16_t data2 = ((second & 0xf) << 8) | third;

		if (enable_kalman_filter)
		{
			distance_3d[buffer_count_3d++] = _kalman_filter.applyKalmanFiltering(buffer_count_3d, data1);
        	distance_3d[buffer_count_3d++] = _kalman_filter.applyKalmanFiltering(buffer_count_3d, data2);
		}
		else
		{
        	distance_3d[buffer_count_3d++] = data1;
        	distance_3d[buffer_count_3d++] = data2;
		}
    }
}

void DistanceProcessor::getDistanceAndAmpliutdeArray3D(uint8_t* received_buffer_3d, uint16_t* distance_3d, uint8_t* amplitude_3d, bool enable_kalman_filter)
{
    uint16_t buffer_count_3d = 0;
    uint16_t buffer_count_amplitude = 0;

    _timestamp_mode_3d   = ((received_buffer_3d[D2_Const::TIMESTAMP_MSB]   & 0xFF) << 8) | received_buffer_3d[D2_Const::TIMESTAMP_LSB];
    _temperature_mode_3d = ((received_buffer_3d[D2_Const::TEMPERATURE_MSB] & 0xFF) << 8) | received_buffer_3d[D2_Const::TEMPERATURE_LSB];

    for (uint16_t data_length = 4; data_length < PACKET_LENGTH_AMPLITUDE_3D; data_length += 5)
    {
        uint8_t first  = received_buffer_3d[data_length];
        uint8_t second = received_buffer_3d[data_length + 1];
        uint8_t third  = received_buffer_3d[data_length + 2];

        uint8_t amplitude1 = received_buffer_3d[data_length + 3];
        uint8_t amplitude2 = received_buffer_3d[data_length + 4];

        uint16_t data1 = (first << 4) | (second >> 4);
        uint16_t data2 = ((second & 0xf) << 8) | third;

        if (enable_kalman_filter)
		{
			distance_3d[buffer_count_3d++] = _kalman_filter.applyKalmanFiltering(buffer_count_3d, data1);
        	distance_3d[buffer_count_3d++] = _kalman_filter.applyKalmanFiltering(buffer_count_3d, data2);
		}
		else
		{
        	distance_3d[buffer_count_3d++] = data1;
        	distance_3d[buffer_count_3d++] = data2;
		}

        amplitude_3d[buffer_count_amplitude++] = amplitude1;
        amplitude_3d[buffer_count_amplitude++] = amplitude2;
    }
}

uint16_t DistanceProcessor::setTimeStamp2D()
{
    return this->_timestamp_mode_2d;
}

uint16_t DistanceProcessor::setTimeStamp3D()
{
    return this->_timestamp_mode_3d;
}

int16_t DistanceProcessor::setTemperature(uint8_t run_mode)
{
    int16_t sensor_temperature;

    if (run_mode == ROS_Const::MODE_2D)
    {
        sensor_temperature = _temperature_mode_2d;
    }
    else if (run_mode == ROS_Const::MODE_3D)
    {
        sensor_temperature = _temperature_mode_3d;
    }
    else
    {
        sensor_temperature = _temperature_mode_3d;
    }

    return sensor_temperature;
}
