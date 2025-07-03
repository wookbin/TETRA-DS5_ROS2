#include "color_encoded_depth_amplitude.h"

void ColorEncodedDepthAmplitude::applyCLAHE(uint8_t clip_limit, uint8_t tiles_grid_size)
{
    _clahe = cv::createCLAHE();
    _clahe->setClipLimit(clip_limit);
    _clahe->setTilesGridSize(cv::Size(tiles_grid_size, tiles_grid_size));

    _clahe->apply(matrix_raw_amplitude, matrix_clahe_applied_amplitude);
}

cv::Mat ColorEncodedDepthAmplitude::applyDepthFlatImage(uint16_t* distance_buffer_3d)
{
    uint16_t buffer_index = 0;

    for (uint8_t y = 0; y < D2_Const::IMAGE_HEIGHT; y++)
    {
        for (uint8_t x = 0; x < D2_Const::IMAGE_WIDTH; x++)
        {
            buffer_index = x + (y * D2_Const::IMAGE_WIDTH);

            uint16_t raw_distance = distance_buffer_3d[buffer_index];
            uint16_t color_level = (int)((float)raw_distance / _color_gap) >= _total_color_number ? (_total_color_number - 1) : (int)raw_distance / _color_gap;

            if (raw_distance < D2_Const::DISTANCE_MAX_VALUE_3D)
            {
                _matrix_color.at<cv::Vec4b>(y, x)[0] = _color_map[color_level].B;
                _matrix_color.at<cv::Vec4b>(y, x)[1] = _color_map[color_level].G;
                _matrix_color.at<cv::Vec4b>(y, x)[2] = _color_map[color_level].R;
                _matrix_color.at<cv::Vec4b>(y, x)[3] = _color_map[color_level].A;
            }
            else if (raw_distance == D2_Const::ADC_OVERFLOW_3D)
            {
                _matrix_color.at<cv::Vec4b>(y, x)[0] = ROS_Const::ADC_OVERFLOW_COLOR.B;
                _matrix_color.at<cv::Vec4b>(y, x)[1] = ROS_Const::ADC_OVERFLOW_COLOR.G;
                _matrix_color.at<cv::Vec4b>(y, x)[2] = ROS_Const::ADC_OVERFLOW_COLOR.R;
                _matrix_color.at<cv::Vec4b>(y, x)[3] = ROS_Const::ADC_OVERFLOW_COLOR.A;
            }
            else if (raw_distance == D2_Const::SATURATION_3D)
            {
                _matrix_color.at<cv::Vec4b>(y, x)[0] = ROS_Const::SATURATION_COLOR.B;
                _matrix_color.at<cv::Vec4b>(y, x)[1] = ROS_Const::SATURATION_COLOR.G;
                _matrix_color.at<cv::Vec4b>(y, x)[2] = ROS_Const::SATURATION_COLOR.R;
                _matrix_color.at<cv::Vec4b>(y, x)[3] = ROS_Const::SATURATION_COLOR.A;
            }
            else
            {
                _matrix_color.at<cv::Vec4b>(y, x)[0] = ROS_Const::NONE_COLOR.B;
                _matrix_color.at<cv::Vec4b>(y, x)[1] = ROS_Const::NONE_COLOR.G;
                _matrix_color.at<cv::Vec4b>(y, x)[2] = ROS_Const::NONE_COLOR.R;
                _matrix_color.at<cv::Vec4b>(y, x)[3] = ROS_Const::NONE_COLOR.A;
            }
        }
    }

    return _matrix_color;
}

cv::Mat ColorEncodedDepthAmplitude::applyAmplitudeFlatImage(uint16_t* distance_buffer_3d, cv::Mat& amplitude_matrix)
{
    uint16_t buffer_index = 0;

    for (uint8_t y = 0; y < D2_Const::IMAGE_HEIGHT; y++)
    {
        for (uint8_t x = 0; x < D2_Const::IMAGE_WIDTH; x++)
        {
            buffer_index = x + (y * D2_Const::IMAGE_WIDTH);

            uint16_t raw_distance = distance_buffer_3d[buffer_index];

            if (raw_distance < D2_Const::DISTANCE_MAX_VALUE_3D)
            {
                _matrix_color.at<cv::Vec4b>(y, x)[0] = amplitude_matrix.at<uint8_t>(0, buffer_index);
                _matrix_color.at<cv::Vec4b>(y, x)[1] = amplitude_matrix.at<uint8_t>(0, buffer_index);
                _matrix_color.at<cv::Vec4b>(y, x)[2] = amplitude_matrix.at<uint8_t>(0, buffer_index);
                _matrix_color.at<cv::Vec4b>(y, x)[3] = amplitude_matrix.at<uint8_t>(0, buffer_index);
            }
            else if (raw_distance == D2_Const::ADC_OVERFLOW_3D)
            {
                _matrix_color.at<cv::Vec4b>(y, x)[0] = ROS_Const::ADC_OVERFLOW_COLOR.B;
                _matrix_color.at<cv::Vec4b>(y, x)[1] = ROS_Const::ADC_OVERFLOW_COLOR.G;
                _matrix_color.at<cv::Vec4b>(y, x)[2] = ROS_Const::ADC_OVERFLOW_COLOR.R;
                _matrix_color.at<cv::Vec4b>(y, x)[3] = ROS_Const::ADC_OVERFLOW_COLOR.A;
            }
            else if (raw_distance == D2_Const::SATURATION_3D)
            {
                _matrix_color.at<cv::Vec4b>(y, x)[0] = ROS_Const::SATURATION_COLOR.B;
                _matrix_color.at<cv::Vec4b>(y, x)[1] = ROS_Const::SATURATION_COLOR.G;
                _matrix_color.at<cv::Vec4b>(y, x)[2] = ROS_Const::SATURATION_COLOR.R;
                _matrix_color.at<cv::Vec4b>(y, x)[3] = ROS_Const::SATURATION_COLOR.A;
            }
            else
            {
                _matrix_color.at<cv::Vec4b>(y, x)[0] = ROS_Const::NONE_COLOR.B;
                _matrix_color.at<cv::Vec4b>(y, x)[1] = ROS_Const::NONE_COLOR.G;
                _matrix_color.at<cv::Vec4b>(y, x)[2] = ROS_Const::NONE_COLOR.R;
                _matrix_color.at<cv::Vec4b>(y, x)[3] = ROS_Const::NONE_COLOR.A;
            }
        }
    }

    return _matrix_color;
}

void ColorEncodedDepthAmplitude::getColorMap(std::vector<ColorCode_t>& color_map)
{
    this->_color_map = color_map;

    this->_total_color_number = _color_map.size();
    this->_color_gap = D2_Const::DISTANCE_MAX_VALUE_3D / _total_color_number;
}

