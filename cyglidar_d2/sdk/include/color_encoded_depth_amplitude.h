#pragma once

#include <opencv4/opencv2/opencv.hpp>

#include "cygbot_constant.h"

class ColorEncodedDepthAmplitude
{
    public:
        void applyCLAHE(uint8_t clip_limit, uint8_t tiles_grid_size);

        cv::Mat applyDepthFlatImage(uint16_t* distance_buffer_3d);
        cv::Mat applyAmplitudeFlatImage(uint16_t* distance_buffer_3d, cv::Mat& amplitude_matrix);

        void getColorMap(std::vector<ColorCode_t>& color_map);

        cv::Mat matrix_raw_amplitude           = cv::Mat::zeros(1, D2_Const::IMAGE_WIDTH * D2_Const::IMAGE_HEIGHT, CV_8UC1);
        cv::Mat matrix_clahe_applied_amplitude = cv::Mat::zeros(1, D2_Const::IMAGE_WIDTH * D2_Const::IMAGE_HEIGHT, CV_8UC1);

    private:
        cv::Mat _matrix_color = cv::Mat::zeros(D2_Const::IMAGE_HEIGHT, D2_Const::IMAGE_WIDTH, CV_8UC4);
        cv::Ptr<cv::CLAHE> _clahe;

        std::vector<ColorCode_t> _color_map;

        uint16_t _total_color_number;
        float _color_gap;
};
