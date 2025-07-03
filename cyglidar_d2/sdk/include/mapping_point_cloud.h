#pragma once

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv4/opencv2/core/mat.hpp>

#include "cygbot_constant.h"
#include "distortion_table.h"

using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

class MappingPointCloud
{
    public:
        MappingPointCloud();
        virtual ~MappingPointCloud();

        void applyPointCloud3DColors(std::shared_ptr<pcl_XYZRGBA>& pcl_3d, uint16_t* distance_buffer_3d);
        void applyAmplitudePointCloud(std::shared_ptr<pcl_XYZRGBA>& pcl_3d, uint16_t* distance_buffer_3d, cv::Mat& amplitude_matrix);

        void getColorMap(std::vector<ColorCode_t>& color_map);

    private:
        DistortionTable* _distortion_table;

        std::vector<ColorCode_t> _color_map;

        uint16_t _total_color_number;

        float _color_gap;
        float _real_world_coordinate_x;
        float _real_world_coordinate_y;
        float _real_world_coordinate_z;
};
