#include "mapping_point_cloud.h"

MappingPointCloud::MappingPointCloud()
{
    _distortion_table = new DistortionTable();

    _distortion_table->initLensTransform(ROS_Const::PIXEL_REAL_SIZE, D2_Const::IMAGE_WIDTH, D2_Const::IMAGE_HEIGHT,
                                       ROS_Const::OFFSET_CENTER_POINT_X, ROS_Const::OFFSET_CENTER_POINT_Y);
}

MappingPointCloud::~MappingPointCloud()
{
    delete _distortion_table;
    _distortion_table = nullptr;
}

void MappingPointCloud::applyPointCloud3DColors(std::shared_ptr<pcl_XYZRGBA>& pcl_3d, uint16_t* distance_buffer_3d)
{
    for (uint16_t buffer_index = 0; buffer_index < D2_Const::IMAGE_HEIGHT * D2_Const::IMAGE_WIDTH; buffer_index++)
    {
        uint16_t raw_distance = distance_buffer_3d[buffer_index];

        if(raw_distance < D2_Const::DISTANCE_MAX_VALUE_3D)
        {
            _distortion_table->transformPixel(buffer_index, raw_distance, _real_world_coordinate_y, _real_world_coordinate_z, _real_world_coordinate_x);

            pcl_3d->points[buffer_index].x = _real_world_coordinate_x;
            pcl_3d->points[buffer_index].y = _real_world_coordinate_y;
            pcl_3d->points[buffer_index].z = _real_world_coordinate_z;

            uint16_t color_level = (int)((float)raw_distance / _color_gap) >= _total_color_number ? (_total_color_number - 1) : (int)raw_distance / _color_gap;

            uint32_t rgb_setup = ((uint32_t)_color_map[color_level].R << 16
                                | (uint32_t)_color_map[color_level].G << 8
                                | (uint32_t)_color_map[color_level].B);

            pcl_3d->points[buffer_index].rgb = *reinterpret_cast<float *>(&rgb_setup);
            pcl_3d->points[buffer_index].a = _color_map[color_level].A;
        }
        else
        {
            pcl_3d->points[buffer_index].x = 0;
            pcl_3d->points[buffer_index].y = 0;
            pcl_3d->points[buffer_index].z = 0;
            pcl_3d->points[buffer_index].a = 0;
        }
    }
}

void MappingPointCloud::applyAmplitudePointCloud(std::shared_ptr<pcl_XYZRGBA>& pcl_3d, uint16_t* distance_buffer_3d, cv::Mat& amplitude_matrix)
{
    for (uint16_t buffer_index = 0; buffer_index < D2_Const::IMAGE_HEIGHT * D2_Const::IMAGE_WIDTH; buffer_index++)
    {
        uint16_t raw_distance = distance_buffer_3d[buffer_index];

        if(raw_distance < D2_Const::DISTANCE_MAX_VALUE_3D)
        {
            _distortion_table->transformPixel(buffer_index, raw_distance, _real_world_coordinate_y, _real_world_coordinate_z, _real_world_coordinate_x);

            pcl_3d->points[buffer_index].x = _real_world_coordinate_x;
            pcl_3d->points[buffer_index].y = _real_world_coordinate_y;
            pcl_3d->points[buffer_index].z = _real_world_coordinate_z;

            uint32_t rgb_setup = ((uint32_t)amplitude_matrix.at<uint8_t>(0, buffer_index) << 16
                                | (uint32_t)amplitude_matrix.at<uint8_t>(0, buffer_index) << 8
                                | (uint32_t)amplitude_matrix.at<uint8_t>(0, buffer_index));

            pcl_3d->points[buffer_index].rgb = *reinterpret_cast<float *>(&rgb_setup);
            pcl_3d->points[buffer_index].a = 0xFF;
        }
        else
        {
            pcl_3d->points[buffer_index].x = 0;
            pcl_3d->points[buffer_index].y = 0;
            pcl_3d->points[buffer_index].z = 0;
            pcl_3d->points[buffer_index].a = 0;
        }
    }
}

void MappingPointCloud::getColorMap(std::vector<ColorCode_t>& color_map)
{
    this->_color_map = color_map;

    this->_total_color_number = _color_map.size();
    this->_color_gap = D2_Const::DISTANCE_MAX_VALUE_3D / _total_color_number;
}
