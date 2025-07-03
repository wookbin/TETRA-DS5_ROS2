#include "lidar_2d_topic.h"

void Lidar2dTopic::initPublisher(rclcpp::Publisher<LaserScan>::SharedPtr publisher_laserscan, rclcpp::Publisher<PointCloud2>::SharedPtr publisher_point_2d)
{
    _publisher_laserscan = publisher_laserscan;
    _publisher_point_2d  = publisher_point_2d;
}

void Lidar2dTopic::applyPointCloud2D(uint16_t* distance_buffer_2d)
{
    uint8_t buffer_index = 0;

    float point_2d_angle = 0;
    float point_2d_angle_variable = 0;

    for (uint8_t i = 0; i < DATA_LENGTH_2D; i++)
    {
        buffer_index = (DATA_LENGTH_2D - 1 - i);

        uint16_t raw_distance = distance_buffer_2d[buffer_index];

        point_2d_angle = ((-D2_Const::HORIZONTAL_ANGLE / 2) + point_2d_angle_variable) * ROS_Const::DEGREE2RADIAN;
        point_2d_angle_variable += D2_Const::ANGLE_INCREMENT_2D;

        float real_world_coordinate_x = (sin(point_2d_angle) * raw_distance);
        float real_world_coordinate_y = (cos(point_2d_angle) * raw_distance);

        _pcl_2d->points[i].x = real_world_coordinate_y * ROS_Const::MM2M;
        _pcl_2d->points[i].y = real_world_coordinate_x * ROS_Const::MM2M;
        _pcl_2d->points[i].z = 0.0;

        if (distance_buffer_2d[buffer_index] < D2_Const::DISTANCE_MAX_VALUE_2D)
        {
            _pcl_2d->points[i].rgba = 0xFFFFFF00; //ARGB(Yellow)
        }
        else
        {
            _pcl_2d->points[i].a = 0; // Turn data invisible when it's greater than the maximum
        }
    }
}

void Lidar2dTopic::assignPCL2D(const std::string& frame_id)
{
    _message_point_cloud_2d = std::make_shared<PointCloud2>();
    _pcl_2d.reset(new pcl_XYZRGBA());

    _pcl_2d->header.frame_id = frame_id;
    _pcl_2d->is_dense        = false;
    _pcl_2d->points.resize(DATA_LENGTH_2D);
}

void Lidar2dTopic::publishPoint2D(rclcpp::Time start_time)
{
    pcl_conversions::toPCL(start_time, _pcl_2d->header.stamp);

    pcl::toROSMsg(*_pcl_2d, *_message_point_cloud_2d); //change type from pointcloud(pcl) to ROS message(PointCloud2)
    _publisher_point_2d->publish(*_message_point_cloud_2d);
}

void Lidar2dTopic::assignLaserScan(const std::string& frame_id)
{
    _message_laserscan = std::make_shared<LaserScan>();

    _message_laserscan->header.frame_id = frame_id;
    _message_laserscan->angle_min       = -D2_Const::HORIZONTAL_ANGLE / 2.0f * ROS_Const::DEGREE2RADIAN;
    _message_laserscan->angle_max       =  D2_Const::HORIZONTAL_ANGLE / 2.0f * ROS_Const::DEGREE2RADIAN;
    _message_laserscan->angle_increment =  D2_Const::ANGLE_INCREMENT_2D * ROS_Const::DEGREE2RADIAN;
    _message_laserscan->scan_time       = 0;
    _message_laserscan->range_min       = D2_Const::DISTANCE_MIN_VALUE_2D * ROS_Const::MM2M;
    _message_laserscan->range_max       = D2_Const::DISTANCE_MAX_VALUE_2D * ROS_Const::MM2M;
    _message_laserscan->ranges.resize(DATA_LENGTH_2D);
    _message_laserscan->intensities.resize(DATA_LENGTH_2D);
}

void Lidar2dTopic::publishScanLaser(rclcpp::Time start_time, uint16_t* distance_buffer_2d)
{
    uint8_t buffer_index = 0;

    _message_laserscan->header.stamp = start_time;

    for (uint8_t i = 0; i < DATA_LENGTH_2D; i++)
    {
        // Reverse data order of the array
        buffer_index = (DATA_LENGTH_2D - 1 - i);

        if (distance_buffer_2d[buffer_index] < D2_Const::DISTANCE_MAX_VALUE_2D)
        {
            _message_laserscan->ranges[i] = distance_buffer_2d[buffer_index] * ROS_Const::MM2M;
        }
        else
        {
            _message_laserscan->ranges[i] = std::numeric_limits<float>::infinity();
        }
    }

    _publisher_laserscan->publish(*_message_laserscan);
}

