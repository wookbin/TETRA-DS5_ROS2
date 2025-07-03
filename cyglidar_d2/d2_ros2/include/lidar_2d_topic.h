#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "cygbot_constant.h"
#include "distance_processor.h"

using LaserScan = sensor_msgs::msg::LaserScan;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

class Lidar2dTopic
{
    public:
        void initPublisher(rclcpp::Publisher<LaserScan>::SharedPtr publisher_laserscan, rclcpp::Publisher<PointCloud2>::SharedPtr publisher_point_2d);

        void applyPointCloud2D(uint16_t* distance_buffer_2d);

        void assignPCL2D(const std::string& frame_id);
        void publishPoint2D(rclcpp::Time start_time);

        void assignLaserScan(const std::string& frame_id);
        void publishScanLaser(rclcpp::Time start_time, uint16_t* distance_buffer_2d);

    private:
        rclcpp::Publisher<LaserScan>::SharedPtr   _publisher_laserscan;
        rclcpp::Publisher<PointCloud2>::SharedPtr _publisher_point_2d;

        std::shared_ptr<LaserScan>   _message_laserscan;
        std::shared_ptr<PointCloud2> _message_point_cloud_2d;
        std::shared_ptr<pcl_XYZRGBA> _pcl_2d;
};
