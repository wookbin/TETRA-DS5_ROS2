#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "cygbot_constant.h"
#include "distance_processor.h"
#include "mapping_point_cloud.h"
#include "color_encoded_depth_amplitude.h"

using Image = sensor_msgs::msg::Image;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using pcl_XYZRGBA = pcl::PointCloud<pcl::PointXYZRGBA>;

class Lidar3dTopic
{
    public:
        Lidar3dTopic();
        virtual ~Lidar3dTopic();

        void initPublisher(rclcpp::Publisher<Image>::SharedPtr publisher_image, rclcpp::Publisher<PointCloud2>::SharedPtr publisher_point_3d);

        void assignImage(const std::string& frame_id);
        void assignPCL3D(const std::string& frame_id);

        void publishDepthFlatImage(rclcpp::Time scan_start_time, uint16_t* distance_buffer_3d);
        void publishAmplitudeFlatImage(rclcpp::Time scan_start_time, uint16_t* distance_buffer_3d);
        void publishDepthPointCloud3D(rclcpp::Time scan_start_time, uint16_t* distance_buffer_3d);
        void publishAmplitudePointCloud3D(rclcpp::Time scan_start_time, uint16_t* distance_buffer_3d);

        void checkAmplitudeStatus(bool enable_clahe, uint8_t clip_limit, uint8_t tiles_grid_size, uint8_t* amplitude_buffer);
        void updateColorConfig(uint8_t color_mode, std::string& notice);

    private:
		void initColorMap();

        MappingPointCloud*   _cyg_pcl;
        ColorEncodedDepthAmplitude* _cyg_opencv;

        rclcpp::Publisher<Image>::SharedPtr       _publisher_image;
        rclcpp::Publisher<PointCloud2>::SharedPtr _publisher_point_3d;

        std::shared_ptr<Image>       _message_image;
        std::shared_ptr<PointCloud2> _message_point_cloud_3d;
        std::shared_ptr<pcl_XYZRGBA> _pcl_3d;

        std::vector<ColorCode_t> _color_map;
        cv::Mat _processed_image;

        uint8_t _color_mode;
        bool _enable_clahe;
};
