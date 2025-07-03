#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "cygbot_constant.h"

class DeviceStatusTopic
{
    public:
        void initPublisher(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_device_status);
        void assignDeviceStatus();
        void getTemperature(int16_t temperature);

        void publishDeviceStatus();

    private:
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _publisher_device_status;
        std::shared_ptr<std_msgs::msg::Float32>              _message_temperature;

        float _sensor_temperature;
};
