#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "cygbot_constant.h"
#include "cygbot_parser.h"
#include "distance_processor.h"
#include "serial_uart.h"
#include "distortion_table.h"
#include "lidar_2d_topic.h"
#include "lidar_3d_topic.h"
#include "device_status_topic.h"

using namespace std::chrono_literals;

class D2Node : public rclcpp::Node
{
    public:
        explicit D2Node();
        virtual ~D2Node();

        void connectBoostSerial();
        void disconnectBoostSerial();
        void loopCygParser();

    private:
        struct received_data_buffer
        {
            rclcpp::Time parsing_start_time;
            rclcpp::Time parsing_end_time;
            uint8_t* packet_data;
        }received_buffer[2];

        void initConfiguration();
        void requestPacketData();
        void convertData(received_data_buffer* _received_buffer);
        void processDoubleBuffer();
        void runPublish();
        void doublebufferThread();
        void publishThread();

        Lidar2dTopic*      topic_2d;
        Lidar3dTopic*      topic_3d;
        DeviceStatusTopic* status_topic;
        SerialUart*        serial_uart;
        DistanceProcessor* distance_processor;
        CygbotParser*      cygbot_parser;

        rclcpp::Time start_time_scan_2d;
        rclcpp::Time start_time_scan_3d;

        std::string port_number;
        uint8_t     baud_rate_mode;
        std::string frame_id;
        uint8_t     run_mode;
        uint8_t     data_type_3d;
        uint8_t     duration_mode;
        uint16_t    duration_value;
        uint8_t     frequency_channel;
        uint8_t     color_mode;
        uint8_t     filter_mode;
        uint16_t    edge_filter_value;
        bool        enable_kalmanfilter;
        bool        enable_clahe;
        uint8_t     clahe_cliplimit;
        uint8_t     clahe_tiles_grid_size;

        std::thread double_buffer_thread;
        std::thread publish_thread;

        std::shared_future<void> future;
        std::promise<void> exit_signal;
        std::future_status status;

        std::string mode_notice;

        uint8_t packet_structure[D2_Const::SCAN_MAX_SIZE];
        uint8_t first_total_packet_data[D2_Const::SCAN_MAX_SIZE];
        uint8_t second_total_packet_data[D2_Const::SCAN_MAX_SIZE];
        uint8_t amplitude_buffer_3d[D2_Const::IMAGE_WIDTH * D2_Const::IMAGE_HEIGHT];
        uint16_t distance_buffer_2d[DATA_LENGTH_2D];
        uint16_t distance_buffer_3d[DATA_LENGTH_3D];

        uint8_t  publish_done_flag;
        uint8_t  publish_data_state;
        uint8_t  double_buffer_index;
};
