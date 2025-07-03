#pragma once

#include <boost/asio.hpp>
#include <string>
#include <iostream>
#include <vector>

#include "cygbot_constant.h"

class SerialUart
{
    public:
        SerialUart();
        virtual ~SerialUart();

        void openSerialPort(const std::string& port, const uint8_t baudrate);
        void closeSerialPort();

        uint16_t getPacketLength(uint8_t* received_buffer);

        void requestRunMode(const uint8_t run_mode, std::string& notice);
        void requestSwitch3DType(const uint8_t mode_3d, std::string& notice);
        void requestDurationControl(const uint8_t run_mode, const uint8_t duration_mode, uint16_t duration_value);
        void requestFrequencyChannel(const uint8_t channel_number);
        void requestSerialBaudRate(const uint8_t select_baud_rate);
        void requestNewFiltering(const uint8_t run_mode, const uint8_t filter_mode, std::string& notice);
        void requestEdgeFiltering(const uint8_t run_mode, uint16_t edge_filter_value);
        void requestDeviceInfo();
        void requestGetDeviceId();

        uint32_t getBaudRate(uint8_t baud_rate_mode);

    private:
        void transferPacketCommand(const std::vector<uint8_t>& payload);

        std::shared_ptr<boost::asio::serial_port> _serial_port;
        boost::asio::io_service   _io_service;
        boost::system::error_code _error_code;

        std::vector<uint8_t> _command_buffer;
        std::vector<uint8_t> _payload_buffer;
};
