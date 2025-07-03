#include "serial_uart.h"

SerialUart::SerialUart() {}

SerialUart::~SerialUart()
{
    if (_serial_port)
	{
		_serial_port->cancel();
		_serial_port->close();
		_serial_port.reset();
	}

	_io_service.stop();
	_io_service.reset();
}

void SerialUart::openSerialPort(const std::string& port, const uint8_t baudrate)
{
	if (_serial_port)
	{
		std::cout << "[ERRPR] PORT IS ALREADY OPENED..." << std::endl;
	}

    _serial_port = std::make_shared<boost::asio::serial_port>(_io_service);

	_serial_port->open(port, _error_code);

	if (_error_code)
	{
		std::cout << "[BOOST SERIAL ERROR] TRIED TO CONNECT WITH \"port=" << port
				  << "\", " << _error_code.message().c_str() << std::endl;
	}

    uint32_t baud_rate = getBaudRate(baudrate);

	_serial_port->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	_serial_port->set_option(boost::asio::serial_port_base::character_size(8));
	_serial_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	_serial_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	_serial_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
}

uint16_t SerialUart::getPacketLength(uint8_t* received_buffer)
{
    if (_serial_port.get() == NULL || !_serial_port->is_open()) return 0;

    uint16_t number_of_packet = _serial_port->read_some(boost::asio::buffer(received_buffer, D2_Const::SCAN_MAX_SIZE), _error_code);

	if (_error_code) return 0;

    return number_of_packet;
}

void SerialUart::requestRunMode(const uint8_t run_mode, std::string& notice)
{
    _payload_buffer.clear();

    switch (run_mode)
    {
        case ROS_Const::MODE_2D:
            _payload_buffer.push_back(D2_Const::SEND_DEPTH_2D);
            notice = "RUN 2D MODE";
            break;
        case ROS_Const::MODE_3D:
            _payload_buffer.push_back(D2_Const::SEND_DEPTH_3D);
            notice = "RUN 3D MODE";
            break;
        case ROS_Const::MODE_DUAL:
            _payload_buffer.push_back(D2_Const::SEND_DEPTH_DUAL);
            notice = "RUN DUAL MODE";
            break;
    }
    _payload_buffer.push_back(D2_Const::COMMAND_DATA);

    transferPacketCommand(_payload_buffer);
}

void SerialUart::requestSwitch3DType(const uint8_t mode_3d, std::string& notice)
{
    _payload_buffer.clear();
    _payload_buffer.push_back(D2_Const::SWITCH_3D_DATA_TYPE);

    if (mode_3d == ROS_Const::MODE_DISTANCE)
    {
        _payload_buffer.push_back(ROS_Const::MODE_DISTANCE);
        notice = "SWITCH TO DISTANCE TYPE";
    }
    else if (mode_3d == ROS_Const::MODE_AMPLITUDE)
    {
        _payload_buffer.push_back(ROS_Const::MODE_AMPLITUDE);
        notice = "SWITCH TO DISTANCE & AMPLITUDE TYPE";
    }

    transferPacketCommand(_payload_buffer);
}

void SerialUart::requestDurationControl(const uint8_t _run_mode, const uint8_t _duration_mode, uint16_t _duration_value)
{
    _payload_buffer.clear();

    if(_run_mode == ROS_Const::MODE_2D || _duration_value > D2_Const::INTEGRATION_MAX_VALUE)
    {
        return;
    }

    uint8_t msb = (_duration_value & 0xFF00) >> 8;
    uint8_t lsb =  _duration_value & 0x00FF;

    if(_duration_mode == ROS_Const::PULSE_MANUAL)
    {
        msb |= (1 << 6); // 2nd bit, set Fixed(1) or Auto
    }

    _payload_buffer.push_back(D2_Const::INTEGRATION_TIME);
    _payload_buffer.push_back(lsb);
    _payload_buffer.push_back(msb);

    transferPacketCommand(_payload_buffer);
}

void SerialUart::requestFrequencyChannel(const uint8_t channel_number)
{
    _payload_buffer.clear();
    _payload_buffer.push_back(D2_Const::FREQUENCY_CHANNEL);
    _payload_buffer.push_back(channel_number);

    transferPacketCommand(_payload_buffer);
}

void SerialUart::requestDeviceInfo()
{
    _payload_buffer.clear();
    _payload_buffer.push_back(D2_Const::DEVICE_INFO);
    _payload_buffer.push_back(D2_Const::COMMAND_DATA);

    transferPacketCommand(_payload_buffer);
}

void SerialUart::requestSerialBaudRate(const uint8_t select_baud_rate)
{
    _payload_buffer.clear();
    _payload_buffer.push_back(D2_Const::SET_BAUDRATE);
    _payload_buffer.push_back(select_baud_rate);

    transferPacketCommand(_payload_buffer);
}

void SerialUart::requestNewFiltering(const uint8_t run_mode, const uint8_t filter_mode, std::string& notice)
{
    if(run_mode == ROS_Const::MODE_2D)
    {
        return;
    }

    _payload_buffer.clear();
    _payload_buffer.push_back(D2_Const::SET_NEW_FILTERING);

    if (filter_mode == ROS_Const::NONE_FILTER)
    {
        _payload_buffer.push_back(filter_mode);
        notice = "NONE FILTER APPLIED";
    }
    else if (filter_mode == ROS_Const::MEDIAN_FILTER)
    {
        _payload_buffer.push_back(filter_mode);
        notice = "MEDIAN FILTER APPLIED";
    }
    else if (filter_mode == ROS_Const::AVERAGE_FILTER)
    {
        _payload_buffer.push_back(filter_mode);
        notice = "AVERAGE FILTER APPLIED";
    }

    transferPacketCommand(_payload_buffer);
}

void SerialUart::requestEdgeFiltering(const uint8_t _run_mode, uint16_t _edge_filter_value)
{
    _payload_buffer.clear();

    if(_run_mode == ROS_Const::MODE_2D)
    {
        return;
    }

    uint8_t msb = (_edge_filter_value & 0xFF00) >> 8;
    uint8_t lsb =  _edge_filter_value & 0x00FF;

    _payload_buffer.push_back(D2_Const::SET_EDGE_FILTERING);
    _payload_buffer.push_back(lsb);
    _payload_buffer.push_back(msb);

    transferPacketCommand(_payload_buffer);
}

void SerialUart::requestGetDeviceId()
{
    _payload_buffer.clear();

    _payload_buffer.push_back(D2_Const::GET_DEVICE_ID);
    _payload_buffer.push_back(D2_Const::COMMAND_DATA);

    transferPacketCommand(_payload_buffer);
}

uint32_t SerialUart::getBaudRate(uint8_t baud_rate_mode)
{
    uint32_t result = 0;

    switch(baud_rate_mode)
    {
        case 0:
            result = 3000000;
            break;
        case 1:
            result = 921600;
            break;
        case 2:
            result = 115200;
            break;
        case 3:
            result = 57600;
            break;
    }

    return result;
}

void SerialUart::closeSerialPort()
{
    _payload_buffer.clear();
    _payload_buffer.push_back(D2_Const::SEND_STOP);
    _payload_buffer.push_back(D2_Const::COMMAND_DATA);

    transferPacketCommand(_payload_buffer);
}

void SerialUart::transferPacketCommand(const std::vector<uint8_t>& payload)
{
    uint8_t check_sum = 0;

    _command_buffer.clear();
    _command_buffer.push_back(D2_Const::NORMAL_MODE);
    _command_buffer.push_back(D2_Const::PRODUCT_CODE);
    _command_buffer.push_back(D2_Const::DEFAULT_ID);

    _command_buffer.push_back(payload.size());
    check_sum ^= payload.size();

    _command_buffer.push_back(0x00);
    check_sum ^= 0x00;

    for(uint8_t i = 0; i < payload.size(); i++)
    {
        uint8_t buffer_index = D2_Const::PAYLOAD_HEADER + i;

        _command_buffer.push_back(payload[i]);
        check_sum ^= _command_buffer[buffer_index];
    }

    _command_buffer.push_back(check_sum);

    _serial_port->write_some(boost::asio::buffer(_command_buffer, _command_buffer.size()), _error_code);
}
