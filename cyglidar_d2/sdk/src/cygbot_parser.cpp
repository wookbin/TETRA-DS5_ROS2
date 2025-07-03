#include "cygbot_parser.h"

uint8_t CygbotParser::CygParser(uint8_t* command_buffer, uint8_t packet_data)
{
	switch (_packet_check_list)
	{
		case kHeader1:
			if (packet_data == D2_Const::NORMAL_MODE)
			{
				command_buffer[D2_Const::HEADER_START] = D2_Const::NORMAL_MODE;
				_command_mode = kNormalMode;
				_packet_check_list = kHeader2;

				return D2_Const::PARSING_STARTED;
			}
			break;
		case kHeader2:
			if (packet_data == D2_Const::PRODUCT_CODE && _command_mode == kNormalMode)
			{
				command_buffer[D2_Const::HEADER_DEVICE] = D2_Const::PRODUCT_CODE;
				_packet_check_list = kHeader3;
			}
			else
			{
				initPacket(packet_data);
			}
			break;
		case kHeader3:
			command_buffer[D2_Const::HEADER_ID] = packet_data;
			_packet_check_list = kLength_LSB;
			break;
		case kLength_LSB:
			command_buffer[D2_Const::LENGTH_LSB] = packet_data;
			_packet_check_list = kLength_MSB;
			break;
		case kLength_MSB:
			command_buffer[D2_Const::LENGTH_MSB] = packet_data;
			_payload_size = ((command_buffer[D2_Const::LENGTH_MSB] << 8) & 0xFF00) | (command_buffer[D2_Const::LENGTH_LSB] & 0x00FF);
			_packet_check_list = kPayload_Header;
			_payload_count = 0;
			break;
		case kPayload_Header:
			command_buffer[kPayload_Header] = packet_data;
			_packet_check_list = kPayload_Data;
			_payload_count++;
			break;
		case kPayload_Data:
			command_buffer[kPayload_Header + _payload_count] = packet_data;
			_payload_count++;

			if (_payload_count > _payload_size)
			{
				initPacket(packet_data); // packet overflow
				return D2_Const::PARSING_FAILED;
			}

			if (_payload_size == _payload_count)
			{
				_packet_check_list = kCheckSum;
				_payload_count = 0;
			}
			break;
		case kCheckSum:
		{
			initPacket(0);

			uint8_t check_sum_byte = calcCheckSum(command_buffer, _payload_size + 6);

			if (check_sum_byte == packet_data)
			{
				command_buffer[D2_Const::PAYLOAD_HEADER + _payload_size] = packet_data;
				return D2_Const::CHECKSUM_PASSED;
				// Succeed packet parsing
			}

			break;
		}
		default:
			initPacket(0);
			break;
	}

	return 0;
}

void CygbotParser::initPacket(uint8_t packet_data)
{
	if (packet_data == D2_Const::NORMAL_MODE)
	{
		_command_mode = kNormalMode;
		_packet_check_list = kHeader2;
	}
	else
	{
		_command_mode = kIdleMode;
		_packet_check_list = kHeader1;
	}
}

//buffer_size = packet total byte (Header1 + Header2 + Header3 + Length1 + Length2 + Payload(Command) + Checksum)
uint8_t CygbotParser::calcCheckSum(uint8_t* buffer, uint16_t buffer_size)
{
	uint8_t sum = 0;

	for (uint16_t i = D2_Const::LENGTH_LSB; i < buffer_size - 1; i++)
	{
		sum ^= buffer[i];
	}

	return sum;
}
