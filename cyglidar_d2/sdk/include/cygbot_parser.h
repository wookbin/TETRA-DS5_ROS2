#pragma once

#include <cstdint>

#include "cygbot_constant.h"

class CygbotParser
{
    enum eCommandMode
	{
		kIdleMode = 0,
		kNormalMode
	};

	enum ePacketCheckList
	{
		kHeader1 = 0,
		kHeader2,
		kHeader3,
		kLength_LSB,
		kLength_MSB,
		kPayload_Header,
		kPayload_Data,
		kCheckSum
	};

    public:
        uint8_t CygParser(uint8_t* command_buffer, uint8_t packet_data);

    private:
        void initPacket(uint8_t packet_data);
		uint8_t calcCheckSum(uint8_t* buffer, uint16_t buffer_size);

		enum eCommandMode     _command_mode      = kIdleMode;
		enum ePacketCheckList _packet_check_list = kHeader1;

		uint16_t _payload_count;
		uint16_t _payload_size;
};
