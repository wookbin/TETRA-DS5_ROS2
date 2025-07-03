#pragma once

#include <cstdint>

typedef struct ColorCodeRGB
{
	uint8_t R;
	uint8_t G;
	uint8_t B;
	uint8_t A;
}ColorCode_t;

namespace ROS_Const
{
	const uint8_t MODE_2D        = 0;
	const uint8_t MODE_3D        = 1;
	const uint8_t MODE_DUAL      = 2;

	const uint8_t MODE_HUE       = 0;
	const uint8_t MODE_RGB       = 1;
	const uint8_t MODE_GRAY      = 2;

	const uint8_t MODE_DISTANCE  = 0;
	const uint8_t MODE_AMPLITUDE = 1;

	const uint8_t NONE_FILTER    = 0;
	const uint8_t MEDIAN_FILTER  = 1;
	const uint8_t AVERAGE_FILTER = 2;

	const uint8_t PUBLISH_DONE   = 0;
	const uint8_t PUBLISH_2D     = 1;
	const uint8_t PUBLISH_3D     = 2;

	const uint8_t PULSE_AUTO     = 0;
	const uint8_t PULSE_MANUAL   = 1;

	const float PI            = 3.14159265f;
	const float MM2M          = 0.001f;
	const float DEGREE2RADIAN = PI / 180.0f;

	const float PIXEL_REAL_SIZE       = 0.02f;	// unit : mm
	const float OFFSET_CENTER_POINT_X = 0.0f;	// unit : pixel
	const float OFFSET_CENTER_POINT_Y = 0.0f;	// unit : pixel

	const ColorCode_t ADC_OVERFLOW_COLOR = { 0xAD, 0xD8, 0xE6, 0xFF };
	const ColorCode_t SATURATION_COLOR   = { 0x80, 0x00, 0x80, 0xFF };
	const ColorCode_t NONE_COLOR     	 = { 0x00, 0x00, 0x00, 0x00 };
}

namespace D2_Const
{
	const uint8_t  IMAGE_WIDTH  = 160;	// unit : pixel
	const uint8_t  IMAGE_HEIGHT = 60;	// unit : pixel

	const uint16_t INTEGRATION_MAX_VALUE = 10000;	// unit : us

	const float HORIZONTAL_ANGLE   = 120.0f;	// unit : degree
	const float ANGLE_INCREMENT_2D = 0.75f;		// unit : degree

	//---> 3D Depth Code
	const uint16_t DISTANCE_MAX_VALUE_3D = 4000;	// unit : mm

	const uint16_t INVALID_DATA_3D  = 4080;
	const uint16_t LOW_AMPLITUDE_3D = 4081;
	const uint16_t ADC_OVERFLOW_3D  = 4082;
	const uint16_t SATURATION_3D    = 4083;
	const uint16_t BAD_PIXEL_3D     = 4084;
	const uint16_t LOW_DCS_3D 		= 4085;
	const uint16_t INTERFERENCE_3D  = 4086;
	const uint16_t EDGE_FILTERED_3D = 4087;

	//---> 2D Depth Code
	const uint16_t DISTANCE_MAX_VALUE_2D = 10000; // unit : mm
	const uint16_t DISTANCE_MIN_VALUE_2D = 100;   // unit : mm

	const uint16_t INVALID_DATA_2D  = 16000;
	const uint16_t LOW_AMPLITUDE_2D = 16001;
	const uint16_t ADC_OVERFLOW_2D  = 16002;
	const uint16_t SATURATION_2D    = 16003;
	const uint16_t BAD_PIXEL_2D     = 16004;
	const uint16_t LOW_DCS_2D       = 16005;
	const uint16_t INTERFERENCE_2D  = 16007;
	const uint16_t EDGE_FILTERED_2D = 16008;

	//---> CygLiDAR D2's PACKET COMMAND
	const uint8_t NORMAL_MODE  = 0x5A;
	const uint8_t PRODUCT_CODE = 0x77;
	const uint8_t DEFAULT_ID   = 0xFF;

	const uint8_t DEVICE_INFO         = 0x10;
	const uint8_t SEND_DEPTH_2D       = 0x01;
	const uint8_t SEND_DEPTH_3D       = 0x08;
	const uint8_t SEND_DEPTH_DUAL     = 0x07;
	const uint8_t SWITCH_3D_DATA_TYPE = 0x15;
	const uint8_t DISTNACE_MODE       = 0x00;
	const uint8_t AMPLITUDE_MODE      = 0x01;
	const uint8_t SEND_STOP           = 0x02;
	const uint8_t INTEGRATION_TIME    = 0x0C;
	const uint8_t FREQUENCY_CHANNEL   = 0x0F;
	const uint8_t SET_BAUDRATE        = 0x12;
	const uint8_t SET_NEW_FILTERING   = 0x13;
	const uint8_t SET_EDGE_FILTERING  = 0x14;
	const uint8_t SET_DEVICE_ID 	  = 0x20;
	const uint8_t GET_DEVICE_ID  	  = 0x21;
	const uint8_t COMMAND_DATA        = 0x00;

	//---> Payload header value = 2D / 3D / Sensor Version(F/W, H/W)
	const uint8_t PACKET_HEADER_2D           = 0x01;
	const uint8_t PACKET_HEADER_3D           = 0x08;
	const uint8_t PACKET_HEADER_AMPLITUDE_3D = 0x88;
	const uint8_t PACKET_HEADER_DEVICE_INFO  = 0x10;
	const uint8_t PACKET_HEADER_DEVICE_ID	 = 0x21;

	const uint8_t HEADER_START	 = 0;
	const uint8_t HEADER_DEVICE	 = 1;
	const uint8_t HEADER_ID		 = 2;
	const uint8_t LENGTH_LSB	 = 3;
	const uint8_t LENGTH_MSB	 = 4;
	const uint8_t PAYLOAD_HEADER = 5;
	const uint8_t PAYLOAD_INDEX  = 6;

	const uint8_t TIMESTAMP_LSB   = 0;
	const uint8_t TIMESTAMP_MSB   = 1;
	const uint8_t TEMPERATURE_LSB = 2;
	const uint8_t TEMPERATURE_MSB = 3;

	//---> After parsing return value
	const uint8_t PARSING_FAILED  = 0x00;
	const uint8_t CHECKSUM_PASSED = 0x01;
	const uint8_t PARSING_STARTED = 0x02;

	const uint16_t SCAN_MAX_SIZE = 25000;
}
