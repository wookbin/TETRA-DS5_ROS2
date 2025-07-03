#include "lidar_3d_topic.h"

Lidar3dTopic::Lidar3dTopic()
{
    _cyg_pcl    = new MappingPointCloud();
    _cyg_opencv = new ColorEncodedDepthAmplitude();
    _message_point_cloud_3d = std::make_shared<PointCloud2>();
    _message_image 		    = std::make_shared<Image>();
}

Lidar3dTopic::~Lidar3dTopic()
{
	delete _cyg_pcl;
	delete _cyg_opencv;

	_cyg_pcl    = nullptr;
	_cyg_opencv = nullptr;
}

void Lidar3dTopic::initPublisher(rclcpp::Publisher<Image>::SharedPtr publisher_image, rclcpp::Publisher<PointCloud2>::SharedPtr publisher_point_3d)
{
    _publisher_image    = publisher_image;
    _publisher_point_3d = publisher_point_3d;
}

void Lidar3dTopic::assignPCL3D(const std::string& frame_id)
{
    _pcl_3d.reset(new pcl_XYZRGBA());

    _pcl_3d->header.frame_id = frame_id;
    _pcl_3d->is_dense        = false;
    _pcl_3d->width           = D2_Const::IMAGE_WIDTH;
    _pcl_3d->height          = D2_Const::IMAGE_HEIGHT;
    _pcl_3d->points.resize(DATA_LENGTH_3D);
}

void Lidar3dTopic::assignImage(const std::string& frame_id)
{
    _message_image->header.frame_id = frame_id;
    _message_image->width           = D2_Const::IMAGE_WIDTH;
    _message_image->height          = D2_Const::IMAGE_HEIGHT;
    _message_image->encoding        = sensor_msgs::image_encodings::BGRA8;
    _message_image->step            = _message_image->width * sizeof(uint32_t);
    _message_image->is_bigendian    = false;
    _message_image->data.resize(_message_image->height * _message_image->step);
}

void Lidar3dTopic::publishDepthFlatImage(rclcpp::Time scan_start_time, uint16_t* distance_buffer_3d)
{
    _message_image->header.stamp = scan_start_time;

	memcpy(_message_image->data.data(), _cyg_opencv->applyDepthFlatImage(distance_buffer_3d).data, _message_image->height * _message_image->width * sizeof(uint32_t));

    _publisher_image->publish(*_message_image);
}

void Lidar3dTopic::publishAmplitudeFlatImage(rclcpp::Time scan_start_time, uint16_t* distance_buffer_3d)
{
    _message_image->header.stamp = scan_start_time;

	if (_enable_clahe)
	{
		_processed_image = _cyg_opencv->applyAmplitudeFlatImage(distance_buffer_3d, _cyg_opencv->matrix_clahe_applied_amplitude);
		memcpy(_message_image->data.data(), _processed_image.data, _message_image->height * _message_image->width * sizeof(uint32_t));
	}
	else
	{
		_processed_image = _cyg_opencv->applyAmplitudeFlatImage(distance_buffer_3d, _cyg_opencv->matrix_raw_amplitude);
		memcpy(_message_image->data.data(), _processed_image.data, _message_image->height * _message_image->width * sizeof(uint32_t));
	}

    _publisher_image->publish(*_message_image);
}

void Lidar3dTopic::publishDepthPointCloud3D(rclcpp::Time scan_start_time, uint16_t* distance_buffer_3d)
{
    pcl_conversions::toPCL(scan_start_time, _pcl_3d->header.stamp);

    _cyg_pcl->applyPointCloud3DColors(_pcl_3d, distance_buffer_3d);

    pcl::toROSMsg(*_pcl_3d, *_message_point_cloud_3d); //change type from pointcloud(pcl) to ROS message(PointCloud2)
    _publisher_point_3d->publish(*_message_point_cloud_3d);
}

void Lidar3dTopic::publishAmplitudePointCloud3D(rclcpp::Time scan_start_time, uint16_t* distance_buffer_3d)
{
    pcl_conversions::toPCL(scan_start_time, _pcl_3d->header.stamp);

	if (_enable_clahe)
	{
		_cyg_pcl->applyAmplitudePointCloud(_pcl_3d, distance_buffer_3d, _cyg_opencv->matrix_clahe_applied_amplitude);
	}
	else
	{
		_cyg_pcl->applyAmplitudePointCloud(_pcl_3d, distance_buffer_3d, _cyg_opencv->matrix_raw_amplitude);
	}

    pcl::toROSMsg(*_pcl_3d, *_message_point_cloud_3d);
    _publisher_point_3d->publish(*_message_point_cloud_3d);
}

void Lidar3dTopic::updateColorConfig(uint8_t color_mode, std::string& notice)
{
    this->_color_mode = color_mode;

    // Call the following function so as to store colors to draw 3D data
    initColorMap();

    if (_color_mode == ROS_Const::MODE_HUE)
    {
        notice = "HUE MODE";
    }
    else if (_color_mode == ROS_Const::MODE_RGB)
    {
        notice = "RGB MODE";
    }
    else if (_color_mode == ROS_Const::MODE_GRAY)
    {
        notice = "GRAY MODE";
    }

    _cyg_pcl->getColorMap(_color_map);
    _cyg_opencv->getColorMap(_color_map);
}

void Lidar3dTopic::checkAmplitudeStatus(bool enable_clahe, uint8_t clip_limit, uint8_t tiles_grid_size, uint8_t* amplitude_data)
{
	this->_enable_clahe = enable_clahe;

	for (uint16_t i = 0; i < D2_Const::IMAGE_WIDTH * D2_Const::IMAGE_HEIGHT; i++)
    {
        _cyg_opencv->matrix_raw_amplitude.at<uint8_t>(0, i) = amplitude_data[i];
    }

	if (enable_clahe)
	{
		_cyg_opencv->applyCLAHE(clip_limit, tiles_grid_size);
	}
}

void Lidar3dTopic::initColorMap()
{
	uint8_t r_setup = 0;
	uint8_t g_setup = 0;
	uint8_t b_setup = 0;

	uint8_t color_array = 0;

	if (_color_mode == ROS_Const::MODE_HUE)
	{
		color_array = 5;
		b_setup = 255;
	}
	else if (_color_mode == ROS_Const::MODE_RGB)
	{
		color_array = 2;
		b_setup = 255;
	}
	else if (_color_mode == ROS_Const::MODE_GRAY)
	{
		color_array = 1;
		b_setup = 0;
	}

    // Iterate for-loop of adding RGB value to an array
	for (uint8_t i = 0; i < color_array; i++)
	{
		for (uint8_t color_count = 0; color_count < 255; color_count++)
		{
			if (_color_mode == ROS_Const::MODE_GRAY)
			{
				switch (i)
				{
					case 0:
						r_setup++;
						g_setup++;
						b_setup++;
						break;
				}
			}
			else
			{
				switch (i)
				{
					case 0: // BLUE -> YELLOW
					case 3:
						r_setup++;
						g_setup++;
						b_setup--;
						break;
					case 1: // YELLOW -> RED
					case 4:
						g_setup--;
						break;
					case 2: // RED -> BLUE
						r_setup--;
						b_setup++;
						break;
				}
			}

			_color_map.push_back({r_setup, g_setup, b_setup, 0xFF});
		}
	}
}
