#include "distortion_table.h"

// linear interpolation function
float DistortionTable::interpolate(const float x_in, const float x0, const float y0, const float x1, const float y1)
{
	if (fabs(x1 - x0) < std::numeric_limits<float>::epsilon())
	{
		return y0;
	}
	else
	{
		return ((x_in - x0) * (y1 - y0) / (x1 - x0)) + y0;
	}
}

float DistortionTable::getAngle(const float x, const float y, const float sensor_point_size_mm)
{
	float radius = sensor_point_size_mm * sqrtf((x * x) + (y * y));
	float alfa_grad = 0;

	for (uint8_t i = 1; i < DISTORTION_TABLE_SIZE; i++)
	{
		if (radius >= REAL_IMAGE_HEIGHT[i - 1] && radius <= REAL_IMAGE_HEIGHT[i])
		{
			alfa_grad = interpolate(radius, REAL_IMAGE_HEIGHT[i - 1], LENS_ANGLE[i - 1], REAL_IMAGE_HEIGHT[i], LENS_ANGLE[i]);
		}
	}

	return alfa_grad;
}

// centerpoint_offset : change centerpoint by using offset. (unit : pixel)
void DistortionTable::initLensTransform(const float sensor_point_size_mm, const uint8_t sensor_width, const uint8_t sensor_height,
									   const float center_point_offset_x, const float center_point_offset_y)
{
	int number_of_columns = sensor_width;
	int number_of_rows    = sensor_height;

	int row0 = 1 - (number_of_rows / 2)    + center_point_offset_x;
	int col0 = 1 - (number_of_columns / 2) + center_point_offset_y;

	for (y = 0, r = row0; y < number_of_rows; r++, y++)
	{
		max_check = 0.0f;

		for (x = 0, c = col0; x < number_of_columns; c++, x++)
		{
			float column = static_cast<float>(c) - 0.5f;
			float row    = static_cast<float>(r) - 0.5f;

			angle_grad = getAngle(column, row, sensor_point_size_mm);

			if (angle_grad > max_check) max_check = angle_grad;

			angle_rad = angle_grad * ROS_Const::DEGREE2RADIAN;

			float x_over_hypotenuse 			 = column * (sinf(angle_rad) / sqrtf((column * column) + (row * row)));
			float y_over_hypotenuse 			 = row    * (sinf(angle_rad) / sqrtf((column * column) + (row * row)));
			float focal_length_over_hypotenuse = cosf(angle_rad);

			table_x[x + (y * D2_Const::IMAGE_WIDTH)] = x_over_hypotenuse;
			table_y[x + (y * D2_Const::IMAGE_WIDTH)] = y_over_hypotenuse;
			table_z[x + (y * D2_Const::IMAGE_WIDTH)] = focal_length_over_hypotenuse;
		}
	}
}

void DistortionTable::transformPixel(uint16_t buffer_index, uint16_t source_origin_z,
									float& destination_x, float& destination_y, float& destination_z)
{
	destination_x = static_cast<float>(-source_origin_z) * table_x[buffer_index] * ROS_Const::MM2M;
	destination_y = static_cast<float>(-source_origin_z) * table_y[buffer_index] * ROS_Const::MM2M;
	destination_z = static_cast<float>( source_origin_z) * table_z[buffer_index] * ROS_Const::MM2M;
}
