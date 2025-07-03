/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, laser_filters authors
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*
\author Yannic Bachmann
*/

#ifndef LASER_SCAN_MEDIAN_SPATIAL_FILTER_H
#define LASER_SCAN_MEDIAN_SPATIAL_FILTER_H

#include "filters/filter_base.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <algorithm>
#include <vector>
#include <stdexcept>
#include <limits>
#include <cmath>

namespace laser_filters
{

class LaserScanMedianSpatialFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan>
{
private:
  int window_size_;

public:

  bool configure()
  {
    getParam("window_size", window_size_);

    // Ensure window size is positive
    if (window_size_ <= 0)
    {
      RCLCPP_ERROR(logging_interface_->get_logger(), "Window size must be positive.\n");
      return false;
    }

    // Ensure window size is odd
    // If the window_size is even, the current range, which will be modified cannot be centered in the window.
    if (window_size_ % 2 == 0)
    {
      RCLCPP_WARN(logging_interface_->get_logger(), "Window size must be odd. Automatically setting window_size to %d instead of %d.\n", window_size_+1, window_size_);
      window_size_ += 1;
    }

    return true;
  }

  virtual ~LaserScanMedianSpatialFilter() {}

  bool update(const sensor_msgs::msg::LaserScan &input_scan, sensor_msgs::msg::LaserScan &filtered_scan)
  {
    filtered_scan = input_scan;

    int half_window = window_size_ / 2;
    std::vector<float> valid_values;
    int nan_count = 0;
    int neg_inf_count = 0;
    int pos_inf_count = 0;

    for (size_t current_beam_index = 0; current_beam_index < input_scan.ranges.size(); ++current_beam_index)
    {
        valid_values.clear();
        nan_count = 0;
        neg_inf_count = 0;
        pos_inf_count = 0;

        // Collect points within the window
        for (int window_offset = -half_window; window_offset <= half_window; ++window_offset)
        {
            int index = current_beam_index + window_offset;

            if (index >= 0 && index < input_scan.ranges.size())
            {
                float value = input_scan.ranges[index];

                if (std::isnan(value))
                {
                    nan_count++;
                }
                else if (value == -std::numeric_limits<float>::infinity())
                {
                    neg_inf_count++;
                }
                else if (value == std::numeric_limits<float>::infinity())
                {
                    pos_inf_count++;
                }
                else
                {
                    valid_values.push_back(value);
                }
            }
        }

        // Determine which set is the largest
        // In case of a tie, prioritize valid-values over nan-values over neg-inf-values over pos-inf-values
        if (valid_values.size() >= nan_count && valid_values.size() >= neg_inf_count && valid_values.size() >= pos_inf_count)
        {
            // Sort the valid values and return the median
            std::sort(valid_values.begin(), valid_values.end());
            filtered_scan.ranges[current_beam_index] = valid_values[valid_values.size() / 2];
        }
        else if (nan_count >= valid_values.size() && nan_count >= neg_inf_count && nan_count >= pos_inf_count)
        {
            filtered_scan.ranges[current_beam_index] = std::numeric_limits<float>::quiet_NaN();
        }
        else if (neg_inf_count >= valid_values.size() && neg_inf_count >= nan_count && neg_inf_count >= pos_inf_count)
        {
            filtered_scan.ranges[current_beam_index] = -std::numeric_limits<float>::infinity();
        }
        else if (pos_inf_count >= valid_values.size() && pos_inf_count >= nan_count && pos_inf_count >= neg_inf_count)
        {
            filtered_scan.ranges[current_beam_index] = std::numeric_limits<float>::infinity();
        }
    }

    return true;
  }
};

} // namespace laser_filters

#endif // LASER_SCAN_MEDIAN_SPATIAL_FILTER_H
