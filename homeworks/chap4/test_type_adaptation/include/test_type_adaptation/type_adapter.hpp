#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
/*
The MIT License (MIT)

Copyright (c) 2023 Marco Matteo Bassa

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <pcl_conversions/pcl_conversions.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

/// Adapter for point cloud types
template <>
struct rclcpp::TypeAdapter<pcl::PointCloud<pcl::PointXYZ>, sensor_msgs::msg::PointCloud2> {
  /// @brief IsSpecialized
  using is_specialized = std::true_type;
  /// @brief CustomType
  using custom_type = pcl::PointCloud<pcl::PointXYZ>;
  /// @brief RosMessageType
  using ros_message_type = sensor_msgs::msg::PointCloud2;

  static void convert_to_ros_message(
      const pcl::PointCloud<pcl::PointXYZ>& input, sensor_msgs::msg::PointCloud2& output) {
    std::cout << "Converting to ROS" << std::endl << std::flush;
    pcl::toROSMsg(input, output);
  }

  static void convert_to_custom(
      const sensor_msgs::msg::PointCloud2& source, pcl::PointCloud<pcl::PointXYZ>& destination) {
    std::cout << "Converting to PCL" << std::endl << std::flush;
    pcl::fromROSMsg(source, destination);
  }
};

using PointCloudAdapted =
    rclcpp::TypeAdapter<pcl::PointCloud<pcl::PointXYZ>, sensor_msgs::msg::PointCloud2>;