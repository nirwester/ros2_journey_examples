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

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "test_type_adaptation/type_adapter.hpp"

namespace journey {

/**
 * Node subscribing to a pointcloud both as adapted and not adapted type
 */
class AdaptedSubscriber : public rclcpp::Node {
 public:
  /// Class constructor
  AdaptedSubscriber(const rclcpp::NodeOptions& options);

 private:
  void pointcloudAdaptedCallback(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  void pointcloudNormalCallback(sensor_msgs::msg::PointCloud2::SharedPtr cloud);

  rclcpp::Subscription<PointCloudAdapted>::SharedPtr subscriber_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr normal_subscriber_;
};

}  // namespace journey
