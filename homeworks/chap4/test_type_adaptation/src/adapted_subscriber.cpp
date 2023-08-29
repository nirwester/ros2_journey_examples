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

#include "test_type_adaptation/adapted_subscriber.hpp"

#include <pcl/common/io.h>

#include <cinttypes>
#include <cstdio>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std;

namespace journey {

AdaptedSubscriber::AdaptedSubscriber(const rclcpp::NodeOptions& options)
    : rclcpp::Node("adapted_subscriber", options) {
  subscriber_ = create_subscription<PointCloudAdapted>("topic_with_adapted_subscriber",
      rclcpp::SensorDataQoS(),
      std::bind(&AdaptedSubscriber::pointcloudAdaptedCallback, this, std::placeholders::_1));
  // Note: if subscribing with the normal ROS type to a topic whose publisher uses a custom type,
  // msgs won't go through beacause of the type mismatch, unless the intra process communication is
  // disabled.
  auto normal_sub_options = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>();
  // normal_sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  normal_subscriber_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "topic_without_adapted_subscriber", rclcpp::SensorDataQoS(),
      std::bind(&AdaptedSubscriber::pointcloudNormalCallback, this, std::placeholders::_1),
      normal_sub_options);
}

void AdaptedSubscriber::pointcloudAdaptedCallback(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  RCLCPP_INFO(get_logger(), "Adapted sub received cloud with address: 0x%" PRIXPTR "\n",
      reinterpret_cast<std::uintptr_t>(cloud.get()));
}

void AdaptedSubscriber::pointcloudNormalCallback(sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
  RCLCPP_INFO(get_logger(), "Normal sub received cloud with address: 0x%" PRIXPTR "\n",
      reinterpret_cast<std::uintptr_t>(cloud.get()));
}

}  // namespace journey

RCLCPP_COMPONENTS_REGISTER_NODE(journey::AdaptedSubscriber)