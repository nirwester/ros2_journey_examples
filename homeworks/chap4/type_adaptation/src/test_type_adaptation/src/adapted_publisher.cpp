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

#include "test_type_adaptation/adapted_publisher.hpp"

#include <pcl/common/io.h>

#include <chrono>
#include <cinttypes>
#include <cstdio>

#include "rclcpp_components/register_node_macro.hpp"

using namespace std;
using namespace chrono_literals;

namespace journey {

AdaptedPublisher::AdaptedPublisher(const rclcpp::NodeOptions& options)
    : rclcpp::Node("adapted_publisher", options) {
  publisher_ = create_publisher<PointCloudAdapted>("topic_with_adapted_subscriber", 1);
  normal_publisher_ = create_publisher<PointCloudAdapted>("topic_without_adapted_subscriber", 1);
  publish_timer_ = create_wall_timer(1s, std::bind(&AdaptedPublisher::publish, this));
}

void AdaptedPublisher::publish() {
  auto publishPointcloud = [this](rclcpp::Publisher<PointCloudAdapted>::SharedPtr publisher,
                               const std::string& screen_msg) {
    std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());
    point_cloud->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
    point_cloud->push_back(pcl::PointXYZ(4.0, 5.0, 6.0));
    std::weak_ptr<std::remove_pointer<decltype(publisher.get())>::type> captured_pub = publisher;
    auto pub_ptr = captured_pub.lock();
    RCLCPP_INFO(get_logger(), "Publishing to %s topic cloud with address: 0x%" PRIXPTR "\n",
        screen_msg.c_str(), reinterpret_cast<std::uintptr_t>(point_cloud.get()));
    pub_ptr->publish(std::move(point_cloud));
  };

  // Publishing to the topic with the adapted subscription
  publishPointcloud(publisher_, "ADAPTED");

  // Publishing to the topic with the normal subscription
  publishPointcloud(normal_publisher_, "NOT ADAPTED");
}

}  // namespace journey

RCLCPP_COMPONENTS_REGISTER_NODE(journey::AdaptedPublisher)