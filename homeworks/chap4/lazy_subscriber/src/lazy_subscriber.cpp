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

#include <algorithm>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class LazySubscriber : public rclcpp::Node {
public:
  LazySubscriber() : Node("lazy_subscriber") {
    rclcpp::PublisherOptions options;
    options.event_callbacks.matched_callback =
        [this](rmw_matched_status_t &status) {
          RCLCPP_INFO_STREAM(
              get_logger(), "Tot Matching connections: " << status.total_count);
          RCLCPP_INFO_STREAM(get_logger(),
                             "Tot Count change: " << status.total_count_change);
          RCLCPP_INFO_STREAM(get_logger(),
                             "Current connections: " << status.current_count);
          RCLCPP_INFO_STREAM(get_logger(),
                             "Current change: " << status.current_count_change);
          if (status.current_count > 0 && !subscription_) {
            RCLCPP_INFO(get_logger(), "Creating subscription");
            subscription_ = create_subscription<std_msgs::msg::String>(
                "topic", 10,
                std::bind(&LazySubscriber::topic_callback, this, _1));
          } else if (status.current_count == 0 && subscription_) {
            RCLCPP_INFO(get_logger(), "Stopping subscription");
            subscription_.reset();
          }
        };
    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "topic_uppercase", 10, options);
  }

private:
  void topic_callback(const std_msgs::msg::String &msg) const {
    std::string uppercase_str = msg.data;
    std::transform(uppercase_str.begin(), uppercase_str.end(),
                   uppercase_str.begin(), ::toupper);
    std_msgs::msg::String uppercase_msg;
    uppercase_msg.data = uppercase_str;
    RCLCPP_INFO(this->get_logger(), "I heard: '%s' and published '%s'",
                msg.data.c_str(), uppercase_str.c_str());
    publisher_->publish(uppercase_msg);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LazySubscriber>());
  rclcpp::shutdown();
  return 0;
}