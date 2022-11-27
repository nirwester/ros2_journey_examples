/*
The MIT License (MIT)

Copyright (c) 2022 Marco Matteo Bassa

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

#include "service_test_pkg/service_client.hpp"

using namespace std::chrono_literals;

ServiceClient::ServiceClient(const rclcpp::NodeOptions& options)
    : rclcpp::Node("service_client", options) {
  client_ = create_client<std_srvs::srv::Empty>("test_service");
  client_->wait_for_service();
  auto use_cb_group = declare_parameter("use_cb_group", false);
  if (use_cb_group) {
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, true);
    timer_ = create_wall_timer(500ms, std::bind(&ServiceClient::timerCallback, this), cb_group_);
  } else {
    timer_ = create_wall_timer(500ms, std::bind(&ServiceClient::timerCallback, this));
  }
  RCLCPP_INFO(get_logger(), "Client started");
}

void ServiceClient::timerCallback() {
  RCLCPP_INFO(get_logger(), "Sending request");
  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  auto future_result = client_->async_send_request(request);
  future_result.wait();
  RCLCPP_INFO(get_logger(), "Got result");
}
