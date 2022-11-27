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

#include "service_test_pkg/service_server.hpp"

using namespace std::chrono_literals;

ServiceServer::ServiceServer(const rclcpp::NodeOptions& options)
    : rclcpp::Node("service_client", options) {
  service_ = create_service<std_srvs::srv::Empty>(
      "test_service", std::bind(&ServiceServer::serviceCallback, this, std::placeholders::_1,
                          std::placeholders::_2));
  RCLCPP_INFO(get_logger(), "Service started");
}

void ServiceServer::serviceCallback(std_srvs::srv::Empty::Request::SharedPtr srv_request,
    std_srvs::srv::Empty::Response::SharedPtr srv_response) {
  RCLCPP_INFO(get_logger(), "Got a request");
}
