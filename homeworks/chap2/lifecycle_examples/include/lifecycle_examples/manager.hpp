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

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <map>
#include <string>
#include <vector>

namespace lifecycle_examples {

using ChangeStateClient = rclcpp::Client<lifecycle_msgs::srv::ChangeState>;

/// Initializes a set of nodes upon receiving a service request.
class Manager : public rclcpp::Node {
public:
  /// @brief Constructor
  /// @param options
  explicit Manager(const rclcpp::NodeOptions& options);

protected:
  /// Service to trigger the activation of the managed nodes
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_activation_;
  /// Callback executed to activate the services
  void
  activateCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                   std_srvs::srv::Trigger::Response::SharedPtr response);
  /// Callback group used by the trigger service
  rclcpp::CallbackGroup::SharedPtr cb_trigger_;
  /// Service to trigger the activation
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
  /// Nodes managed by this manager.
  std::vector<std::string> managed_nodes_;
  /// Client used to change the state of the managed nodes
  std::map<std::string, ChangeStateClient::SharedPtr> change_state_clients_;
};

} // namespace lifecycle_examples
