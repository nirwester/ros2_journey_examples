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

#include "lifecycle_examples/manager.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace lifecycle_examples {

using ChangeStateClient = rclcpp::Client<lifecycle_msgs::srv::ChangeState>;

Manager::Manager(const rclcpp::NodeOptions &options)
    : rclcpp::Node("manager", options) {
  // The managed nodes can be configured using parameters
  managed_nodes_ = declare_parameter(
      "managed_nodes", std::vector<std::string>{"node_1", "node_2"});
  // For each managed node, a client needs to be created to trigger its
  // transitions
  for (const auto &managed_node : managed_nodes_) {
    change_state_clients_[managed_node] =
        create_client<lifecycle_msgs::srv::ChangeState>(managed_node +
                                                        "/change_state");
  }
  // The code calls a service from a service callback, hence a dedicated
  // callback group and a multithreaded executor are required.
  cb_trigger_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  trigger_srv_ = create_service<std_srvs::srv::Trigger>(
      "trigger_activation",
      std::bind(&Manager::activateCallback, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, cb_trigger_);
}

void Manager::activateCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  // Waiting services availability
  response->success = false;
  for (const auto &managed_node : managed_nodes_) {
    while (!change_state_clients_[managed_node]->wait_for_service(1s) &&
           rclcpp::ok()) {
      RCLCPP_INFO_STREAM(get_logger(), "Waiting service of " << managed_node);
    }
  }
  // Utility lambda to wait for the result.
  auto waitTransition = [this](auto &result, const std::string &node_name) {
    // This works for ROS 2 Galactic, on Humble and later you'll need to access
    // the shared future. It would be a good idea to add a timeout here.
    while (result.wait_for(1s) != std::future_status::ready && rclcpp::ok()) {
      RCLCPP_INFO_STREAM(get_logger(), "Waiting transition of " << node_name);
    }
  };
  auto config_request =
      std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  // Asking all the managed nodes to configure
  config_request->transition.id =
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
  for (const auto &managed_node : managed_nodes_) {
    auto result =
        change_state_clients_[managed_node]->async_send_request(config_request);
    waitTransition(result, managed_node);
    if (!result.get()->success) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to configure " << managed_node);
      return;
    }
  }
  // Asking all the managed nodes to activate
  config_request->transition.id =
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
  for (const auto &managed_node : managed_nodes_) {
    auto result =
        change_state_clients_[managed_node]->async_send_request(config_request);
    waitTransition(result, managed_node);
    if (!result.get()->success) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to activate " << managed_node);
      return;
    }
  }
  response->success = true;
  RCLCPP_INFO(get_logger(), "Initialization completed");
}

} // namespace lifecycle_examples

RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_examples::Manager)
