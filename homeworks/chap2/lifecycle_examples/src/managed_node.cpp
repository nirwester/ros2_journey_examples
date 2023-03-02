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

#include "lifecycle_examples/managed_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace lifecycle_examples {

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

ManagedNode::ManagedNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("managed_node", options){};

CallbackReturn ManagedNode::on_configure(const rclcpp_lifecycle::State &state) {
  // Return success to indicate that configuration succeeded
  RCLCPP_INFO(get_logger(), "Configured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ManagedNode::on_activate(const rclcpp_lifecycle::State &state) {
  // Activate any resources that were paused or stopped
  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ManagedNode::on_deactivate(const rclcpp_lifecycle::State &state) {
  // Deactivate any resources that need to be paused or stopped
  return CallbackReturn::SUCCESS;
}

CallbackReturn ManagedNode::on_cleanup(const rclcpp_lifecycle::State &state) {
  // Clean up any resources used by the node
  return CallbackReturn::SUCCESS;
}

} // namespace lifecycle_examples

RCLCPP_COMPONENTS_REGISTER_NODE(lifecycle_examples::ManagedNode)
