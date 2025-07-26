#include "a_simple_project/a_simple_lib.hpp"

#include <cstdio>
#include <rclcpp_components/register_node_macro.hpp>

namespace example {

ASimpleNode::ASimpleNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ASimpleNode", options) {}

int ASimpleNode::sumInts(int first, int second) { return first + second; }
}  // namespace example

RCLCPP_COMPONENTS_REGISTER_NODE(example::ASimpleNode)
