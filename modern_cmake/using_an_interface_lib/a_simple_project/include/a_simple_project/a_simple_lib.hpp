#pragma once
#include <rclcpp/rclcpp.hpp>

namespace example {
class ASimpleNode : public rclcpp::Node {
 public:
  ASimpleNode(const rclcpp::NodeOptions& options);

  int sumInts(int first, int second);
};
}  // namespace example
