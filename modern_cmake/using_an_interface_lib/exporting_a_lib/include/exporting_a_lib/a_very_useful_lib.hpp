#pragma once
#include <geometry_msgs/msg/point.hpp>

namespace example {

double distance2d(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2);

}  // namespace example
