#include "exporting_a_lib/a_very_useful_lib.hpp"

#include <cmath>

namespace example {

double distance2d(
    const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2) {
  return sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
}
}  // namespace example
