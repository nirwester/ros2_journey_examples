#include <a_plugin/a_plugin.hpp>
#include <cmath>

namespace example {

void PointObstacle::initialize(double x, double y) {
  x_ = x;
  y_ = y;
}

std::pair<double, double> PointObstacle::getPosition() { return std::pair<double, double>{x_, y_}; }

}  // namespace example

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(example::PointObstacle, example::Obstacle)
