#pragma once
#include <an_interface_lib/a_plugin_base.hpp>
#include <utility>

namespace example {

class PointObstacle : public example::Obstacle {
 public:
  void initialize(double x, double y) override;

  std::pair<double, double> getPosition() override;
};

}  // namespace example
