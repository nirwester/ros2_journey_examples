#pragma once
#include <utility>

namespace example {

class Obstacle {
 public:
  virtual void initialize(double x, double y) = 0;
  virtual std::pair<double, double> getPosition() = 0;
  virtual ~Obstacle() {}

 protected:
  Obstacle() {}
  double x_{0.0};
  double y_{0.0};
};

}  // namespace example
