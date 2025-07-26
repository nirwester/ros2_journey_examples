#include <memory>
#include <string>
#include <utility>

#include "an_interface_lib/a_plugin_base.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("use_obstacle_plugin");
  pluginlib::ClassLoader<example::Obstacle> obstacle_loader(
      "an_interface_lib",  // Package containing the base class for lookup
      "example::Obstacle"  // Fully qualified name of the base class
  );
  std::shared_ptr<example::Obstacle> my_obstacle;
  try {
    my_obstacle = obstacle_loader.createSharedInstance("example::PointObstacle");
    RCLCPP_INFO(logger, "Successfully loaded PointObstacle plugin!");
    my_obstacle->initialize(10.0, 20.0);
    std::pair<double, double> pos = my_obstacle->getPosition();
    RCLCPP_INFO(logger, "Plugin reported position: (%.2f, %.2f)", pos.first, pos.second);
  } catch (pluginlib::PluginlibException& ex) {
    RCLCPP_ERROR(logger, "Failed to load plugin: %s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
