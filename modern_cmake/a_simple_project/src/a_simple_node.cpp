#include <cstdio>
#include "a_simple_project/a_simple_lib.hpp"

int main(int argc, char ** argv)
{
  const rclcpp::NodeOptions options;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<example::ASimpleNode>(options));
  rclcpp::shutdown();
  return 0;
}
