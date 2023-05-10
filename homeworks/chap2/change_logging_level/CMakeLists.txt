cmake_minimum_required(VERSION 3.5)
project(change_logging_level)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

install(TARGETS
  publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()