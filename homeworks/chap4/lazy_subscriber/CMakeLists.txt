cmake_minimum_required(VERSION 3.5)
project(lazy_subscriber)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs)

add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

add_executable(lazy_subscriber src/lazy_subscriber.cpp)
ament_target_dependencies(lazy_subscriber rclcpp std_msgs)

install(TARGETS
  publisher
  subscriber
  lazy_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()