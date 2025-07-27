cmake_minimum_required(VERSION 3.5)
project(lazy_subscriber)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(publisher src/publisher.cpp)

target_link_libraries(publisher PRIVATE
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber PRIVATE
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

add_executable(lazy_subscriber src/lazy_subscriber.cpp)
target_link_libraries(lazy_subscriber PRIVATE
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

install(TARGETS
  publisher
  subscriber
  lazy_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()