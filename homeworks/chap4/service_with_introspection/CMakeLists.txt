cmake_minimum_required(VERSION 3.5)
project(service_with_introspection)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()