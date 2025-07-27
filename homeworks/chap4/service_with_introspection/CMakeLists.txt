cmake_minimum_required(VERSION 3.5)
project(service_with_introspection)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(server src/server.cpp)
target_link_libraries(server PRIVATE
  rclcpp::rclcpp
  ${example_interfaces_TARGETS}
)

add_executable(client src/client.cpp)
target_link_libraries(client PRIVATE
  rclcpp::rclcpp
  ${example_interfaces_TARGETS}
)

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()