cmake_minimum_required(VERSION 3.8)
project(using_a_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(a_plugin REQUIRED)
find_package(an_interface_lib REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(use_obstacle_plugin src/use_obstacle_plugin.cpp)

target_link_libraries(use_obstacle_plugin PRIVATE
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${an_interface_lib_TARGETS}
)

install(TARGETS use_obstacle_plugin
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()