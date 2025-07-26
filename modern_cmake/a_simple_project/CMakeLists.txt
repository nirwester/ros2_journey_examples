# Note: you can create the base package executing ros2 pkg create --build-type
# ament_cmake --license Apache-2.0 --dependencies rclcpp rclcpp_components
# --node-name simple_executable a_simple_project

cmake_minimum_required(VERSION 3.8)
project(a_simple_project)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(simple_library SHARED src/a_simple_lib.cpp)

target_link_libraries(simple_library PUBLIC rclcpp::rclcpp
                                            rclcpp_components::component)
# Note: if you don't know the name of the target, you can specify
# ${rclcpp_components_TARGETS}

target_include_directories(
  simple_library PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

rclcpp_components_register_nodes(simple_library "example::ASimpleNode")

add_executable(simple_executable src/a_simple_node.cpp)

target_link_libraries(simple_executable PRIVATE simple_library)

install(
  TARGETS simple_library simple_executable
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
