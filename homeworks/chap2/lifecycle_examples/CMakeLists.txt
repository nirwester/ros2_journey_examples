cmake_minimum_required(VERSION 3.5)
project(lifecycle_examples)

find_package(ament_cmake REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
    PUBLIC 
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

set(dependencies
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_srvs
)

add_library(manager SHARED
  src/manager.cpp
)

ament_target_dependencies(manager
  ${dependencies}
)

add_library(managed_node SHARED
  src/managed_node.cpp
)

ament_target_dependencies(managed_node
  ${dependencies}
)

rclcpp_components_register_nodes(manager "lifecycle_examples::Manager")
rclcpp_components_register_nodes(managed_node "lifecycle_examples::ManagedNode")

install(TARGETS
  manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  managed_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_export_include_directories(include)
ament_export_libraries(manager managed_node)
ament_export_dependencies(${dependencies})

ament_package()
