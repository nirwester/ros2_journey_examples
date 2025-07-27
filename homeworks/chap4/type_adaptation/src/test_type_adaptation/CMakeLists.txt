cmake_minimum_required(VERSION 3.8)
project(test_type_adaptation)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -std=c++17)

find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp_components REQUIRED)

set(CPP_FILES src/adapted_publisher.cpp src/adapted_subscriber.cpp)

add_library(${PROJECT_NAME} SHARED ${CPP_FILES})

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
                         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})

ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp_components
pcl_conversions sensor_msgs)

# Starting from Kilted
# target_link_libraries(
#   ${PROJECT_NAME}
#   PUBLIC ${pcl_conversions_TARGETS}
#          ${PCL_LIBRARIES} rclcpp::rclcpp rclcpp_components::component
#          ${sensor_msgs_TARGETS})

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)

rclcpp_components_register_nodes(${PROJECT_NAME} "journey::AdaptedPublisher")
rclcpp_components_register_nodes(${PROJECT_NAME} "journey::AdaptedSubscriber")

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_export_dependencies(pcl_conversions rclcpp_components sensor_msgs)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_package()
