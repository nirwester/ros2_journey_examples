cmake_minimum_required(VERSION 3.8)
project(a_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(an_interface_lib REQUIRED)
find_package(pluginlib REQUIRED)

add_library(plugin_implementation SHARED src/a_plugin.cpp)
target_include_directories(
  plugin_implementation
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_link_libraries(plugin_implementation PUBLIC ${an_interface_lib_TARGETS}
                                                   pluginlib::pluginlib)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  TARGETS plugin_implementation
  EXPORT export_plugin_implementation
  LIBRARY DESTINATION lib)

ament_export_targets(export_plugin_implementation)
ament_export_dependencies(an_interface_lib pluginlib)

ament_package()
