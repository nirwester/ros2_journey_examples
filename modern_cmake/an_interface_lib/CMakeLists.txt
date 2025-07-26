cmake_minimum_required(VERSION 3.8)
project(an_interface_lib)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
add_library(an_interface_library INTERFACE)

target_include_directories(
  an_interface_library
  INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(TARGETS an_interface_library EXPORT export_an_interface_library)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_an_interface_library)

ament_package()
