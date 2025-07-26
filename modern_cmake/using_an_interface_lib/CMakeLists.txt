cmake_minimum_required(VERSION 3.8)
project(using_an_interface_lib)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(an_interface_lib REQUIRED)

add_library(another_very_useful_lib SHARED src/another_very_useful_lib.cpp)

target_include_directories(another_very_useful_lib
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(another_very_useful_lib
  PUBLIC
  an_interface_lib::an_interface_library
)

install(TARGETS
  another_very_useful_lib
  EXPORT export_another_very_useful_lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_another_very_useful_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(an_interface_lib)

ament_package()
