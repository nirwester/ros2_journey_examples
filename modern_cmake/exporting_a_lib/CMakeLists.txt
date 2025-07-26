cmake_minimum_required(VERSION 3.8)
project(exporting_a_lib)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(a_very_useful_lib SHARED src/a_very_useful_lib.cpp)

target_include_directories(a_very_useful_lib
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(a_very_useful_lib PUBLIC
  ${geometry_msgs_TARGETS}
)

install(TARGETS a_very_useful_lib
  EXPORT export_a_very_useful_lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_a_very_useful_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs)

ament_package()
