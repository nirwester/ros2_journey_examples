cmake_minimum_required(VERSION 3.8)
project(service_test_pkg)

add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)

find_package(ament_cmake REQUIRED)
find_package(std_srvs REQUIRED)

set(CPP_FILES
  src/service_server.cpp
  src/service_client.cpp
)

add_library(${PROJECT_NAME} SHARED ${CPP_FILES})

target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  std_srvs
)

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)

add_executable(test_server_exe src/service_server_exe.cpp)
add_executable(test_client_exe src/service_client_exe.cpp)

target_link_libraries(test_server_exe
  ${PROJECT_NAME}
)

target_link_libraries(test_client_exe
  ${PROJECT_NAME}
)

install(TARGETS test_server_exe DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_client_exe DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_export_dependencies(std_srvs)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_package()