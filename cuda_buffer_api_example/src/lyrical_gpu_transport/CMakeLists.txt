cmake_minimum_required(VERSION 3.20)
project(cuda_buffer_api_example LANGUAGES CXX CUDA)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cuda_buffer REQUIRED)
find_package(cuda_buffer_backend REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_buffer REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(CUDAToolkit REQUIRED)

# Downloading test input data (stanford bunny)
set(BUNNY_ARCHIVE "${CMAKE_CURRENT_BINARY_DIR}/bunny.tar.gz")
set(BUNNY_EXTRACT_DIR "${CMAKE_CURRENT_BINARY_DIR}/stanford_bunny")
set(BUNNY_INPUT_PLY "${CMAKE_CURRENT_BINARY_DIR}/input.ply")
file(DOWNLOAD
  "https://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz"
  "${BUNNY_ARCHIVE}"
  EXPECTED_HASH SHA256=a5720bd96d158df403d153381b8411a727a1d73cff2f33dc9b212d6f75455b84
  TLS_VERIFY ON
  STATUS BUNNY_DOWNLOAD_STATUS
)
list(GET BUNNY_DOWNLOAD_STATUS 0 BUNNY_DOWNLOAD_RESULT)
if(NOT BUNNY_DOWNLOAD_RESULT EQUAL 0)
  list(GET BUNNY_DOWNLOAD_STATUS 1 BUNNY_DOWNLOAD_ERROR)
  message(FATAL_ERROR "Failed to download Stanford bunny dataset: ${BUNNY_DOWNLOAD_ERROR}")
endif()
file(ARCHIVE_EXTRACT
  INPUT "${BUNNY_ARCHIVE}"
  DESTINATION "${BUNNY_EXTRACT_DIR}"
  PATTERNS "bunny/data/bun000.ply"
)
configure_file(
  "${BUNNY_EXTRACT_DIR}/bunny/data/bun000.ply"
  "${BUNNY_INPUT_PLY}"
  COPYONLY
)

function(link_common_cuda_target target_name)
  target_link_libraries(${target_name}
    cuda_buffer::cuda_buffer
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    CUDA::cudart
    CUDA::cuda_driver
  )
endfunction()

add_executable(native_gpu_publisher src/gpu_publisher.cpp)
target_link_libraries(native_gpu_publisher
  ament_index_cpp::ament_index_cpp
)
link_common_cuda_target(native_gpu_publisher)

add_executable(gpu_voxel_filter src/gpu_voxel_filter.cpp)
target_sources(gpu_voxel_filter PRIVATE src/gpu_voxel_filter_kernels.cu)
set_target_properties(gpu_voxel_filter PROPERTIES
  LINKER_LANGUAGE CXX
)
link_common_cuda_target(gpu_voxel_filter)
execute_process(
  COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libstdc++.so
  OUTPUT_VARIABLE LIBSTDCXX_SO
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(EXISTS "${LIBSTDCXX_SO}")
  target_link_libraries(gpu_voxel_filter "${LIBSTDCXX_SO}")
endif()

add_executable(native_gpu_subscriber src/gpu_subscriber.cpp)
link_common_cuda_target(native_gpu_subscriber)

install(TARGETS native_gpu_publisher gpu_voxel_filter native_gpu_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(FILES "${BUNNY_INPUT_PLY}"
  DESTINATION share/${PROJECT_NAME}/data)

ament_package()
