#include <cuda_runtime.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "cuda_buffer/cuda_buffer_api.hpp"
#include "happly.h"
#include "rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosidl_buffer/buffer.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class NativeGpuPublisher : public rclcpp::Node
{
public:
  NativeGpuPublisher()
  : Node("native_gpu_publisher")
  {
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", 10);
    host_points_ = load_points();

    timer_ = create_wall_timer(500ms, [this]() {
      auto msg = create_message();
      const auto backend = msg.data.get_backend_type();
      RCLCPP_INFO(
        get_logger(),
        "Publishing %zu points from a %s-backed rosidl::Buffer",
        host_points_.size() / 3, backend.c_str());
      publisher_->publish(msg);
    });
  }

private:
  std::vector<float> load_points()
  {
    const auto package_input =
      (ament_index_cpp::get_package_share_path("cuda_buffer_api_example") / "data" / "input.ply").string();
    try {
      happly::PLYData ply_in(package_input);
      auto & vertices = ply_in.getElement("vertex");
      std::vector<float> points;
      const auto x_prop = vertices.getProperty<float>("x");
      const auto y_prop = vertices.getProperty<float>("y");
      const auto z_prop = vertices.getProperty<float>("z");
      points.reserve(x_prop.size() * 3);
      for (size_t i = 0; i < x_prop.size(); ++i) {
        points.push_back(x_prop[i]);
        points.push_back(y_prop[i]);
        points.push_back(z_prop[i]);
      }
      return points;
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        get_logger(),
        "Could not load %s (%s); publishing a generated point cloud",
        package_input.c_str(), error.what());
    }

    std::vector<float> points;
    points.reserve(1024 * 3);
    for (size_t i = 0; i < 1024; ++i) {
      const float t = static_cast<float>(i) * 0.01F;
      points.push_back(t);
      points.push_back(t * t);
      points.push_back(0.25F * t);
    }
    return points;
  }

  rosidl::Buffer<uint8_t> make_data_buffer(size_t bytes) const
  {
    int device_count = 0;
    const auto device_result = cudaGetDeviceCount(&device_count);
    if (device_result == cudaSuccess && device_count > 0) {
      auto buffer = cuda_buffer_backend::allocate_buffer(bytes);
      auto write_handle = cuda_buffer_backend::from_output_buffer(buffer, nullptr);
      RCLCPP_INFO(
        get_logger(),
        "Writing point data into CUDA buffer at device address %p",
        write_handle.get_ptr());
      cuda_buffer_backend::to_buffer(
        host_points_.data(), bytes, write_handle, nullptr, cudaMemcpyHostToDevice);
      return buffer;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "No CUDA device is visible to this container; publishing a CPU-backed rosidl::Buffer");

    rosidl::Buffer<uint8_t> buffer(bytes);
    std::memcpy(buffer.data(), host_points_.data(), bytes);
    return buffer;
  }

  sensor_msgs::msg::PointCloud2 create_message() const
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.width = static_cast<uint32_t>(host_points_.size() / 3);
    msg.height = 1;
    msg.is_bigendian = false;
    msg.point_step = 3 * sizeof(float);
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = true;

    sensor_msgs::msg::PointField field_x;
    field_x.name = "x";
    field_x.offset = 0;
    field_x.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_x.count = 1;

    sensor_msgs::msg::PointField field_y;
    field_y.name = "y";
    field_y.offset = sizeof(float);
    field_y.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_y.count = 1;

    sensor_msgs::msg::PointField field_z;
    field_z.name = "z";
    field_z.offset = 2 * sizeof(float);
    field_z.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field_z.count = 1;

    msg.fields = {field_x, field_y, field_z};
    msg.data = make_data_buffer(host_points_.size() * sizeof(float));

    return msg;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<float> host_points_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NativeGpuPublisher>();
  rclcpp::executors::EventsCBGExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
