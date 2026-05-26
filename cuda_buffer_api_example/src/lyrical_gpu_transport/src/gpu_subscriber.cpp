#include <cuda_runtime.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

#include "cuda_buffer/cuda_buffer_api.hpp"
#include "cuda_buffer/cuda_error.hpp"
#include "rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class NativeGpuSubscriber : public rclcpp::Node
{
public:
  NativeGpuSubscriber()
  : Node("native_gpu_subscriber")
  {
    output_path_ = declare_parameter<std::string>("output_path", "/output/filtered_cloud.ply");

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.acceptable_buffer_backends = "cuda";

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "filtered_points", 10,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        if (msg->data.size() == 0) {
          RCLCPP_WARN(get_logger(), "Received an empty filtered cloud; skipping save");
          return;
        }

        const auto points = read_points(*msg);
        save_ply(points);

        RCLCPP_INFO(
          get_logger(),
          "Saved %zu filtered CUDA-backed points from '%s' backend to %s",
          points.size() / 3, msg->data.get_backend_type().c_str(), output_path_.c_str());
      },
      subscription_options);
  }

private:
  std::vector<float> read_points(const sensor_msgs::msg::PointCloud2 & msg) const
  {
    std::vector<float> points(msg.data.size() / sizeof(float));
    if (points.empty()) {
      return points;
    }

    if (msg.data.get_backend_type() == "cuda") {
      auto read_handle = cuda_buffer_backend::from_input_buffer(msg.data, nullptr);
      CUDA_CHECK(cudaMemcpy(
        points.data(), read_handle.get_ptr(), msg.data.size(), cudaMemcpyDeviceToHost));
      return points;
    }

    std::memcpy(points.data(), msg.data.data(), msg.data.size());
    return points;
  }

  void save_ply(const std::vector<float> & points) const
  {
    const std::filesystem::path path(output_path_);
    if (path.has_parent_path()) {
      std::filesystem::create_directories(path.parent_path());
    }

    std::ofstream output(output_path_);
    if (!output.is_open()) {
      throw std::runtime_error("failed to open filtered PLY for writing: " + output_path_);
    }

    const auto point_count = points.size() / 3;
    output << "ply\n";
    output << "format ascii 1.0\n";
    output << "element vertex " << point_count << "\n";
    output << "property float x\n";
    output << "property float y\n";
    output << "property float z\n";
    output << "end_header\n";

    for (size_t i = 0; i + 2 < points.size(); i += 3) {
      output << points[i] << ' ' << points[i + 1] << ' ' << points[i + 2] << '\n';
    }
  }

  std::string output_path_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NativeGpuSubscriber>();
  rclcpp::executors::EventsCBGExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
