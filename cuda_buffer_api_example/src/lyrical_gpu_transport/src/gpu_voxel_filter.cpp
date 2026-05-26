#include <cuda_runtime.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>

#include "cuda_buffer/cuda_buffer_api.hpp"
#include "cuda_buffer/cuda_error.hpp"
#include "gpu_voxel_filter_kernels.hpp"
#include "rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class GpuVoxelFilter : public rclcpp::Node
{
public:
  GpuVoxelFilter()
  : Node("gpu_voxel_filter")
  {
    leaf_size_ = declare_parameter<float>("leaf_size", 0.003F);
    CUDA_CHECK(cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking));

    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.acceptable_buffer_backends = "cuda";

    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points", 10,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        if (msg->data.get_backend_type() != "cuda") {
          RCLCPP_WARN(
            get_logger(), "Dropping cloud received with '%s' backend; gpu_voxel_filter requires cuda",
            msg->data.get_backend_type().c_str());
          return;
        }

        auto filtered_msg = create_filtered_message(*msg);
        RCLCPP_INFO(
          get_logger(),
          "Filtered %u input points to %u output points entirely on GPU and publishing with '%s' backend",
          msg->width * msg->height, filtered_msg.width,
          filtered_msg.data.get_backend_type().c_str());
        publisher_->publish(filtered_msg);
      },
      subscription_options);
  }

  ~GpuVoxelFilter() override
  {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

private:
  sensor_msgs::msg::PointCloud2 create_filtered_message(const sensor_msgs::msg::PointCloud2 & input_msg)
  {
    if (input_msg.point_step != 3 * sizeof(float)) {
      throw std::runtime_error("gpu_voxel_filter expects tightly packed xyz float32 points");
    }

    const auto input_point_count = input_msg.width * input_msg.height;
    const auto output_capacity = input_point_count;
    const auto output_capacity_bytes = static_cast<size_t>(output_capacity) * input_msg.point_step;

    auto input_handle = cuda_buffer_backend::from_input_buffer(input_msg.data, stream_);
    float * compacted_points = nullptr;
    CUDA_CHECK(cudaMallocAsync(&compacted_points, output_capacity_bytes, stream_));

    const auto result = run_gpu_voxel_filter(
      reinterpret_cast<const float *>(input_handle.get_ptr()), input_point_count, leaf_size_,
      compacted_points, output_capacity, stream_);

    const auto output_bytes = static_cast<size_t>(result.point_count) * input_msg.point_step;
    auto output_buffer = cuda_buffer_backend::allocate_buffer(output_bytes);
    if (output_bytes > 0) {
      auto output_handle = cuda_buffer_backend::from_output_buffer(output_buffer, stream_);
      CUDA_CHECK(cudaMemcpyAsync(
        output_handle.get_ptr(), compacted_points, output_bytes, cudaMemcpyDeviceToDevice, stream_));
      CUDA_CHECK(cudaStreamSynchronize(stream_));
    }
    CUDA_CHECK(cudaFreeAsync(compacted_points, stream_));
    CUDA_CHECK(cudaStreamSynchronize(stream_));

    sensor_msgs::msg::PointCloud2 output_msg;
    output_msg.header = input_msg.header;
    output_msg.header.stamp = now();
    output_msg.height = 1;
    output_msg.width = result.point_count;
    output_msg.fields = input_msg.fields;
    output_msg.is_bigendian = false;
    output_msg.point_step = input_msg.point_step;
    output_msg.row_step = output_msg.point_step * output_msg.width;
    output_msg.is_dense = true;
    output_msg.data = std::move(output_buffer);
    return output_msg;
  }

  float leaf_size_{0.003F};
  cudaStream_t stream_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpuVoxelFilter>();
  rclcpp::executors::EventsCBGExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
