#include "gpu_voxel_filter_kernels.hpp"

#include <algorithm>
#include <cstdint>

#include "cuda_buffer/cuda_error.hpp"

namespace
{

constexpr uint64_t kEmptyKey = 0;
constexpr int32_t kVoxelBias = 1 << 20;
constexpr int32_t kVoxelMin = -kVoxelBias;
constexpr int32_t kVoxelMax = kVoxelBias - 1;

uint32_t next_power_of_two(uint32_t value)
{
  if (value <= 1) {
    return 1;
  }

  --value;
  value |= value >> 1;
  value |= value >> 2;
  value |= value >> 4;
  value |= value >> 8;
  value |= value >> 16;
  return value + 1;
}

__device__ uint64_t splitmix64(uint64_t value)
{
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

__device__ bool make_voxel_key(float x, float y, float z, float inverse_leaf_size, uint64_t * key)
{
  const int32_t ix = static_cast<int32_t>(floorf(x * inverse_leaf_size));
  const int32_t iy = static_cast<int32_t>(floorf(y * inverse_leaf_size));
  const int32_t iz = static_cast<int32_t>(floorf(z * inverse_leaf_size));

  if (ix < kVoxelMin || ix > kVoxelMax ||
    iy < kVoxelMin || iy > kVoxelMax ||
    iz < kVoxelMin || iz > kVoxelMax)
  {
    return false;
  }

  const uint64_t ux = static_cast<uint64_t>(ix + kVoxelBias);
  const uint64_t uy = static_cast<uint64_t>(iy + kVoxelBias);
  const uint64_t uz = static_cast<uint64_t>(iz + kVoxelBias);
  *key = (ux << 42) | (uy << 21) | uz;
  return true;
}

__global__ void accumulate_voxels_kernel(
  const float * input_points,
  uint32_t input_point_count,
  float inverse_leaf_size,
  uint64_t * voxel_keys,
  float * sum_x,
  float * sum_y,
  float * sum_z,
  uint32_t * counts,
  uint32_t table_mask)
{
  const uint32_t point_index = blockIdx.x * blockDim.x + threadIdx.x;
  if (point_index >= input_point_count) {
    return;
  }

  const float x = input_points[point_index * 3];
  const float y = input_points[point_index * 3 + 1];
  const float z = input_points[point_index * 3 + 2];

  uint64_t key = 0;
  if (!make_voxel_key(x, y, z, inverse_leaf_size, &key)) {
    return;
  }
  ++key;

  uint32_t slot = static_cast<uint32_t>(splitmix64(key)) & table_mask;
  while (true) {
    const auto previous = atomicCAS(
      reinterpret_cast<unsigned long long *>(&voxel_keys[slot]),
      static_cast<unsigned long long>(kEmptyKey),
      static_cast<unsigned long long>(key));

    if (previous == kEmptyKey || previous == key) {
      atomicAdd(&sum_x[slot], x);
      atomicAdd(&sum_y[slot], y);
      atomicAdd(&sum_z[slot], z);
      atomicAdd(&counts[slot], 1U);
      return;
    }

    slot = (slot + 1) & table_mask;
  }
}

__global__ void compact_voxels_kernel(
  const uint64_t * voxel_keys,
  const float * sum_x,
  const float * sum_y,
  const float * sum_z,
  const uint32_t * counts,
  uint32_t table_size,
  float * output_points,
  uint32_t output_capacity,
  uint32_t * output_count)
{
  const uint32_t slot = blockIdx.x * blockDim.x + threadIdx.x;
  if (slot >= table_size || voxel_keys[slot] == kEmptyKey) {
    return;
  }

  const uint32_t count = counts[slot];
  if (count == 0) {
    return;
  }

  const uint32_t output_index = atomicAdd(output_count, 1U);
  if (output_index >= output_capacity) {
    return;
  }

  const float inverse_count = 1.0F / static_cast<float>(count);
  output_points[output_index * 3] = sum_x[slot] * inverse_count;
  output_points[output_index * 3 + 1] = sum_y[slot] * inverse_count;
  output_points[output_index * 3 + 2] = sum_z[slot] * inverse_count;
}

}  // namespace

GpuVoxelFilterResult run_gpu_voxel_filter(
  const float * input_points,
  uint32_t input_point_count,
  float leaf_size,
  float * output_points,
  uint32_t output_capacity,
  cudaStream_t stream)
{
  if (input_point_count == 0 || leaf_size <= 0.0F) {
    return {0};
  }

  const uint32_t table_size = next_power_of_two(std::max<uint32_t>(input_point_count * 4, 1024));
  const size_t keys_bytes = table_size * sizeof(uint64_t);
  const size_t floats_bytes = table_size * sizeof(float);
  const size_t counts_bytes = table_size * sizeof(uint32_t);

  uint64_t * voxel_keys = nullptr;
  float * sum_x = nullptr;
  float * sum_y = nullptr;
  float * sum_z = nullptr;
  uint32_t * counts = nullptr;
  uint32_t * output_count_device = nullptr;

  CUDA_CHECK(cudaMallocAsync(&voxel_keys, keys_bytes, stream));
  CUDA_CHECK(cudaMallocAsync(&sum_x, floats_bytes, stream));
  CUDA_CHECK(cudaMallocAsync(&sum_y, floats_bytes, stream));
  CUDA_CHECK(cudaMallocAsync(&sum_z, floats_bytes, stream));
  CUDA_CHECK(cudaMallocAsync(&counts, counts_bytes, stream));
  CUDA_CHECK(cudaMallocAsync(&output_count_device, sizeof(uint32_t), stream));

  CUDA_CHECK(cudaMemsetAsync(voxel_keys, 0, keys_bytes, stream));
  CUDA_CHECK(cudaMemsetAsync(sum_x, 0, floats_bytes, stream));
  CUDA_CHECK(cudaMemsetAsync(sum_y, 0, floats_bytes, stream));
  CUDA_CHECK(cudaMemsetAsync(sum_z, 0, floats_bytes, stream));
  CUDA_CHECK(cudaMemsetAsync(counts, 0, counts_bytes, stream));
  CUDA_CHECK(cudaMemsetAsync(output_count_device, 0, sizeof(uint32_t), stream));

  constexpr uint32_t block_size = 256;
  const uint32_t point_blocks = (input_point_count + block_size - 1) / block_size;
  accumulate_voxels_kernel<<<point_blocks, block_size, 0, stream>>>(
    input_points, input_point_count, 1.0F / leaf_size, voxel_keys, sum_x, sum_y, sum_z,
    counts, table_size - 1);
  CUDA_CHECK(cudaGetLastError());

  const uint32_t table_blocks = (table_size + block_size - 1) / block_size;
  compact_voxels_kernel<<<table_blocks, block_size, 0, stream>>>(
    voxel_keys, sum_x, sum_y, sum_z, counts, table_size, output_points, output_capacity,
    output_count_device);
  CUDA_CHECK(cudaGetLastError());

  uint32_t output_count = 0;
  CUDA_CHECK(cudaMemcpyAsync(
    &output_count, output_count_device, sizeof(uint32_t), cudaMemcpyDeviceToHost, stream));
  CUDA_CHECK(cudaStreamSynchronize(stream));

  CUDA_CHECK(cudaFreeAsync(voxel_keys, stream));
  CUDA_CHECK(cudaFreeAsync(sum_x, stream));
  CUDA_CHECK(cudaFreeAsync(sum_y, stream));
  CUDA_CHECK(cudaFreeAsync(sum_z, stream));
  CUDA_CHECK(cudaFreeAsync(counts, stream));
  CUDA_CHECK(cudaFreeAsync(output_count_device, stream));
  CUDA_CHECK(cudaStreamSynchronize(stream));

  return {std::min(output_count, output_capacity)};
}
