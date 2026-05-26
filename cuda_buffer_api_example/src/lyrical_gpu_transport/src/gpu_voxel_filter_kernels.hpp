#pragma once

#include <cstdint>

#include <cuda_runtime.h>

struct GpuVoxelFilterResult
{
  uint32_t point_count;
};

GpuVoxelFilterResult run_gpu_voxel_filter(
  const float * input_points,
  uint32_t input_point_count,
  float leaf_size,
  float * output_points,
  uint32_t output_capacity,
  cudaStream_t stream);
