Build and run the CUDA-backed point cloud pipeline:

```bash
docker compose up --build
```

The pipeline is:

1. `native_gpu_publisher` loads the Stanford bunny PLY and publishes `points` with the CUDA `rosidl::Buffer` backend.
2. `gpu_voxel_filter` receives `points`, applies a CUDA voxel grid centroid filter without copying point data back to the CPU, and republishes `filtered_points` with the CUDA backend.
3. `native_gpu_subscriber` receives `filtered_points` and writes `/output/filtered_cloud.ply`, mounted to `./output/filtered_cloud.ply` on the host.

The example uses the CUDA `rosidl::Buffer` backend shipped with ROS Lyrical and opts subscribers into the `cuda` backend. It also uses Lyrical's `EventsCBGExecutor` in all three nodes. Only the subscriber copies the final filtered cloud to host memory, because it writes a PLY file.

The Stanford bunny input cloud is downloaded by CMake from the Stanford 3D Scanning Repository during the image build and installed with the demo package.
