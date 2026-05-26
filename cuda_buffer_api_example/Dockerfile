FROM osrf/ros:lyrical-desktop

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=lyrical
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_ROOT=/opt/ros/lyrical
ENV CUDA_HOME=/usr

# Install only the CUDA build pieces needed by this sample; ROS Lyrical itself
# and the standard desktop utilities come from the base image.
RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-dev-13-1 \
    cuda-nvcc-13-1 \
    build-essential \
    cmake \
    curl \
    ros-${ROS_DISTRO}-cuda-buffer \
    ros-${ROS_DISTRO}-cuda-buffer-backend \
    && rm -rf /var/lib/apt/lists/*

# Add happly.h for the PLY input path used by the publisher.
ADD https://raw.githubusercontent.com/nmwsharp/happly/master/happly.h /usr/local/include/happly.h

WORKDIR /ros2_ws
COPY . src/lyrical_gpu_transport
RUN source "${ROS_ROOT}/setup.bash" \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source ${ROS_ROOT}/setup.bash" >> /etc/bash.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /etc/bash.bashrc

ENV AMENT_PREFIX_PATH=/ros2_ws/install:${ROS_ROOT}
ENV CMAKE_PREFIX_PATH=/ros2_ws/install:${ROS_ROOT}
ENV LD_LIBRARY_PATH=/ros2_ws/install/lib:${ROS_ROOT}/lib
ENV PATH=${ROS_ROOT}/bin:${PATH}

CMD ["bash"]
