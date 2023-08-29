ARG FROM_IMAGE=osrf/ros:iron-desktop
ARG OVERLAY_WS=/opt/ros/overlay_ws

FROM $FROM_IMAGE

# clone overlay source
ARG OVERLAY_WS
WORKDIR $OVERLAY_WS
COPY src src

# install overlay dependencies
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && rosdep install -y \
      --from-paths src \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/*

# build overlay source
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build

# source entrypoint setup
ENV OVERLAY_WS $OVERLAY_WS
RUN sed --in-place --expression \
      '$isource "$OVERLAY_WS/install/setup.bash"' \
      /ros_entrypoint.sh

# run launch file
CMD ["ros2", "launch", "test_type_adaptation", "test_type_adaptation.launch.py"]
