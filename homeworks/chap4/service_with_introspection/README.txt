Compile with "colcon build" note: requires ROS 2 >= Iron
source install/setup.sh

Terminal 1: ros2 run service_with_introspection server
Terminal 2: ros2 topic echo /add_two_ints/_service_event
Terminal 3: ros2 run service_with_introspection client
