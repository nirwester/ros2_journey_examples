- Compile using "colcon build" from your workspace root folder
- Source the install space of your workspace: "source install/setup.sh"
- To reproduce a deadlock launch: "ros2 launch service_test_pkg launch_with_deadlock.launch.py"
- To run without deadlocks: "ros2 launch service_test_pkg launch_without_deadlock.launch.py"

Use the parameters specified in the launchfile to enable / disable the callback group and the multithreaded executor
