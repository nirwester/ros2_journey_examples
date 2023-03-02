COMPILATION

- Setup ROS 2 Galactic (small adaptations are required for calling services on Humble and Rolling)
- Put the package in a folder named "src" and run "colcon build".

EXECUTION

Terminal 1
- source install/setup.sh
- ros2 launch lifecycle_examples lifecycle_example.launch.py

Terminal 2
- ros2 service call /trigger_activation std_srvs/srv/Trigger "{}"

EXPECTED OUTPUT:

[component_container_mt-1] [INFO] [1677784853.602283758] [node1]: Configured

[component_container_mt-1] [INFO] [1677784853.602738883] [node2]: Configured

[component_container_mt-1] [INFO] [1677784853.603004783] [node1]: Activated

[component_container_mt-1] [INFO] [1677784853.603173078] [node2]: Activated

[component_container_mt-1] [INFO] [1677784853.603276840] [manager]: Initialization completed


NOTE:
The code calls a service from a service callback, hence a dedicated callback group
and a multithreaded executor are required.
