# Compilation

colcon build

NOTE: Requires ROS2 >= Humble

## Setup using docker (optional)


Setup a docker environment:

```
docker build -t test_adaptation --pull .
```

Execute the docker environment:

```
docker run -it --rm test_adaptation

```

# Run with intra process communication

source install/setup.bash

ros2 launch test_type_adaptation test_type_adaptation.launch.py

## Example of output

[component_container-1] [INFO] [1693552812.346326041] [adapted_publisher]: Publishing to ADAPTED topic cloud with address: 0x558F6D9C64E0
[component_container-1] [INFO] [1693552812.346614871] [adapted_publisher]: Publishing to NOT ADAPTED topic cloud with address: 0x558F6D7B4C20
[component_container-1] Converting to ROS
[component_container-1] [INFO] [1693552812.347246495] [adapted_subscriber]: Adapted sub received cloud with address: 0x558F6D9C64E0
[component_container-1] [INFO] [1693552812.347344995] [adapted_subscriber]: Received point 1 x: 1 y 2 z 3, Point 2 x: 4 y 5 z 3
[component_container-1] [INFO] [1693552812.347532823] [adapted_subscriber]: Normal sub received cloud with address: 0x558F6DCBE690 
[component_container-1] [INFO] [1693552812.347689286] [adapted_subscriber]: Received point 1 x: 1 y 2 z 3, Point 2 x: 4 y 5 z 3

If both the publisher and the subscriber use the same type, no conversion is applied and the memory is shared between the nodes. In the case of mismatching types, a conversion is applied and no memory-sharing is possible (intra-process memory copy happens, which is still better than serialzation).

NOTE: if using old versions of rclcpp (Sept 2023 patch) and the intra process communication is not disabled, messages will be silently dropped if the types are mismatching.

# Run without intra process communication

source install/setup.bash

ros2 launch test_type_adaptation test_type_adaptation_no_intra_process.launch.py

## Example of output

[component_container-1] [INFO] [1693326031.993434726] [adapted_publisher]: Publishing to ADAPTED topic cloud with address: 0x55A6B4F2B160

[component_container-1] [INFO] [1693326031.993545362] [adapted_publisher]: Publishing to NOT ADAPTED topic cloud with address: 0x55A6B4F2B160

[component_container-1] Converting to ROS

[component_container-1] Converting to ROS

[component_container-1] Converting to PCL

[component_container-1] [INFO] [1693326031.993668634] [adapted_subscriber]: Adapted sub received cloud with address: 0x55A6B4F2B0D0
[component_container-1] [INFO] [1693326031.993688434] [adapted_subscriber]: Received point 1 x: 1 y 2 z 3, Point 2 x: 4 y 5 z 3

[component_container-1] [INFO] [1693326031.993716204] [adapted_subscriber]: Normal sub received cloud with address: 0x55A6B4F2B170
[component_container-1] [INFO] [1693326031.993728634] [adapted_subscriber]: Received point 1 x: 1 y 2 z 3, Point 2 x: 4 y 5 z 3

In the case of mismatching types, a conversion is applied. If both the publisher and the subscriber use a custom type, two conversions are applied.
