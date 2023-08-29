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

[component_container-1] [INFO] [1693326904.073274834] [adapted_publisher]: Publishing to ADAPTED topic cloud with address: 0x562FD76C4990

[component_container-1] [INFO] [1693326904.073504960] [adapted_publisher]: Publishing to NOT ADAPTED topic cloud with address: 0x562FD76C4900

[component_container-1] [INFO] [1693326904.073724720] [adapted_subscriber]: Adapted sub received cloud with address: 0x562FD76C4990

If both the publisher and the subscriber use the same type, no conversion is applied and the memory is shared between the nodes. In the case of mismatching types, a conversion is applied and no memory-sharing is possible.

NOTE: in the latter case, if the intra process communication is not disabled, the messages will be silently dropped.

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

[component_container-1] [INFO] [1693326031.993716204] [adapted_subscriber]: Normal sub received cloud with address: 0x55A6B4F2B170

In the case of mismatching types, a conversion is applied. If both the publisher and the subscriber use a custom type, two conversions are applied.
