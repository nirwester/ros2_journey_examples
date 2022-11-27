import launch
import os
import yaml
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description with both nodes ran by the same executor"""

    container = ComposableNodeContainer(
            name='image_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='service_test_pkg',
                    plugin='ServiceServer',
                    name='front_camera',
                    output='screen'),
                ComposableNode(
                    package='service_test_pkg',
                    plugin='ServiceClient',
                    namespace='perception',
                    output='screen')
            ],
            output='both',
    )

    return launch.LaunchDescription([container])
