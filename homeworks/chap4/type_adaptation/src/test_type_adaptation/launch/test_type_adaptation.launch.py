from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription


def generate_launch_description():
    """Generate launch description"""

    launch_description = LaunchDescription()

    """Running both nodes in the same container"""
    container = ComposableNodeContainer(
        name="test_type_adaptation_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # Publisher
            ComposableNode(
                package="test_type_adaptation",
                plugin="journey::AdaptedPublisher",
                name="adapted_publisher",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # Subscriber
            ComposableNode(
                package="test_type_adaptation",
                plugin="journey::AdaptedSubscriber",
                name="adapted_subscriber",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="both",
    )

    launch_description.add_action(container)

    return launch_description
