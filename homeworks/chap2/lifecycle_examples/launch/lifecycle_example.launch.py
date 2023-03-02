import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    this_pkg_name = "lifecycle_examples"
    container = ComposableNodeContainer(
        name='image_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
                ComposableNode(
                    package=this_pkg_name,
                    plugin='lifecycle_examples::Manager',
                    name='manager',
                    parameters=[{'managed_nodes': ["node1", "node2"]}]),
                ComposableNode(
                    package=this_pkg_name,
                    plugin='lifecycle_examples::ManagedNode',
                    name='node1'),
                ComposableNode(
                    package=this_pkg_name,
                    plugin='lifecycle_examples::ManagedNode',
                    name='node2')
        ],
        output='both',
    )

    return launch.LaunchDescription([container])
