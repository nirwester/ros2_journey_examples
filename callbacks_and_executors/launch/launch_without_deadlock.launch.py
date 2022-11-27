from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    srv_node = Node(
        package='service_test_pkg',
        name='service_client',
        executable='test_client_exe',
        output='screen',
        parameters=[{'use_cb_group': True, 'multi_threaded_executor': True}]
    )
    client_node = Node(
        package='service_test_pkg',
        name='service_server',
        executable='test_server_exe',
        output='screen'
    )
    ld.add_action(srv_node)
    ld.add_action(client_node)
    return ld
