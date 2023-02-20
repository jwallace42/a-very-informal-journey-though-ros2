import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name='managed_node_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='the_node',
                plugin='the_node::ManagedNodeOne',
                name="managed_node_one"),
            ComposableNode(
                package='the_node',
                plugin='the_node::ManagedNodeTwo',
                name="managed_node_two"),
            ComposableNode(
                package='the_node',
                plugin='the_node::ManagedNodeStarter',
                name="managed_node_starter")
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])