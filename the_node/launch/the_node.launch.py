import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description with multiple components."""
    ld = LaunchDescription()

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
                name="managed_node_two")
        ],
        output='screen',
    )
    manager_node = Node(
        package="the_node",
        executable="the_node_executable"
    )

    ld.add_action(container)
    ld.add_action(manager_node)

    return ld