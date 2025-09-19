from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='workshop_main',
            executable='minimal_publisher', # This matches the entry point name
            name='my_publisher_node',
            output='screen'
        ),
        Node(
            package='workshop_main',
            executable='minimal_subscriber', # This matches the entry point name
            name='my_subscriber_node',
            output='screen'
        )
    ])