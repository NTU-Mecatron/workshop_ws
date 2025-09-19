from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='',     # Fill here
            executable='',  # Fill here
            name='',        # Fill here
            output='screen'
        ),
        Node(
            package='',     # Fill here
            executable='',  # Fill here
            name='',        # Fill here 
            output='screen'
        )
    ])