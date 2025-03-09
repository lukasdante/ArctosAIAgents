from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cancontrol',
            executable='actor',
            namespace='actor',
        ),
        Node(
            package='utils',
            executable='manager',
            name='manager',
            parameters=[
                {'nodes': 'actor,manager,X,Y,Z,A,B,C,G'}
            ]
        )
    ])