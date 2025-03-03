from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aiagent',
            executable='writer',
            name='writer',
            parameters=[
            ]
        ),
        Node(
            package='aiagent',
            executable='talker',
            name='talker',
            parameters=[
            ]
        ),
        Node(
            package='aiagent',
            executable='recorder',
            name='recorder',
            parameters=[
                {'threshold': 5000},
            ]
        ),
        Node(
            package='utils',
            executable='manager',
            name='manager',
            parameters=[
                {'nodes': 'manager,recorder,talker,writer'}
            ]
        )
    ])