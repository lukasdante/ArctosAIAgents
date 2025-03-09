from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aiagent',
            executable='talker',
            name='talker',
        ),
        Node(
            package='cancontrol',
            executable='actor',
            namespace='actor',
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
            package='aiagent',
            executable='writer',
            name='writer'
        ),
        Node(
            package='aiagent',
            executable='agent',
            name='agent'
        ),
        Node(
            package='vision',
            executable='observer',
            name='observer',
        ),
        Node(
            package='utils',
            executable='manager',
            name='manager',
            parameters=[
                {'nodes': 'talker,actor,recorder,writer,agent,observer,manager,X,Y,Z,A,B,C,G'}
            ]
        )
    ])