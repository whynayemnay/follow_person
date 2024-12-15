from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow_person_twist',
            executable='lidar_transform',
            name='lidar_transform',
            output='screen'
        ),
        Node(
            package='follow_person_twist',
            executable='path_playback',
            name='path_playback',
            output='screen'
        ),
        Node(
            package='follow_person_twist',
            executable='path_record',
            name='path_record',
            output='screen'
        ),
        Node(
            package='follow_person_twist',
            executable='person_detection',
            name='person_detection',
            output='screen'
        ),
        Node(
            package='follow_person_twist',
            executable='person_follow',
            name='person_follow',
            output='screen'
        ),

    ])