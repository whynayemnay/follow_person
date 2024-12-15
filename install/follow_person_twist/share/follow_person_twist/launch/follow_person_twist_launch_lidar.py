from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow_person_twist',
            executable='lidar',
            name='lidar',
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
            executable='person_detection_lidar',
            name='person_detection_lidar',
            output='screen'
        ),
        Node(
            package='follow_person_twist',
            executable='person_follow',
            name='person_follow',
            output='screen'
        ),

    ])