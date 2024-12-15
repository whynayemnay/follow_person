from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='follow_person',
            executable='lidar_transform',
            name='lidar_transform',
            output='screen'
        ),
        Node(
            package='follow_person',
            executable='nav2_goal',
            name='nav2_goal',
            output='log'
        ),
        Node(
            package='follow_person',
            executable='path_playback',
            name='path_playback',
            output='screen'
        ),
        Node(
            package='follow_person',
            executable='path_record',
            name='path_record',
            output='screen'
        ),
        Node(
            package='follow_person',
            executable='person_detection_lidar_nav', 
            name='person_detection_lidar_nav',
            output='log'
        ),
    ])