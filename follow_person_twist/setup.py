from setuptools import find_packages, setup

package_name = 'follow_person_twist'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/follow_person_twist/launch', ['launch/follow_person_twist_launch.py']),
        ('share/follow_person_twist/launch', ['launch/follow_person_twist_launch_lidar.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kirils',
    maintainer_email='kirils@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_transform = follow_person_twist.lidar_transform:main',
            'path_playback = follow_person_twist.path_playback:main',
            'path_record = follow_person_twist.path_record:main',
            'person_detection_lidar = follow_person_twist.person_detection_lidar:main',
            'person_detection = follow_person_twist.person_detection:main',
            'person_follow = follow_person_twist.person_follow:main',
            'lidar = follow_person_twist.lidar:main'
        ],
    },
)
