from setuptools import find_packages, setup

package_name = 'follow_person'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/follow_person/launch', ['launch/follow_person_launch.py']),
        ('share/follow_person/launch', ['launch/follow_person_launch_lidar.py'])
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
            'lidar_transform = follow_person.lidar_transform:main',
            'nav2_goal = follow_person.nav2_goal:main',
            'path_playback = follow_person.path_playback:main',
            'path_record = follow_person.path_record:main',
            'person_detection_nav = follow_person.person_detection_nav:main',
            'person_detection_lidar_nav = follow_person.person_detection_lidar_nav:main'
        ],
    },
)
