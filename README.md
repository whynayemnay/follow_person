# follow_person

Make sure you have installed ultralytics on the correct version of python that is also associated with the ROS2 otherwise you will have Module not found errors.

Run colcon build to build the package and source it.

Start the robot in the docking position. The nav2 and either SLAM or AMCL also need to be running. The package is set to run with AMCL by default, otherwise there are some changes to subscribed topics in path_record.py file needed.

run: ros2 launch follow_person follow_person_launch.py / ros2 launch follow_person follow_person_launch_lidar.py to run all the nodes

ros2 service call /start_playback std_srvs/srv/Trigger "{}"

ros2 service call /start_recording std_srvs/srv/Trigger "{}"

ros2 service call /stop_recording std_srvs/srv/Trigger "{}"

ros2 service call /start_navigation std_srvs/srv/Trigger "{}"

ros2 service call /stop_navigation std_srvs/srv/Trigger "{}"

these are the service call commands that are available, what they do is self-explanatory. Before starting playback stop the navigation otherwise navigation will override the playback process.


## warehouse.sdf

The script to create a model of a person with a walking animation. Either replace the already existing warehouse.sdf file that comes with TB4 simulation under turtlebot4_simulator/turtlebot4_ignition_bringup/worlds
/warehouse.sdf or add the script starting from lines 258-361 to the same file. This script should automatically dowload the needed model and animation for the model from the official Gazebo hosting platform. After loading the world the model will spawn and go through set waypoints.

