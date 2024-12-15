# follow_person

Make sure you have installed ultralytics on the correct version of python that is also associated with the ROS2 otherwise you will have Module not found errors.

Run colcon build to build the package and source it.

Start the robot in the docking position. And AMCL running is set by default, otherwise there are some changes to subscribed topics in path_record.py file needed.

run: ros2 launch follow_person follow_person_launch.py / ros2 launch follow_person follow_person_launch_lidar.py to run all the nodes

ros2 service call /start_playback std_srvs/srv/Trigger "{}"

ros2 service call /start_recording std_srvs/srv/Trigger "{}"

ros2 service call /stop_recording std_srvs/srv/Trigger "{}"

ros2 service call /start_navigation std_srvs/srv/Trigger "{}"

ros2 service call /stop_navigation std_srvs/srv/Trigger "{}"

these are the service call commands that are available, what they do is self-explanatory. Before starting playback stop the navigation otherwise navigation will override the playback process.
