#!/bin/bash

# Source ROS 2 and your workspace
source /opt/ros/humble/setup.bash
source ~/Documents/ros_ws/install/setup.bash

# Victim Launcher
gnome-terminal --title="victims.launch.py" -- bash -c "
  ros2 launch projects victims.launch.py \
    use_rviz:=false \
    use_gui:=false \
    spawn_shelfino:=true \
    generate_new_map_config:=false \
    gen_map_params_file:=/home/sincerejuliya/Documents/ros_ws/src/Shelfino_ROS2/map_pkg/config/victims1.yaml;
  exec bash"
