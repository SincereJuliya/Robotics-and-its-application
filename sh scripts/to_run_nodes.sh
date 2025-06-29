#!/bin/bash

# Source ROS 2 and your workspace
source /opt/ros/humble/setup.bash
source ~/Documents/ra/src/install/setup.bash

# Map Generator
gnome-terminal --title="mapGeneratorNode" -- bash -c "
  ros2 run robot_planning mapGeneratorNode;
  exec bash"

# Task Planner
gnome-terminal --title="taskPlannerNode" -- bash -c "
  ros2 run robot_planning taskPlannerNode;
  exec bash"

# Motion Planner
gnome-terminal --title="motionPlannerNode" -- bash -c "
  ros2 run robot_planning motionPlannerNode;
  exec bash"

# Follow Path
gnome-terminal --title="followPath" -- bash -c "
  ros2 run robot_planning followPath \
    --ros-args -p use_sim_time:=true \
    -p controller_id:=FollowPath \
    -p goal_checker_id:=goal_checker;
  exec bash"

# Victim Launcher
gnome-terminal --title="victims.launch.py" -- bash -c "
  ros2 launch projects victims.launch.py \
    use_rviz:=false \
    use_gui:=false \
    use_sim_time:=true \
    spawn_shelfino:=true \
    gen_map_params_file:=/home/sincerejuliya/Documents/ra/src/Shelfino_ROS2/map_pkg/config/demos/victims/victims3.yaml;
  exec bash"
