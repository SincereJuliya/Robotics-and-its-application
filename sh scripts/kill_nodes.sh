#!/bin/bash

# Set ROS 2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Killing all ROS 2 nodes and gzserver processes..."

pkill -f "/opt/ros/humble/bin/ros2"
pkill -f "/opt/ros/humble"
pkill -9 -f "/opt/ros/humble/bin/ros2"
pkill ros2
pkill -f "gzserver /home/sincerejuliya/"
pkill -9 -f "gzserver /home/sincerejuliya/"
pkill -f "/home/sincerejuliya/Documents/ra/src/"
pkill -9 -f "/home/sincerejuliya/Documents/ra/src/"

pkill -f "ros2 launch projects victims.launch.py"
pkill -f "ros2 run robot_planning mapGeneratorNode"
pkill -f "ros2 run robot_planning taskPlannerNode"
pkill -f "ros2 run robot_planning motionPlannerNode"
pkill -f "ros2 run robot_planning followPath"

echo "Done."
  
