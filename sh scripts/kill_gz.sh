#!/bin/bash

# Set ROS 2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Killing gzserver processes for reruning..."

#pkill -f "/opt/ros/humble/lib"
pkill -f "ros2 launch projects victims.launch.py"

echo "Done."
  
