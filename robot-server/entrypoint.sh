#!/bin/bash

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash

# Set FastDDS as the RMW implementation
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds_config/fastdds_profile_pi.xml
export ROS_DOMAIN_ID=0

# Start micro-ROS agent in the background
echo "Starting micro-ROS agent..."
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 --verbose 4 &

# Start HMI
echo "Starting HMI web server..."
cd /app && exec uvicorn main:app --host 0.0.0.0 --port 8000