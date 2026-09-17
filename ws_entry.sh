#!/bin/bash
echo "Setting up ROS2 Jazzy environment..."
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="./fastdds_config/fastdds_profile_desktop.xml"
cd ./ros_ws
colcon build
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "Workspace sourced successfully!"
else
    echo "Warning: install/setup.bash not found."
fi
cd ..
echo "ROS2 environment ready."