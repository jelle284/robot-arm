#!/bin/bash
cd "$(dirname "$0")"

echo "🔧 Setting up ROS2 Jazzy environment..."
source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID=0
echo "🎯 ROS_DOMAIN_ID set to 0"

# Set discovery server to find ROS2 topics across network
export ROS_DISCOVERY_SERVER=192.168.0.10:11811
echo "🚀 ROS_DISCOVERY_SERVER set to 192.168.0.10:11811"

cd ./ros_ws
echo "🏗️ Building workspace..."
colcon build

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✅ Workspace sourced successfully!"
else
    echo "⚠️ Warning: install/setup.bash not found. Did the build fail?"
fi

echo "🚀 Entering ROS2 Console Window..."
exec bash --rcfile <(cat ~/.bashrc; echo 'export PS1="[ROS2 Jazzy WS] $PS1"')