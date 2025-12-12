#!/bin/bash
# Simple test publisher for /chatter topic

cd "$(dirname "$0")"
source ros2_ws/install/setup.bash

echo "Publishing to /chatter topic..."
echo "Press Ctrl+C to stop"
echo ""

ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from ROS2!'" --rate 1
