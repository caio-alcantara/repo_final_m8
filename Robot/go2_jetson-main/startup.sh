#!/bin/bash

echo "=== Starting Robot System ==="

# Ensure environment is set
cd ~/go2_ros2_toolbox
source setup.sh

# Kill everything first for clean start
echo "Cleaning up old processes..."
pkill -9 -f zenoh-bridge
pkill -9 -f nav2
pkill -9 -f slam_toolbox
sleep 2

# Clear any stale DDS discovery state
rm -rf ~/.ros/fastrtps_* 2>/dev/null
rm -rf /tmp/cyclonedds_* 2>/dev/null

# Start Zenoh bridge FIRST (it needs to be ready before ROS nodes)
echo "Starting Zenoh bridge..."
zenoh-bridge-ros2dds -l tcp/0.0.0.0:7447 -c ~/go2_ros2_toolbox/zenoh-config.json5 > /tmp/zenoh.log 2>&1 &
ZENOH_PID=$!

# Wait for Zenoh to be fully up
sleep 3

# Verify Zenoh is running
if ! kill -0 $ZENOH_PID 2>/dev/null; then
    echo "ERROR: Zenoh bridge failed to start!"
    cat /tmp/zenoh.log
    exit 1
fi

echo "Zenoh bridge started (PID: $ZENOH_PID)"

# Now start ROS2 nodes
echo "Starting Nav2 and SLAM..."
ros2 launch go2_core go2_startup.launch.py

# Cleanup on exit
trap "echo 'Shutting down...'; kill $ZENOH_PID; exit 0" SIGINT SIGTERM
