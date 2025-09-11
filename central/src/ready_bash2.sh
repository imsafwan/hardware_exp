#!/bin/bash
# Run camera bridge and writer for UAV MAVSDK setup

# Move into project folder
cd ~/uav_mavsdk || exit 1

# Load v4l2loopback module if not already loaded
echo "Loading v4l2loopback..."
sudo modprobe v4l2loopback devices=1 video_nr=0 card_label="VirtualCam"

# Start camera bridge in the background
echo "Starting camera_bridge.py..."
python3 camera_bridge.py &

# Give it a short delay to initialize
sleep 2

# Start camera writer
echo "Starting camera_write.py..."
python3 camera_write.py
