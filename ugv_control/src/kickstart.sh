#!/bin/bash
# bsh.sh - start robot localization, Nav2, and GPS correction in tmux with delays

# Source ROS 2 and your workspace
#source /opt/ros/humble/setup.bash
#source ~/navsat_ws/install/setup.bash

#kill existing if any
tmux kill-server


sleep 2

# Start EKF session
tmux new-session -d -s ekf "ros2 launch clearpath_navsat ekf_launch.launch.py"
echo "Started EKF session."
sleep 2

# Start GPS correction session
tmux new-session -d -s gps "cd ~/navsat_ws/src/clearpath_navsat/clearpath_navsat/launch && python3 correct_gps.py"
echo "Started GPS correction session."
sleep 2

# Start Nav2 session
tmux new-session -d -s nav2 "ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=$HOME/clearpath/"
echo "Started Nav2 session."
sleep 8



echo "All tmux sessions started: ekf, nav2, gps"
echo "Attach with: tmux attach -t <session_name>"
