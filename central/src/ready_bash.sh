#!/bin/bash

tmux kill-server

tmux new-session -d -s husky -n SLAM "cd ~/husky_ws_3; source install/setup.bash; ros2 launch clearpath_nav2_demos slam.launch.py setup_path:=$HOME/clearpath/ use_sim_time:=true"
tmux split-window -h -t husky "cd ~/husky_ws_3; source install/setup.bash; sleep 2; ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=$HOME/clearpath/ use_sim_time:=true"
tmux split-window -v -t husky "cd ~/husky_ws_3; source install/setup.bash; sleep 7; ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_0000"
tmux attach -t husky
