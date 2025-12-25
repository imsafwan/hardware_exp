# UAV Control System

This project provides autonomous control for UAV (drone) operations, including mission execution, GPS navigation, photo capture, and vision-based landing.

## Overview

The system consists of several Python scripts that work together to execute UAV missions:

- **uav_actions.py**: Core UAV action functions (takeoff, navigation, landing, photo capture)
- **uav_action_test_v4.py**: Main mission executor that receives action plans via UDP and executes them
- **gps_logger.py**: Records UAV GPS positions to CSV files
- **task_status_broadcaster.py**: Broadcasts UAV and Area of Interest (AoI) status updates
- **vision_landing.py**: Vision-based landing using ArUco markers
- **uav_task.sh**: Shell script to launch the main UAV task and status broadcaster


## Logs

- Mission logs are stored in `src/logs/`
- GPS data in `src/gps_logs/`
- Photos captured during missions in `src/photo/`



