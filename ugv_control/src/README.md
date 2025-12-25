# UGV Control System

This project provides autonomous control for UGV (Unmanned Ground Vehicle) operations, including coordination with UAVs for takeoff and landing, GPS navigation, and mission execution.

## Overview

The system consists of several Python scripts and shell scripts that work together to execute UGV missions using ROS2 and Nav2:

- **ugv_action_test_v4.py**: Main mission executor that receives action plans from a YAML configuration, executes navigation tasks, communicates with UAVs via UDP for takeoff/landing coordination, and reports status updates.
- **gps_logger.py**: ROS2 node that subscribes to GPS data and logs UGV positions (latitude, longitude, altitude) to timestamped CSV files at 1 Hz.
- **task_status_broadcaster.py**: Broadcasts UGV mission status and action updates via UDP to a central manager or other systems.
- **ugv_task.sh**: Shell script that launches the main UGV task executor, status broadcaster, and GPS logger in parallel.
- **kickstart.sh**: Initializes ROS2 environment by launching EKF localization, Nav2 navigation stack, and GPS correction services in separate tmux sessions.


## Logs and Data

- Mission execution logs are stored in `src/logs/` with timestamped filenames.
- GPS position data is logged to `src/gps_logs/` as CSV files.
- Temporary status files are maintained in `src/tmp/`.



## How to start the experiment ?

Run on the UGV Onboard Computer, e.g., Primary Computer of Husky.

1. Ensure the UGV is facing east (this helps with localization sensor fusion).
Restart the robot to reinitialize the EKF base frame. Restarting can be skipped if the robot is freshly started, but must be done if the robot has been run previously, as the EKF localization may be corrupted:
   ```bash
   sudo systemctl restart clearpath-robot.service
   ```

2. Record the initial GPS location at the start of the experiment:
   ```bash
   ros2 topic echo /a200_1093/sensors/gps_0/navsatfix --once
   ```
   Insert this GPS value at line 469 of the `src/ugv_action_test_v4.py` script

3. Start EKF-based localization on the Clearpath Husky robot:
   ```bash
   ./kickstart.sh
   ```

4. Launch Nav2 for navigation on the Clearpath Husky robot:
   ```bash
   ros2 launch clearpath_nav2_demos nav2.launch.py setup_path:=$HOME/clearpath/
   ```