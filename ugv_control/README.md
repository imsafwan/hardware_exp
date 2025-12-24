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



## Usage

1. Before you start the experiment from central manager, make sure to run the ROS2 localization and navigation services with this:

   ```bash
   ./src/kickstart.sh
   ```

The system will begin executing actions defined in the configuration file and coordinate with UAVs as needed.


