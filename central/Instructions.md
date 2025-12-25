# How to Run a Physical Experiment with Hardware

## UGV Side (Run on the UGV Onboard Computer, e.g., Primary Computer of Husky)

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

## UAV Side (Run on UAV Jetson Computer)

1. Fix the Jetson computer's time (as it may be messed up):
   ```bash
   sudo date -s "2025-11-03 14:30:00"
   ```

## Central Computer

1. Provide your input as `scene.yaml` inside the `src/Inputs/` folder to describe your scene.

2. Run the following:
   ```bash
   cd src/
   python3 central_check_v5.py
   ```
