# Experimental Trial: Gazebo Simulation Guide
Follow these steps to initialize the UAV-UGV simulation environment. Note: Run each step in a new terminal window or tab.

**Prerequisites:** Ensure PX4 Autopilot and Clearpath ROS 2 packages are installed and built before running these steps.

1. Launch PX4 SITL Drone
Starts the PX4 Software In The Loop (SITL) simulation with the custom world.
```bash
cd PX4-Autopilot/
PX4_GZ_WORLD=my_world/exp_world_v1 make px4_sitl
```
2. Start MicroXRCEAgent
Establishes communication between PX4 and ROS 2.

```bash
MicroXRCEAgent udp4 -p 8888
```
3. Launch Clearpath UGV
Spawns the Clearpath UGV in the Gazebo environment at the specified coordinates.

```bash
ros2 launch clearpath_gz simulation.launch.py x:=0.0 y:=2.0
```
4. Initialize UGV Packages
Launches the UGV localization and Nav2 stacks.

```bash
cd ~/hardware_exp
bash central/src/ready_bash.sh
```
5. Initialize UAV Camera Bridge
Creates the virtual camera bridge between Gazebo and ROS for the UAV.

```bash
cd ~/hardware_exp
bash central/src/ready_bash2.sh
```
6. Run Simulation Clock
Starts the custom clock writer to sync simulation time.

```bash
cd ~/hardware_exp/central/src/
python3 clock_writer.py
```
7. Run Main Experiment Script
Executes the central control logic for the trial.

```bash
cd ~/hardware_exp/central/src/
python3 central_check_v5.py
```