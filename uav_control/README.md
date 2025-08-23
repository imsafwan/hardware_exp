## UAV Control



1. Go to directory :
``` bash
cd ~/uav_control/src
```

2. Note: always check if the drone's ekf origin is correct. If the drone is shifted by hands , there are chances that ekf origin is not correct hence will the navigation be impacted by this.

Resatrting the drone will solve this.
 

3. Run UAV's code:

``` bash
./uav_task.sh
```

- This runs the ```uav_action_test_v3.py``` to execute the actions.
- It also runs ```task_status_broadcaster.py``` that broadcasts actions' statuses for coordination between UAV and UGV.

