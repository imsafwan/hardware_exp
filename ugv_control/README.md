## UGV Control


1. If needed, restart services:
``` bash
sudo systemctl restart clearpath-robot.service
```

2. start Localization and Nav2 for navigation:
``` bash
cd ~/ugv_control/src
./kickstart.sh
```

3. Note: always check if the origin of map->gps frame transformation is correct inside ```ugv_action_test.py```. We will automate this in future.

4. Run UGV's code:
``` bash
./ugv_task.sh
```

- This runs the ```ugv_action_test_v3.py``` to execute the actions.
- It also runs ```task_status_broadcaster.py``` that broadcasts actions' statuses for coordination between UAV and UGV.