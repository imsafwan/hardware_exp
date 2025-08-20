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
python3 ugv_action_test.py 
```

