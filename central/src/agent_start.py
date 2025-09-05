#!/usr/bin/env python3

''' This script starts the UAV and UGV tasks (action_test.py and broadcasrter.py) remotely via SSH  '''

import paramiko
import socket
import json
import select
import sys



# === Config ===
UAV_HOST = "192.168.10.120"
UAV_USER = "safwan"
UAV_PASS = "jetson"
UAV_SESSION = "uav_task"
UAV_CMD = (f'tmux kill-session -t {UAV_SESSION} 2>/dev/null; ' 
          f'tmux new-session -d -s {UAV_SESSION} "cd ~/uav_control/src && ./uav_task.sh; exec bash"')

UGV_HOST = "192.168.10.77"
UGV_USER = "administrator"
UGV_PASS = "clearpath"
UGV_SESSION = "ugv_task"
UGV_CMD = (
    f'tmux kill-session -t {UGV_SESSION} 2>/dev/null; '
    f'tmux new-session -d -s {UGV_SESSION} "source /opt/ros/humble/setup.bash  && cd ~/ugv_control/src && ./ugv_task.sh; exec bash"'
)

UAV_PORT = 5055
UGV_PORT = 5056
BUFFER_SIZE = 4096

# === Helpers ===
def run_remote(host, user, passwd, cmd):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(host, username=user, password=passwd)
    ssh.exec_command(cmd)
    ssh.close()


# === Main ===
def main():

    # Start tasks on both agents
    print("🚀 Starting UGV task remotely...")
    run_remote(UGV_HOST, UGV_USER, UGV_PASS, UGV_CMD)

    print("🚀 Starting UAV task remotely...")
    run_remote(UAV_HOST, UAV_USER, UAV_PASS, UAV_CMD)

    

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n🛑 Central server interrupted by user. Exiting...")
