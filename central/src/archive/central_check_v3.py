#!/usr/bin/env python3
import paramiko
import socket
import json
import select
import sys
import os

# === Config ===
UAV_HOST = "192.168.10.120"
UAV_USER = "safwan"
UAV_PASS = "jetson"
UAV_SESSION = "uav_task"
UAV_CMD = (
    f'tmux kill-session -t {UAV_SESSION} 2>/dev/null; '
    f'tmux new-session -d -s {UAV_SESSION} "cd ~/uav_control/src && ./uav_task.sh; exec bash"'
)

UGV_HOST = "192.168.10.77"
UGV_USER = "administrator"
UGV_PASS = "clearpath"
UGV_SESSION = "ugv_task"
UGV_CMD = (
    f'tmux kill-session -t {UGV_SESSION} 2>/dev/null; '
    f'tmux new-session -d -s {UGV_SESSION} "source /opt/ros/humble/setup.bash && cd ~/ugv_control/src && ./ugv_task.sh; exec bash"'
)

UAV_PORT = 5055
UGV_PORT = 5056
BUFFER_SIZE = 4096

AOI_FILE = "aoi_status.txt"  # text file for AoI status

# === Helpers ===
def run_remote(host, user, passwd, cmd):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(host, username=user, password=passwd)
    ssh.exec_command(cmd)
    ssh.close()

def create_socket(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    return sock

def pretty_print(agent, msg):
    print(f"\n Message from {agent}: {msg}")

def all_success(status_dict):
    return status_dict and all(v.get("status") == "SUCCESS" for v in status_dict.values())

def load_aoi_status():
    """Load AoI status dict from file if it exists."""
    if os.path.exists(AOI_FILE):
        with open(AOI_FILE, "r") as f:
            try:
                return json.load(f)
            except json.JSONDecodeError:
                return {}
    return {}

def save_aoi_status(aoi_status):
    """Save AoI status dict to file."""
    with open(AOI_FILE, "w") as f:
        json.dump(aoi_status, f, indent=2)

# === Main ===
def main():
    # Load existing AoI status (or start empty)
    aoi_status = load_aoi_status()

    # Create UDP sockets
    sock_uav = create_socket(UAV_PORT)
    sock_ugv = create_socket(UGV_PORT)
    print(f"Central listening on ports -> UAV:{UAV_PORT}, UGV:{UGV_PORT}")

    uav_task_status, ugv_task_status = {}, {}
    last_msgs = {"UAV": None, "UGV": None}  # cache last messages

    while True:
        readable, _, _ = select.select([sock_uav, sock_ugv], [], [])
        for s in readable:
            try:
                data, addr = s.recvfrom(BUFFER_SIZE)
                msg = json.loads(data.decode())
            except json.JSONDecodeError:
                print(f"Invalid JSON from {addr}: {data}")
                continue

            agent = "UAV" if s is sock_uav else "UGV"

            # Only print if different from last message for this agent
            if msg != last_msgs[agent]:
                pretty_print(agent, msg)
                last_msgs[agent] = msg

            if "uav_task_status" in msg:
                uav_task_status = msg["uav_task_status"]
            if "ugv_task_status" in msg:
                ugv_task_status = msg["ugv_task_status"]

            if "aoi_status" in msg:
                for k, v in msg["aoi_status"].items():
                    if v.upper() == "COMPLETED":  # normalize check
                        aoi_status[k] = "COMPLETED"

            # Check if both tasks finished successfully
            if all_success(uav_task_status) and all_success(ugv_task_status):
                print("\n\n ----------------  sortie completed! ----------------- \n\n")

                # Kill remote tmux sessions
                run_remote(UGV_HOST, UGV_USER, UGV_PASS, f"tmux kill-session -t {UGV_SESSION}")
                run_remote(UAV_HOST, UAV_USER, UAV_PASS, f"tmux kill-session -t {UAV_SESSION}")

                # Save AoI status to file
                save_aoi_status(aoi_status)

                sys.exit(0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCentral server interrupted by user. Exiting...")
