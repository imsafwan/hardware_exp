#!/usr/bin/env python3
import paramiko
import socket
import json
import select
import sys
import os
import subprocess
import time
import yaml
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

AOI_FILE = "aoi_status.txt"



def get_sim_time_from_file(filename="/home/safwan/hardware_exp/central/src/clock_log.txt", retry_delay=0.05, timeout=5.0):
    return time.time()
    

# === Helpers ===
def run_remote(host, user, passwd, cmd):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect(host, username=user, password=passwd)
        # Run in tmux to avoid blocking
        wrapped_cmd = f"tmux new-session -d -s auto_session '{cmd}'"
        ssh.exec_command(wrapped_cmd, get_pty=True)
    except Exception as e:
        print(f"SSH error for {host}: {e}")
    finally:
        ssh.close()

def create_socket(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    return sock

def pretty_print(agent, msg):
    print(f"\nMessage from {agent}: {msg}")

def all_success(status_dict):
    return status_dict and all(v.get("status") == "SUCCESS" for v in status_dict.values())

def load_aoi_status():
    

    # Path to scene.yaml
    scene_file = os.path.join("Inputs", "scene.yaml")

    # Load YAML
    with open(scene_file, "r") as f:
        scene_data = yaml.safe_load(f)

    # Build AOI status dictionary
    aoi_status = {}
    for node in scene_data.get("scenario", {}).get("nodes", []):
        if node.get("type") == "air_only":
            node_id = node["ID"]
            if node.get("time_last_service", 0.0) == 0.0:
                aoi_status[node_id] = "PENDING"
            else:
                aoi_status[node_id] = "COMPLETED"

    return aoi_status

def save_aoi_status(aoi_status):
    """Save AoI status dict to file."""
    with open(AOI_FILE, "w") as f:
        json.dump(aoi_status, f, indent=2)

def start_local_tasks():
    """Start UAV and UGV tasks on their respective computers via SSH (in tmux)."""
    try:
        # Kill old UAV session and start a new one remotely
        run_remote(UAV_HOST, UAV_USER, UAV_PASS, UAV_CMD)
            
        # Kill old UGV session and start a new one remotely
        run_remote(UGV_HOST, UGV_USER, UGV_PASS, UGV_CMD)

        print(" Started UAV and UGV tasks in remote tmux sessions")

    except Exception as e:
        print(f" Failed to start tasks: {e}")
        
    

def start_planners():
    """Start the planner processes in tmux sessions."""
    try:
        # Kill any existing planner session first
        subprocess.run(["tmux", "kill-session", "-t", "planners"], stderr=subprocess.DEVNULL)
        
        # Create new tmux session for planners
        subprocess.run([
            "tmux", "new-session", "-d", "-s", "planners",
            "python3 initial_plan.py"
        ], check=True)
        
        # Split the window and run the second planner
        subprocess.run([
            "tmux", "split-window", "-t", "planners",
            "python3 replanner1.py"
        ], check=True)
        
        # Optional: Set window layout (horizontal split)
        subprocess.run([
            "tmux", "select-layout", "-t", "planners", "even-horizontal"
        ])
        
        print("Started planners in tmux session 'planners'")
        
    except subprocess.CalledProcessError as e:
        print(f"Error starting planners in tmux: {e}")
        # Fallback to background processes
        try:
            subprocess.Popen(["python3", "initial_plan.py"])
            subprocess.Popen(["python3", "replanner1.py"])
            print("Fallback: Started planners as background processes")
        except Exception as fallback_error:
            print(f"Fallback also failed: {fallback_error}")
            
    except Exception as e:
        print(f"Unexpected error starting planners: {e}")

def cleanup():
    """Kill all tmux sessions."""
    try:
        # Kill all tmux sessions
        run_remote(UGV_HOST, UGV_USER, UGV_PASS, f"tmux kill-session -t {UGV_SESSION}")
        run_remote(UAV_HOST, UAV_USER, UAV_PASS, f"tmux kill-session -t {UAV_SESSION}")
        subprocess.run(["tmux", "kill-session", "-t", "planners"], stderr=subprocess.DEVNULL)
        
        
        print("\n Cleaned up all tmux sessions \n")

        time.sleep(3)
        
    except Exception as e:
        print(f"Cleanup error: {e}")





def update_scene_yaml(visit_status, file_path="Inputs/scene.yaml"):
    # Load scene.yaml
    with open(file_path, "r") as f:
        scene = yaml.safe_load(f)

    nodes = scene["scenario"]["nodes"]

    # -----------------------------------------------------
    # 1. Update AOI visit times
    # -----------------------------------------------------
    for k, v in visit_status.items():
        for node in nodes:
            if node["ID"] == k:
                node["time_last_service"] = float(v)

    # -----------------------------------------------------
    # 2. Load last UAV landing action
    # -----------------------------------------------------
    with open("output/uav_actions.yaml", "r") as f:
        actions = yaml.safe_load(f)

    land_act = actions["sortie 1"][-1]
    assert land_act["type"] == "land_on_UGV", f"Unexpected last action: {land_act['type']}"

    # Update agent position (assuming single agent)
    for agent in scene["agents"]:
        agent["location"]["x"] = land_act["location"]["lat"]
        agent["location"]["y"] = land_act["location"]["lon"]

    # -----------------------------------------------------
    # 3. Ask user to add new points
    # -----------------------------------------------------
    # add_new = input("Do you want to add a new point? (Y/N): ").strip().upper()

    # while add_new == "Y":
    #     print("\nEnter details for the new node:")

    #     new_id = input("  Node ID: ").strip()
    #     new_x = float(input("  X coordinate: "))
    #     new_y = float(input("  Y coordinate: "))
    #     new_type = input("  Type (air_only/path_only/etc): ").strip()

    #     new_node = {
    #         "ID": new_id,
    #         "type": new_type,
    #         "location": {"x": new_x, "y": new_y},
    #         "time_last_service": 0.0
    #     }

    #     scene["scenario"]["nodes"].append(new_node)
    #     print(f"Added new node: {new_node}")

    #     add_new = input("\nAdd another new point? (Y/N): ").strip().upper()

    # -----------------------------------------------------
    # 4. Save back updated scene.yaml
    # -----------------------------------------------------
    with open(file_path, "w") as f:
        yaml.safe_dump(scene, f, sort_keys=False)

    print(f"Updated {file_path}")








import socket, json, time, select, sys, threading, subprocess

# === Main ===
def main():

    # Load existing AoI status
    aoi_status = load_aoi_status()
    visit_status = {}

    # Create UDP sockets
    sock_uav = create_socket(UAV_PORT)
    sock_ugv = create_socket(UGV_PORT)
    print(f"Central listening on ports -> UAV:{UAV_PORT}, UGV:{UGV_PORT}")

    
    # 0. Solve the scene once before starting
    subprocess.run(["python3", "solver_RL.py"], check=True)
    time.sleep(1)
    

    mission_start_time = get_sim_time_from_file()
    sortie_start_time = mission_start_time

    try:

        while True:
            
            print('<--------- Executing one sortie --------->')

            # 1. Start UAV-UGV tasks
            start_local_tasks()
            time.sleep(5)

            # 2. Send initial plan and start replanner
            start_planners()

            # 3. Listen for status updates
            uav_task_status, ugv_task_status = {}, {}
            last_msgs = {"UAV": None, "UGV": None}

            # --- Inner loop for a sortie ---
            one_sortie_complete = False
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

                    # Only print if different from last message
                    if msg != last_msgs[agent]:
                        pretty_print(agent, msg)
                        last_msgs[agent] = msg

                    # Update status
                    if "uav_task_status" in msg:
                        uav_task_status = msg["uav_task_status"]
                    if "ugv_task_status" in msg:
                        ugv_task_status = msg["ugv_task_status"]
                    if "aoi_status" in msg:
                        for k, v in msg["aoi_status"].items():
                            if str(v).upper() == "COMPLETED":
                                if aoi_status.get(k) != "COMPLETED":
                                    aoi_status[k] = "COMPLETED"
                                    visit_status[k] = round(get_sim_time_from_file() - mission_start_time, 3)

                    # 4. Check if both tasks completed successfully
                    if all_success(uav_task_status):# and all_success(ugv_task_status):
                        print("\n\n---------------- Sortie completed! -----------------\n\n")
                        print('\n sortie completion time: {} sec'.format(get_sim_time_from_file() - sortie_start_time))
                        sortie_start_time = get_sim_time_from_file()
                        print("visit status --->", visit_status)
                        
                        # Save AoI status and cleanup
                        save_aoi_status(aoi_status)
                        cleanup()

                        # 4.a Update the scene.yaml
                        update_scene_yaml(visit_status)

                        # === Run recharge + replan in parallel ===
                        def recharge_thread():
                            rs_t = get_sim_time_from_file()
                            while True:
                                if (get_sim_time_from_file() - rs_t) >= 30:  # 30 sec recharge
                                    break
                                time.sleep(0.5)
              
                        def replan_thread():
                            if all(v == "COMPLETED" for v in aoi_status.values()):
                                print("All points are visited")
                            else:
                                subprocess.run(["python3", "solver_RL.py"], check=True)

                        t1 = threading.Thread(target=recharge_thread, daemon=True)
                        t2 = threading.Thread(target=replan_thread, daemon=True)
                        t1.start()
                        t2.start()
                        t1.join()
                        t2.join()

                        print("\n\n---------------- replan & recharge completed! -----------------\n\n")
                        one_sortie_complete = True
                        break  # break inner sortie loop
                
                if one_sortie_complete:
                    # Clear any pending messages from UAV/UGV sockets
                    for s in [sock_uav, sock_ugv]:
                        s.setblocking(False)
                        while True:
                            try:
                                data, addr = s.recvfrom(BUFFER_SIZE)
                                msg = json.loads(data.decode())
                                # (optional) log discarded message:
                                # print(f"Cleared pending message from {addr}: {msg}")
                            except (BlockingIOError, json.JSONDecodeError):
                                break
                        s.setblocking(True)  # restore blocking mode
                    break  


            # After sortie, check if mission is complete
            if all(v == "COMPLETED" for v in aoi_status.values()):
                print("Mission end")
                print('Mission took around {} sec'.format(get_sim_time_from_file() - mission_start_time))
                break  # break outer while loop
        
            else:
                print('mission continuing')

    except KeyboardInterrupt:
        print("\nCentral server interrupted by user. Cleaning up...")
        cleanup()
        sys.exit(1)


if __name__ == "__main__":
    main()