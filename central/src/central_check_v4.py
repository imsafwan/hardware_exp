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

def get_sim_time_from_file(filename="/home/safwan/hardware_exp/central/src/clock_log.txt"):
    # keep a static variable to store last valid time
    if not hasattr(get_sim_time_from_file, "last_time"):
        get_sim_time_from_file.last_time = None

    try:
        with open(filename, "r") as f:
            line = f.read().strip()
            if line:
                get_sim_time_from_file.last_time = float(line)
                return get_sim_time_from_file.last_time
    except Exception as e:
        # could add logging here if you want
        pass

    # fallback to last valid time if reading failed
    return get_sim_time_from_file.last_time

# === Helpers ===
def run_remote(host, user, passwd, cmd):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect(host, username=user, password=passwd)
        ssh.exec_command(cmd)
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

    # Add an "unknown" AOI status
    aoi_status["AOI_unknown"] = "PENDING"
    return aoi_status

def save_aoi_status(aoi_status):
    """Save AoI status dict to file."""
    with open(AOI_FILE, "w") as f:
        json.dump(aoi_status, f, indent=2)

def start_local_tasks():
    """Start local UAV/UGV tasks in tmux sessions."""
    try:
        # Start UAV task
        subprocess.run([
            "tmux", "kill-session", "-t", UAV_SESSION
        ], stderr=subprocess.DEVNULL)
        
        subprocess.run([
            "tmux", "new-session", "-d", "-s", UAV_SESSION,
            "bash", "-c", 
            "cd ~/hardware_exp/uav_control/src && ./uav_task.sh; exec bash"
        ])
        
        # Start UGV task  
        subprocess.run([
            "tmux", "kill-session", "-t", UGV_SESSION
        ], stderr=subprocess.DEVNULL)
        
        subprocess.run([
            "tmux", "new-session", "-d", "-s", UGV_SESSION,
            "bash", "-c", 
            "cd ~/hardware_exp/ugv_control/src && ./ugv_task.sh; exec bash"
        ])
        
        print("Started UAV and UGV tasks in tmux sessions")
        
    except Exception as e:
        print(f"Error starting local tasks: {e}")

def start_planners():
    """Start the planner processes in separate terminals."""
    try:
        # Option 1: Using gnome-terminal (for Ubuntu/GNOME)
        subprocess.Popen([
            "gnome-terminal", "--", "bash", "-c", 
            "python3 initial_plan.py; exec bash"
        ])
        
        subprocess.Popen([
            "gnome-terminal", "--", "bash", "-c", 
            "python3 replanner1.py; exec bash"
        ])
        
        # Option 2: Using tmux split (alternative approach)
        # subprocess.run([
        #     "tmux", "new-session", "-d", "-s", "planners",
        #     "python3 dummy_planner.py"
        # ])
        # subprocess.run([
        #     "tmux", "split-window", "-t", "planners",
        #     "python3 replanner1.py"
        # ])
        
        print("Started planners in separate terminals")
        
    except Exception as e:
        print(f"Error starting planners: {e}")
        # Fallback to background processes if terminal opening fails
        try:
            subprocess.Popen(["python3", "initial_plan.py"])
            subprocess.Popen(["python3", "replanner1.py"])
            print("Fallback: Started planners as background processes")
        except Exception as fallback_error:
            print(f"Fallback also failed: {fallback_error}")
        
    

def cleanup():
    """Kill tmux sessions."""
    try:
        subprocess.run(["tmux", "kill-session", "-t", UAV_SESSION], stderr=subprocess.DEVNULL)
        subprocess.run(["tmux", "kill-session", "-t", UGV_SESSION], stderr=subprocess.DEVNULL)
        subprocess.run(["tmux", "kill-session", "-t", "planners"], stderr=subprocess.DEVNULL)
        
        # For hardware, uncomment below:
        # run_remote(UGV_HOST, UGV_USER, UGV_PASS, f"tmux kill-session -t {UGV_SESSION}")
        # run_remote(UAV_HOST, UAV_USER, UAV_PASS, f"tmux kill-session -t {UAV_SESSION}")
        
        print("Cleaned up all tmux sessions")
        
    except Exception as e:
        print(f"Cleanup error: {e}")





def update_scene_yaml(visit_status, file_path="Inputs/scene.yaml"):
    
    with open(file_path, "r") as f:
        scene = yaml.safe_load(f)
    nodes = scene["scenario"]["nodes"]
    for k, v in visit_status.items():
        for node in nodes:
            if node["ID"] == k:
                node["time_last_service"] = v
    with open(file_path, "w") as f:
        yaml.safe_dump(scene, f, sort_keys=False)

    print(f"Updated {file_path}")








# === Main ===
def main():


    # Load existing AoI status
    aoi_status = load_aoi_status()
    visit_status = {}

    # Create UDP sockets
    sock_uav = create_socket(UAV_PORT)
    sock_ugv = create_socket(UGV_PORT)
    print(f"Central listening on ports -> UAV:{UAV_PORT}, UGV:{UGV_PORT}")

    # 0. solve the scene

    sortie_start_time = get_sim_time_from_file()


    # 1. Start UAV-UGV tasks
    start_local_tasks()

    time.sleep(2)
    
    
    # 2. Send initial plan and start replanner
    start_planners()

    

    # 3. Listen for status updates
    uav_task_status, ugv_task_status = {}, {}
    last_msgs = {"UAV": None, "UGV": None}

    try:
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
                            try:
                                if aoi_status[k] != 'COMPLETED':
                                   aoi_status[k] = "COMPLETED"
                                   visit_status[k] = round((get_sim_time_from_file() - sortie_start_time),3)
                            except:
                               pass



                # 4. Check if both tasks completed successfully
                if all_success(uav_task_status) and all_success(ugv_task_status):
                    print("\n\n---------------- Sortie completed! -----------------\n\n")
                    print(' visit status --->', visit_status)
                    # Save AoI status and cleanup
                    save_aoi_status(aoi_status)
                    cleanup()



                    # 5. Update the scene.yaml
                    update_scene_yaml(visit_status)

                    sys.exit(0)

                    
                    
    except KeyboardInterrupt:
        print("\nCentral server interrupted by user. Cleaning up...")
        cleanup()
        sys.exit(1)

if __name__ == "__main__":
    main()