#!/usr/bin/env python3
import json
import socket
import time
import threading
import networkx as nx
import math
from SPRP_uav import uav_is_replanning
from SPRP_ugv import ugv_is_replanning
import datetime
import os
import logging
from scenario_helper import ScenarioParameters
import yaml

class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True
    

# === Logging Setup ===

# ensure log directory exists
LOG_DIR = "logs"
os.makedirs(LOG_DIR, exist_ok=True)

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
LOG_FILE = os.path.join(LOG_DIR, f"replanner_{timestamp}.log")

logging.basicConfig(
    filename=LOG_FILE,   # now stored inside logs/
    level=logging.INFO,  # DEBUG, INFO, WARNING, ERROR, CRITICAL
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s"
)

logger = logging.getLogger("main")



def logger_print(msg, level="info"):
    print(msg)
    if level == "info":
        logger.info(msg)
    elif level == "warn":
        logger.warn(msg)
    elif level == "error":
        logger.error(msg)
    elif level == "debug":
        logger.debug(msg)






def haversine_distance(coord1, coord2):
    """Calculate distance between two GPS coordinates in meters."""
    R = 6371000  # Earth radius in meters
    lat1, lon1 = math.radians(coord1[0]), math.radians(coord1[1])
    lat2, lon2 = math.radians(coord2[0]), math.radians(coord2[1])
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = (math.sin(dlat/2)**2 + 
         math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c



#----------------------------------------------#

scene = ScenarioParameters('Inputs/scene.yaml')
scene_road_network = scene.road_network_G 

# --- Initialize new graph with IDs instead of coordinates ---
road_network = nx.Graph()

# Add nodes with ID and coordinates
for coord in scene_road_network.nodes:
    node_id = scene.corrd_to_id_map[coord]
    road_network.add_node(node_id, lat=coord[0], lon=coord[1]) 

# Add edges with weights
for u, v, data in scene_road_network.edges(data=True):
    road_network.add_edge(
        scene.corrd_to_id_map[u],
        scene.corrd_to_id_map[v],
        weight=data['weight']
    )






# ---------------- Config ---------------- #
MANAGER_IP = "192.168.0.161"

# Listening ports
UAV_REPLAN_PORT = 5040
UGV_REPLAN_PORT = 5010
UAV_STATUS_PORT = 5041
UGV_STATUS_PORT = 5011

# Sending ports
UAV_PLAN_PORT = 5007
UGV_PLAN_PORT = 5008

# ---------------- Global State ---------------- #
global_state = {"UAV": {}, "UGV": {}}
plan_id = 1

# ---------------- Setup Sockets ---------------- #
sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def setup_socket(port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", port))
    sock.setblocking(False)
    return sock

def send_with_ack(msg, target_ip, target_port, agent_name, max_retries=100, timeout=1.0):
    """Send msg and wait for ACK from agent"""
    sock_ack = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_ack.bind(("0.0.0.0", 6000))   
    sock_ack.settimeout(timeout)

    

    for attempt in range(max_retries):
        # Send message
        sock_tx.sendto(msg, (target_ip, target_port))
        logger_print(f"→ Sent {msg[:50]}... to {agent_name} (attempt {attempt+1})")

        try:
            data, _ = sock_ack.recvfrom(4096)
            ack = json.loads(data.decode())
            if ack.get("msg_type") == "ACK" and ack.get("plan_id") == json.loads(msg.decode())["plan_id"]:
                logger_print(f"✅ ACK received from {agent_name}")
                sock_ack.close()
                return True
        except socket.timeout:
            logger_print(f"⚠️ No ACK from {agent_name}, retrying...")

    logger_print(f"❌ Failed to deliver plan to {agent_name} after {max_retries} retries")
    sock_ack.close()
    return False




def send_plans(agent_name):
    """Send new plans to both agents"""

    logger_print(" > Generating new plans... \n")
    global plan_id
    plan_id += 1

    logger_print(f'\n Global state : {global_state}\n')

    if agent_name == "UAV":
                    uav_state = global_state['UAV']['uav_state']
                    ugv_state = global_state['UGV']['ugv_state']
                    uav_remaining_actions = global_state['UAV'].get('uav_remaining_actions', [])
                    ugv_remaining_actions = global_state['UGV'].get('ugv_remaining_actions', [])
                    uav_plan, ugv_plan = uav_is_replanning(uav_state, ugv_state, uav_remaining_actions, ugv_remaining_actions, road_network)

    else:
                    uav_state = global_state['UAV']['uav_state']
                    ugv_state = global_state['UGV']['ugv_state']
                    uav_remaining_actions = global_state['UAV'].get('uav_remaining_actions', [])
                    ugv_remaining_actions = global_state['UGV'].get('ugv_remaining_actions', [])
                    uav_plan, ugv_plan = ugv_is_replanning(uav_state, ugv_state, uav_remaining_actions, ugv_remaining_actions, road_network)
        
    
    # Send to UAV
    msg_uav = json.dumps({"msg_type": "UAV_PLAN", "plan_id": plan_id, "new_plan": uav_plan}).encode()
    send_with_ack(msg_uav, MANAGER_IP, UAV_PLAN_PORT, "UAV")
    logger_print(f"- Sent plan {plan_id} to UAV")
    
    # Send to UGV  
    msg_ugv = json.dumps({"msg_type": "UGV_PLAN", "plan_id": plan_id, "new_plan": ugv_plan}).encode()
    send_with_ack(msg_ugv, MANAGER_IP, UGV_PLAN_PORT, "UGV")
    logger_print(f"- Sent plan {plan_id} to UGV")


    # Ensure folder exists
    os.makedirs("output", exist_ok=True)

    # Dump UAV actions
    with open(os.path.join("output", "uav_actions.yaml"), "w") as f:
        yaml.dump({'sortie 1':uav_plan}, f, Dumper=NoAliasDumper, sort_keys=False)

    # Dump UGV actions
    with open(os.path.join("output", "ugv_actions.yaml"), "w") as f:
        yaml.dump({'sortie 1':ugv_plan}, f, Dumper=NoAliasDumper, sort_keys=False)

    print("Saved uav_actions.yaml and ugv_actions.yaml in output/")

# ---------------- Listeners ---------------- #
def listen_replan(sock, agent_name):
    """Listen for replan requests"""
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            msg = json.loads(data.decode())
            if msg.get("msg_type") == "REPLAN_REQUEST":
                logger_print('\n ---------------- replanning request ----------- \n')
                logger_print(f"- {agent_name} requested replanning: {msg.get('reason')}")
                send_plans(agent_name)
                
        
        except:
            time.sleep(0.1)

def listen_status(sock, agent_name):
    """Listen for status updates and update global state"""
    while True:
        #logger_print('global_state:', global_state)
        try:
            data, _ = sock.recvfrom(4096)
            msg = json.loads(data.decode())
            if msg.get("msg_type") == "STATUS":
                # Simple global state update
                global_state[agent_name] = msg
                
                # logger_print status
                if agent_name == "UAV" and "uav_state" in msg:
                    loc = msg["uav_state"].get("loc", [0, 0])
                    remaining = len(msg.get("uav_remaining_actions", []))
                    #logger_print(f"📥 UAV: ({loc[0]:.6f},{loc[1]:.6f}), {remaining} left")
                elif agent_name == "UGV" and "ugv_state" in msg:
                    loc = msg["ugv_state"].get("loc", [0, 0]) 
                    remaining = len(msg.get("ugv_remaining_actions", []))
                    #logger_print(f"📥 UGV: ({loc[0]:.6f},{loc[1]:.6f}), {remaining} left")
        except:
            time.sleep(0.1)

# ---------------- Main ---------------- #
def main():
    logger_print("🚀 Mission Manager starting...")
    
    # Setup sockets
    uav_replan_sock = setup_socket(UAV_REPLAN_PORT)
    ugv_replan_sock = setup_socket(UGV_REPLAN_PORT)
    uav_status_sock = setup_socket(UAV_STATUS_PORT)
    ugv_status_sock = setup_socket(UGV_STATUS_PORT)
    
    # Start listeners
    threads = [
        threading.Thread(target=listen_replan, args=(uav_replan_sock, "UAV"), daemon=True),
        threading.Thread(target=listen_replan, args=(ugv_replan_sock, "UGV"), daemon=True),
        threading.Thread(target=listen_status, args=(uav_status_sock, "UAV"), daemon=True),
        threading.Thread(target=listen_status, args=(ugv_status_sock, "UGV"), daemon=True)
    ]
    
    for t in threads:
        t.start()
    
    logger_print("✅ Manager running. Press Ctrl+C to stop.")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger_print("\n🛑 Manager stopped")

if __name__ == "__main__":
    main()