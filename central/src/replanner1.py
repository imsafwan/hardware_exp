#!/usr/bin/env python3
import json
import socket
import time
import threading
import networkx as nx
import math
from SPRP_uav import uav_is_replanning
from SPRP_ugv import ugv_is_replanning

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

# --- Road network (simple 3-node graph) ---
road_network = nx.Graph()

# Road network nodes (UAV MUST land at these points)
road_network.add_node("A", lat=41.870046, lon=-87.650351)  # Start position
road_network.add_node("B", lat=41.870007, lon=-87.650315)  # Road intersection 1
road_network.add_node("C", lat=41.869950, lon=-87.650280)  # Road intersection 2
road_network.add_node("D", lat=41.869890, lon=-87.650240)  # Road intersection 3
road_network.add_node("E", lat=41.869960, lon=-87.650180)  # Road intersection 4
road_network.add_node("F", lat=41.870020, lon=-87.650120)  # Road intersection 5

# Calculate distances and add edges
edges = [
    ("A", "B"),
    ("B", "C"), 
    ("C", "D"),
    ("D", "E"),
    ("E", "F"),
    ("B", "E"),  # Shortcut road
    ("A", "C"),  # Alternative route
    ("C", "F")   # Another route option
]

for start, end in edges:
    start_coord = (road_network.nodes[start]["lat"], road_network.nodes[start]["lon"])
    end_coord = (road_network.nodes[end]["lat"], road_network.nodes[end]["lon"])
    distance = haversine_distance(start_coord, end_coord)
    road_network.add_edge(start, end, weight=distance)



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



def send_plans(agent_name):
    """Send new plans to both agents"""

    print(" > Generating new plans... \n")
    global plan_id
    plan_id += 1

    print('\n Global state : ', global_state, '\n')

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
    msg = {"msg_type": "UAV_PLAN", "plan_id": plan_id, "new_plan": uav_plan}
    sock_tx.sendto(json.dumps(msg).encode(), (MANAGER_IP, UAV_PLAN_PORT))
    print(f"- Sent plan {plan_id} to UAV")
    
    # Send to UGV  
    msg = {"msg_type": "UGV_PLAN", "plan_id": plan_id, "new_plan": ugv_plan}
    sock_tx.sendto(json.dumps(msg).encode(), (MANAGER_IP, UGV_PLAN_PORT))
    print(f"- Sent plan {plan_id} to UGV")

# ---------------- Listeners ---------------- #
def listen_replan(sock, agent_name):
    """Listen for replan requests"""
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            msg = json.loads(data.decode())
            if msg.get("msg_type") == "REPLAN_REQUEST":
                print('\n ---------------- replanning request ----------- \n')
                print(f"- {agent_name} requested replanning: {msg.get('reason')}")
                send_plans(agent_name)
                
        
        except:
            time.sleep(0.1)

def listen_status(sock, agent_name):
    """Listen for status updates and update global state"""
    while True:
        #print('global_state:', global_state)
        try:
            data, _ = sock.recvfrom(4096)
            msg = json.loads(data.decode())
            if msg.get("msg_type") == "STATUS":
                # Simple global state update
                global_state[agent_name] = msg
                
                # Print status
                if agent_name == "UAV" and "uav_state" in msg:
                    loc = msg["uav_state"].get("loc", [0, 0])
                    remaining = len(msg.get("uav_remaining_actions", []))
                    #print(f"📥 UAV: ({loc[0]:.6f},{loc[1]:.6f}), {remaining} left")
                elif agent_name == "UGV" and "ugv_state" in msg:
                    loc = msg["ugv_state"].get("loc", [0, 0]) 
                    remaining = len(msg.get("ugv_remaining_actions", []))
                    #print(f"📥 UGV: ({loc[0]:.6f},{loc[1]:.6f}), {remaining} left")
        except:
            time.sleep(0.1)

# ---------------- Main ---------------- #
def main():
    print("🚀 Mission Manager starting...")
    
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
    
    print("✅ Manager running. Press Ctrl+C to stop.")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n🛑 Manager stopped")

if __name__ == "__main__":
    main()