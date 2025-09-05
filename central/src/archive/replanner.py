#!/usr/bin/env python3
import json
import socket
import time
import threading
from collections import defaultdict

# ---------------- Config ---------------- #
MANAGER_IP = "192.168.0.161"  # This machine's IP

# Ports where manager listens
UAV_REPLAN_PORT = 5040    # UAV sends REPLAN_REQUEST here
UGV_REPLAN_PORT = 5010    # UGV sends REPLAN_REQUEST here
UAV_STATUS_PORT = 5041    # UAV sends STATUS here
UGV_STATUS_PORT = 5011    # UGV sends STATUS here

# Ports where UAV/UGV listen for plan updates
UAV_PLAN_PORT = 5007
UGV_PLAN_PORT = 5008

# ---------------- Global State ---------------- #
global_state = {
    "UAV": {"last_update": 0, "data": {}},
    "UGV": {"last_update": 0, "data": {}}
}
plan_id = 0
state_lock = threading.Lock()

# ---------------- Setup Sockets ---------------- #
def setup_sockets():
    """Setup all UDP sockets"""
    sockets = {}
    
    # Listening sockets
    sockets['uav_replan'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sockets['ugv_replan'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sockets['uav_status'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sockets['ugv_status'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Bind listening sockets
    sockets['uav_replan'].bind(("", UAV_REPLAN_PORT))
    sockets['ugv_replan'].bind(("", UGV_REPLAN_PORT))
    sockets['uav_status'].bind(("", UAV_STATUS_PORT))
    sockets['ugv_status'].bind(("", UGV_STATUS_PORT))
    
    # Set non-blocking
    for sock in sockets.values():
        sock.setblocking(False)
    
    # Sending socket
    sockets['tx'] = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    return sockets

# ---------------- Replanning Logic ---------------- #
def generate_new_plans(reason, requesting_agent):
    """Generate new plans based on current state and reason"""
    print(f"🧭 Generating new plans due to: {reason} from {requesting_agent}")
    
    with state_lock:
        current_state = global_state.copy()
    
    # Simple replanning logic - you can make this more sophisticated
    uav_plan = [
        {
            "type": "takeoff_from_UGV",
            "ins_id": f"takeoff_{int(time.time())}",
        },
        {
            "type": "move_to_location",
            "ins_id": f"uav_move_{int(time.time())}",
            "location": {"lat": 41.8701, "lon": -87.6503},
            "end_time": 120  # 2 minutes from start
        },
        {
            "type": "land_on_UGV",
            "ins_id": f"land_{int(time.time())}",
        }
    ]
    
    ugv_plan = [
        {
            "type": "move_to_location",
            "ins_id": f"ugv_move_{int(time.time())}",
            "location": {"lat": 41.8700, "lon": -87.65025},
            "end_time": 100  # Move to position before UAV lands
        },
        {
            "type": "allow_take_off_from_UGV",
            "ins_id": f"allow_takeoff_{int(time.time())}",
        },
        {
            "type": "allow_land_on_UGV",
            "ins_id": f"allow_land_{int(time.time())}",
        }
    ]
    
    return uav_plan, ugv_plan

def send_plan_update(agent, plan_id, plan, sock_tx):
    """Send plan update to UAV or UGV"""
    msg = {
        "msg_type": "PLAN_UPDATE",
        "plan_id": plan_id,
        "new_plan": plan,
        "time": time.time()
    }
    
    port = UAV_PLAN_PORT if agent == "UAV" else UGV_PLAN_PORT
    
    try:
        data = json.dumps(msg).encode()
        sock_tx.sendto(data, (MANAGER_IP, port))
        print(f"📤 Sent plan {plan_id} to {agent} ({len(plan)} actions)")
    except Exception as e:
        print(f"❌ Failed to send plan to {agent}: {e}")

# ---------------- Message Listeners ---------------- #
def listen_for_replan_requests(sock, agent_name, sock_tx):
    """Listen for replan requests from UAV or UGV"""
    global plan_id
    
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            msg = json.loads(data.decode())
            
            if msg.get("msg_type") == "REPLAN_REQUEST":
                reason = msg.get("reason", "Unknown reason")
                print(f"🚨 {agent_name} requested replanning: {reason}")
                
                # Generate new plans
                plan_id += 1
                uav_plan, ugv_plan = generate_new_plans(reason, agent_name)
                
                # Send plans to both agents
                send_plan_update("UAV", plan_id, uav_plan, sock_tx)
                send_plan_update("UGV", plan_id, ugv_plan, sock_tx)
                
        except BlockingIOError:
            # No data available - normal for non-blocking socket
            time.sleep(0.1)
        except Exception as e:
            print(f"❌ Error in {agent_name} replan listener: {e}")
            time.sleep(0.1)

def listen_for_status_updates(sock, agent_name):
    """Listen for status updates from UAV or UGV"""
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            msg = json.loads(data.decode())
            
            if msg.get("msg_type") == "STATUS":
                with state_lock:
                    global_state[agent_name]["data"] = msg
                    global_state[agent_name]["last_update"] = time.time()
                
                # Print condensed status
                if agent_name == "UAV" and "uav_state" in msg:
                    loc = msg["uav_state"].get("loc", [0, 0])
                    #print('UAV msg:', msg)
                    remaining = len(msg.get("uav_remaining_actions", []))
                    print(f" UAV: pos=({loc[0]:.4f},{loc[1]:.4f}), {remaining} actions left")
                    
                elif agent_name == "UGV" and "ugv_state" in msg:
                    loc = msg["ugv_state"].get("loc", [0, 0])
                    remaining = len(msg.get("ugv_remaining_actions", []))
                    #print('UGV msg:', msg)  # Debug print
                    print(f" UGV: pos=({loc[0]:.4f},{loc[1]:.4f}), {remaining} actions left")
                
        except BlockingIOError:
            time.sleep(0.1)
        except Exception as e:
            print(f"❌ Error in {agent_name} status listener: {e}")
            time.sleep(0.1)

def print_status_summary():
    """Print periodic status summary"""
    while True:
        try:
            with state_lock:
                current_time = time.time()
                uav_age = current_time - global_state["UAV"]["last_update"]
                ugv_age = current_time - global_state["UGV"]["last_update"]

            print('global_state:', global_state)  # Debug print
            
            print(f"\n📊 Status Summary:")
            print(f"   UAV: {'Connected' if uav_age < 5 else 'Lost connection'} (last: {uav_age:.1f}s ago)")
            print(f"   UGV: {'Connected' if ugv_age < 5 else 'Lost connection'} (last: {ugv_age:.1f}s ago)")
            print(f"   Current Plan ID: {plan_id}")
            
            time.sleep(0.05)  # Print every 10 seconds
            
        except Exception as e:
            print(f"❌ Status summary error: {e}")
            time.sleep(10)

# ---------------- Main ---------------- #
def main():
    print("🚀 Starting Mission Manager...")
    print(f"   Manager IP: {MANAGER_IP}")
    print(f"   Listening for UAV replan requests on port {UAV_REPLAN_PORT}")
    print(f"   Listening for UGV replan requests on port {UGV_REPLAN_PORT}")
    print(f"   Listening for UAV status on port {UAV_STATUS_PORT}")
    print(f"   Listening for UGV status on port {UGV_STATUS_PORT}")
    
    # Setup sockets
    sockets = setup_sockets()
    
    # Start all listener threads
    threads = [
        # Replan request listeners
        threading.Thread(
            target=listen_for_replan_requests,
            args=(sockets['uav_replan'], "UAV", sockets['tx']),
            daemon=True
        ),
        threading.Thread(
            target=listen_for_replan_requests,
            args=(sockets['ugv_replan'], "UGV", sockets['tx']),
            daemon=True
        ),
        
        # Status listeners
        threading.Thread(
            target=listen_for_status_updates,
            args=(sockets['uav_status'], "UAV"),
            daemon=True
        ),
        threading.Thread(
            target=listen_for_status_updates,
            args=(sockets['ugv_status'], "UGV"),
            daemon=True
        ),
        
        # Status summary
        threading.Thread(target=print_status_summary, daemon=True)
    ]
    
    # Start all threads
    for thread in threads:
        thread.start()
    
    print("Mission Manager running. Press Ctrl+C to stop.")
    
    try:
        # Keep main thread alive
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nMission Manager stopped by user")
    finally:
        # Clean up sockets
        for sock in sockets.values():
            try:
                sock.close()
            except:
                pass

if __name__ == "__main__":
    main()