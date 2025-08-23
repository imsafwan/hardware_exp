import asyncio
import yaml
import logging
import socket
from mavsdk import System
from uav_actions import takeoff, goto_gps_target_modi_2, land, vision_landing
import time
import os
import json
import threading


# ==== Utility ====
async def anext(aiter):
    return await aiter.__anext__()

# ==== Logging Setup ====
MISSION_LOG_DIR = "log"
os.makedirs(MISSION_LOG_DIR, exist_ok=True)
timestamp = time.strftime('%Y%m%d-%H%M%S')
log_path = f"{MISSION_LOG_DIR}/mission_{timestamp}.log"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.FileHandler(log_path), logging.StreamHandler()]
)
log = logging.getLogger(__name__)

# ==== UDP Broadcast ====
def broadcast_status(full_status):
    BROADCAST_IP = "255.255.255.255"   # adjust if needed to subnet broadcast
    UGV_LISTEN_PORT = 5005
    C_LISTEN_PORT = 5055
    try:
        msg = json.dumps(full_status)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.sendto(msg.encode(), (BROADCAST_IP, UGV_LISTEN_PORT))
        sock.sendto(msg.encode(), (BROADCAST_IP, C_LISTEN_PORT))
        sock.close()
    except Exception as e:
        log.error(f"❌ Failed to broadcast status: {e}")

# ==== Heartbeat Thread ====
def heartbeat_broadcast(uav_task_status, stop_event):
    """Continuously broadcast UAV task status until stop_event is set."""
    while not stop_event.is_set():
        try:
            broadcast_status({"uav_task_status": uav_task_status})
        except Exception as e:
            log.error(f"❌ Heartbeat broadcast error: {e}")
        time.sleep(1.0)   # 1 Hz heartbeat


# ==== Listen for UGV ====
UGV_PORT = 5006
sock_ugv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_ugv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_ugv.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock_ugv.bind(("", UGV_PORT))
ugv_status = {}


# ------ Main Mission Runner -------
async def run():
    uav_task_status = {}

    # Connect to drone
    drone = System()
    await drone.connect(system_address="udp://:14540")
    log.info("Connecting to drone...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            log.info("✅ Drone connected.")
            break

    # Get reference GPS position
    ''' ref_pos = await anext(drone.telemetry.position())
    ref_lat = ref_pos.latitude_deg
    ref_lon = ref_pos.longitude_deg
    ref_alt = ref_pos.absolute_altitude_m

    pv = await anext(drone.telemetry.position_velocity_ned())
    offset_n = pv.position.north_m
    offset_e = pv.position.east_m'''
    
    #ref_pos = await anext(drone.telemetry.position())
    ref_lat = 0 #ref_pos.latitude_deg
    ref_lon = 0 #ref_pos.longitude_deg
    ref_alt = 0 #ref_pos.absolute_altitude_m

    #pv = await anext(drone.telemetry.position_velocity_ned())
    offset_n = 0 # pv.position.north_m
    offset_e = 0 #pv.position.east_m

    # Load YAML mission
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        actions_yaml = os.path.join(script_dir, '../config/uav_actions.yaml')

        with open(actions_yaml, 'r') as file:
            plan = yaml.safe_load(file)
        log.info(" Mission plan loaded.")
    except Exception as e:
        log.error(f" Failed to load mission plan: {e}")
        return

    # Initialize mission status with PENDING for all actions
    for i, action in enumerate(plan["actions"]):
        action_id = action.get('ins_id', f"uav_action_{i+1}")
        uav_task_status[action_id] = {"type": action["type"], "status": "PENDING"}

    # ---- Start heartbeat thread ----
    stop_event = threading.Event()
    hb_thread = threading.Thread(target=heartbeat_broadcast, args=(uav_task_status, stop_event))
    hb_thread.daemon = True
    hb_thread.start()

    # Execute actions
    try:
        for i, action in enumerate(plan["actions"]):
            action_id = action.get('ins_id', f"uav_action_{i+1}")
            action_type = action["type"]
            log.info(f" Executing action {action_id}: {action_type}")
            st_time = time.time()

            success = False
            try:
                if action_type == "takeoff_from_UGV":
                    print('takeoff action')
                    time.sleep(5)
                    success = True
                    # success = await takeoff(drone, altitude=7.0)

                elif action_type == "move_to_location":
                   
                    
                    time.sleep(5)
                    success = True
                    ''' success = await goto_gps_target_modi_2(
                        drone,
                        action['location']["lat"], action['location']["lon"], 7,
                        ref_lat, ref_lon, ref_alt,
                        speed_mps=1.0,
                        arrival_thresh=0.25,
                        offset_n=offset_n,
                        offset_e=offset_e
                    )'''

                elif action_type == "land_on_UGV":
                    s_time = time.time()
                    time.sleep(5)
                    success = True
                    '''while True:
                        if time.time() - s_time > 240:   # timeout
                            break
                        data, _ = sock_ugv.recvfrom(1024)
                        try:
                            msg = json.loads(data.decode())
                            ugv_status.update(msg)
                            status = ugv_status["ugv_task_status"].get(action_id, {}).get("status")
                            if status == "SUCCESS":
                                success = await land(drone)  # or vision_landing(drone)
                                break
                        except Exception:
                            continue'''

                else:
                    log.warning(f" Unknown action type: {action_type}")

            except Exception as e:
                log.error(f" Failed to execute action '{action_type}': {e}")
                success = False

            # Update mission status
            uav_task_status[action_id]["status"] = "SUCCESS" if success else "FAILED"

            # Log time taken
            action_time = time.time() - st_time
            log.info(f"Time taken to complete {action_id}: {action_time:.2f} sec")

            if not success:
                log.error(f" Aborting mission due to failed action: {action_type}")
                break

    finally:
        # ---- Stop heartbeat when finished ----
        time.sleep(5)
        stop_event.set()
        hb_thread.join()

    log.info("✅ Mission plan execution completed.")


if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        log.warning(" Interrupted by user.")
    except Exception as e:
        log.exception(f" Unhandled error: {e}")

