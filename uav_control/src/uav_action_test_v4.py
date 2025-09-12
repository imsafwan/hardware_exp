''' It executes UAV actions from an action plan, communicates with a central manager via UDP, and reports status.  '''


from rclpy.clock import Clock
import socket, json, time, os, tempfile, shutil
from collections import deque
import asyncio
from mavsdk import System
from uav_actions import takeoff, goto_gps_target_modi_2, land, vision_landing
import logging
import datetime

# ensure log directory exists
LOG_DIR = "logs"
os.makedirs(LOG_DIR, exist_ok=True)

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
LOG_FILE = os.path.join(LOG_DIR, f"uav_actions_{timestamp}.log")

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

# ==== Listen for UGV ====
UGV_PORT = 5006
sock_ugv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_ugv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_ugv.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock_ugv.bind(("", UGV_PORT))
ugv_status = {}





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



# ---------------- Setup ----------------
UAV_PORT = 5007               # UAV listens here for plan updates from Manager
MANAGER_IP = "192.168.0.161"  # central laptop / mission manager
MANAGER_PORT_RP = 5040      # UAV sends status/replan requests to manager for replanning
MANAGER_PORT_UAS = 5041     # UAV sends status/replan requests to manager for uav status

sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rx.bind(("", UAV_PORT))
sock_rx.setblocking(False)   # non-blocking mode

sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

TMP_DIR = "tmp"
if os.path.exists(TMP_DIR):
    shutil.rmtree(TMP_DIR)
os.makedirs(TMP_DIR, exist_ok=True)
STATUS_FILE = os.path.join(TMP_DIR, "uav_status.json")

# ---------------- Utils ----------------
def update_status_file(uav_task_status):
    with tempfile.NamedTemporaryFile("w", delete=False, dir=TMP_DIR) as f:
        json.dump({"uav_task_status": uav_task_status}, f)
        temp_name = f.name
    os.replace(temp_name, STATUS_FILE)

AOI_STATUS_FILE = os.path.join(TMP_DIR, "aoi_status.json")
aoi_status_dict = {}

def update_AoI_status_file(AoI_id, status="COMPLETED"):
    aoi_status_dict[AoI_id] = status
    with tempfile.NamedTemporaryFile("w", delete=False, dir=TMP_DIR) as f:
        json.dump({"aoi_status": aoi_status_dict}, f)
        temp_name = f.name
    os.replace(temp_name, AOI_STATUS_FILE)





async def status_broadcaster(drone, action_queue, sortie_start_time, period=0.1):
    """Continuously broadcast UAV status every `period` seconds."""
    while True:
        try:
            # Get current position
            pos = await drone.telemetry.position().__anext__()
            gps = (pos.latitude_deg, pos.longitude_deg)

            # Compute elapsed mission time
            time_elapsed = get_sim_time_from_file() - sortie_start_time

            # Remaining actions
            remaining = list(action_queue)  # shallow copy

            msg = {
                "msg_type": "STATUS",
                "sender": "UAV",
                "uav_state": {
                    "loc": gps,
                    "time_elapsed": time_elapsed
                },
                "uav_remaining_actions": remaining,
                "time": get_sim_time_from_file()
            }

            sock_tx.sendto(json.dumps(msg).encode(), (MANAGER_IP, MANAGER_PORT_UAS))

        except Exception as e:
            logger_print(f"Status broadcaster error: {e}")

        await asyncio.sleep(period)  # wait before sending next update


def send_replan_request(reason):
    msg = {
        "msg_type": "REPLAN_REQUEST",
        "sender": "UAV",
        "reason": reason,
        "time": get_sim_time_from_file()
    }
    sock_tx.sendto(json.dumps(msg).encode(), (MANAGER_IP, MANAGER_PORT_RP))
    logger_print(f"<---------------------------  Sent REPLAN_REQUEST ------------------------------>: {reason}")


def check_for_new_plan(current_plan_id, action_queue, uav_task_status):
    """Poll socket up to max_checks times for PLAN_UPDATE.""" 
    max_checks=10
    checks = 0
    while checks < max_checks:
        try:
            data, _ = sock_rx.recvfrom(4096)  # non-blocking
            msg = json.loads(data.decode())
            if msg.get("msg_type") == "UAV_PLAN":
                plan_id = msg["plan_id"]
                if plan_id > current_plan_id:
                    logger_print(f" New plan {plan_id} received")
                    current_plan_id = plan_id

                    # Replace the queue with the new plan
                    action_queue.clear()
                    action_queue.extend(msg["new_plan"])

                    # Reset task status
                    uav_task_status.clear()
                    for i, a in enumerate(msg["new_plan"]):
                        aid = a.get("ins_id", f"uav_{i}")
                        uav_task_status[aid] = {"type": a["type"], "status": "PENDING"}
                    update_status_file(uav_task_status)

            checks += 1
        except BlockingIOError:
            break  # no more data available

    return current_plan_id



# ---------------- Action Executor ----------------
async def execute_action(drone, action, ref_lat, ref_lon, ref_alt, offset_n, offset_e):
    action_type = action["type"]
    try:
        if action_type == "takeoff_from_UGV":
            return await takeoff(drone, altitude=7.0)

        elif action_type == "move_to_location":
            return await goto_gps_target_modi_2(
                drone,
                action["location"]["lat"], action["location"]["lon"], 7,
                ref_lat, ref_lon, ref_alt,
                speed_mps=1.0, arrival_thresh=0.25,
                offset_n=offset_n, offset_e=offset_e
            )

        elif action_type == "land_on_UGV":

            s_time = time.time()
            success = False      
            while True:
                if time.time() - s_time > 240:   # timeout
                    break
                data, _ = sock_ugv.recvfrom(1024)
                try:
                    msg = json.loads(data.decode())
                    ugv_status.update(msg)
                    action_id = action['ins_id']
                    status = ugv_status["ugv_task_status"].get(action_id, {}).get("status")
                    if status == "SUCCESS":
                        # time.sleep(5)
                        # success = True
                        success = await vision_landing(drone) #land(drone)  # or vision_landing(drone)


                        


                        break
                except Exception:
                    continue
            return success

        else:
            logger_print(f"Unknown action: {action_type}")
            return False
    except Exception as e:
        logger_print(f"Action {action_type} failed: {e}")
        return False


# ---------------- Main ----------------
async def run():

    # Connect to drone
    drone = System()
    await drone.connect(system_address="udp://:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            logger_print("UAV connected to PX4")
            break

    ref_pos = await drone.telemetry.position().__anext__()
    ref_lat, ref_lon, ref_alt = ref_pos.latitude_deg, ref_pos.longitude_deg, ref_pos.absolute_altitude_m
    pv = await drone.telemetry.position_velocity_ned().__anext__()
    offset_n, offset_e = pv.position.north_m, pv.position.east_m

    action_queue = deque()
    uav_task_status = {}
    current_plan_id = 0
    uav_takeoff_time = get_sim_time_from_file()

    # Start background status broadcaster
    asyncio.create_task(status_broadcaster(drone, action_queue, uav_takeoff_time))

    # === Execution Loop ===
    while True:

        # Always check for latest plan (even if queue is not empty)
        current_plan_id = check_for_new_plan(current_plan_id, action_queue, uav_task_status)

        logger_print('action list : {}'.format(action_queue))

        if action_queue:
            action = action_queue.popleft()
            action_id = action.get("ins_id", f"uav_{int(get_sim_time_from_file())}")
            logger_print(f"Executing {action_id}: {action['type']}")

            start_time = get_sim_time_from_file()
            success = await execute_action(drone, action, ref_lat, ref_lon, ref_alt, offset_n, offset_e)
            exe_time = get_sim_time_from_file() - start_time
            logger_print(' Time takes to execute action {}: {:.1f}s'.format(action, exe_time))

            # Update status + report to manager
            uav_task_status[action_id] = {"type": action["type"], "status": "SUCCESS" if success else "FAILED"}
            update_status_file(uav_task_status)


            # upate PoI status for move_to_location actions
            if action["type"] == "move_to_location" and success:
                
                update_AoI_status_file(action['aoi_id'])
            

            # Local trigger: exceeded expected end_time
            if len(list(action_queue)) >=2 : # if len = 1, that means remaining actions = move+land pair and if len = 0 , then only land.
                if "end_time" in action:
                    uav_time_elapsed = get_sim_time_from_file() - uav_takeoff_time
                    logger_print(' uav_time_elapsed: {:.1f}s, action end_time: {:.1f}s'.format(uav_time_elapsed, action["end_time"]))
                    if uav_time_elapsed > action["end_time"]:
                        send_replan_request("Exceeded expected action time")
                        await asyncio.sleep(1.0)


            if not success:
                logger_print(f"Aborting mission at {action_id}")
                action_queue.clear()

        else:
            # Nothing to do, keep polling for new plan
            await asyncio.sleep(0.5)



if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        logger_print("UAV executor stopped by user")
