import asyncio
import yaml
import logging
import time
import os
import json
import tempfile
import socket
import shutil





# === Logging Setup ===
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

TMP_DIR = "tmp"
if os.path.exists(TMP_DIR):
    shutil.rmtree(TMP_DIR)   # delete entire folder + old files
os.makedirs(TMP_DIR, exist_ok=True)

STATUS_FILE = os.path.join(TMP_DIR, "uav_status.json")


def update_status(uav_task_status):
    try:
        with tempfile.NamedTemporaryFile("w", delete=False, dir="tmp") as f:
            json.dump({"uav_task_status": uav_task_status}, f)
            temp_name = f.name
        os.replace(temp_name, STATUS_FILE)  # atomic replace
    except Exception as e:
        log.error(f"❌ Failed to write status: {e}")


# ==== Listen for UGV ====
UGV_PORT = 5006
sock_ugv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_ugv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_ugv.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock_ugv.bind(("", UGV_PORT))
ugv_status = {}


async def run():
    uav_task_status = {}

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

    # Init all actions as PENDING
    for i, action in enumerate(plan["actions"]):
        action_id = action.get('ins_id', f"uav_action_{i+1}")
        uav_task_status[action_id] = {"type": action["type"], "status": "PENDING"}
    update_status(uav_task_status)

    # Execute mission
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
                    
                    while True:
                        if time.time() - s_time > 240:   # timeout
                            break
                        data, _ = sock_ugv.recvfrom(1024)
                        try:
                            msg = json.loads(data.decode())
                            ugv_status.update(msg)
                            status = ugv_status["ugv_task_status"].get(action_id, {}).get("status")
                            if status == "SUCCESS":
                                time.sleep(5)
                                success = True
                                #success = await land(drone)  # or vision_landing(drone)
                                break
                        except Exception:
                            continue

                else:
                    log.warning(f" Unknown action type: {action_type}")

            except Exception as e:
                log.error(f" Failed to execute action '{action_type}': {e}")
                success = False


            # Update status
            uav_task_status[action_id]["status"] = "SUCCESS" if success else "FAILED"
            update_status(uav_task_status)

            log.info(f"Time taken {action_id}: {time.time()-st_time:.2f}s")

            if not success:
                log.error(f" Aborting mission at {action_type}")
                break

    finally:
        log.info("<------ uav sortie complete ------>")
        update_status(uav_task_status)
        

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        log.warning(" Interrupted by user.")
