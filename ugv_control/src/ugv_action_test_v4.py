#!/usr/bin/env python3
''' It executes UGV actions from an action plan, communicates with UAV via UDP, and reports status to a central manager.  '''





from rclpy.clock import Clock
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from sensor_msgs.msg import NavSatFix
import utm
import yaml
import os
import json
import time
import socket
import shutil
import threading
import tempfile
from collections import deque
import logging
import os
import datetime


# ensure log directory exists
LOG_DIR = "logs"
os.makedirs(LOG_DIR, exist_ok=True)
timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
LOG_FILE = os.path.join(LOG_DIR, f"ugv_actions_{timestamp}.log")

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


logger_print("System started")






# def get_sim_time_from_file(filename="/home/safwan/hardware_exp/central/src/clock_log.txt"):
#     # keep a static variable to store last valid time
#     if not hasattr(get_sim_time_from_file, "last_time"):
#         get_sim_time_from_file.last_time = None

#     try:
#         with open(filename, "r") as f:
#             line = f.read().strip()
#             if line:
#                 get_sim_time_from_file.last_time = float(line)
#                 return get_sim_time_from_file.last_time
#     except Exception as e:
#         # could add logging here if you want
#         pass

#     # fallback to last valid time if reading failed
#     return get_sim_time_from_file.last_time

def get_sim_time_from_file(filename="/home/safwan/hardware_exp/central/src/clock_log.txt", retry_delay=0.05, timeout=5.0):
    start = time.time()
    while True:
        try:
            with open(filename, "r") as f:
                line = f.read().strip()
                if line:
                    return float(line)
        except Exception:
            pass

        if time.time() - start > timeout:
            raise RuntimeError("Timeout: could not read sim time from file.")
        time.sleep(retry_delay)

UAV_PORT = 5005
sock_uav = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_uav.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_uav.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock_uav.bind(("", UAV_PORT))
uav_status = {}

class UGVActionRunner(Node):
    def __init__(self, actions_yaml, map_origin):
        super().__init__('ugv_action_runner')

        self.ugv_start_time = get_sim_time_from_file()
        
        # Basic setup
        self.actions_yaml = os.path.expanduser(actions_yaml)
        self.map_origin = map_origin
        
        # UTM conversion setup
        utm_x, utm_y, zone_number, _ = utm.from_latlon(map_origin[0], map_origin[1])
        self.utm_origin_easting = utm_x
        self.utm_origin_northing = utm_y
        self.utm_zone = zone_number
        
        # Live location tracking
        self.current_position = {"lat": map_origin[0], "lon": map_origin[1]}  # Default to map origin
        self.position_lock = threading.Lock()
        
        # ROS2 setup
        namespace = "a200_0000"
        self.nav_client = ActionClient(self, NavigateToPose, f'{namespace}/navigate_to_pose')
        
        # Subscribe to GPS topic for live location
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            f'{namespace}/sensors/gps_0/fix',
            self.gps_callback,
            10
        )
        
        # Shared state with simple lock
        self.action_queue = deque()
        self.current_action = {"value": None}
        self.ugv_task_status = {}
        self.current_plan_id = 0
        self.lock = threading.Lock()
        
        # Networking
        self.setup_networking()
        
        # File management
        self.tmp_dir = "tmp"
        if os.path.exists(self.tmp_dir):
            shutil.rmtree(self.tmp_dir)
        os.makedirs(self.tmp_dir, exist_ok=True)
        self.status_file = os.path.join(self.tmp_dir, "ugv_status.json")
        
        # Load initial plan
        #self.load_initial_plan()
        
        # Start background threads
        self.start_background_tasks()
        
        
        logger_print("UGV Action Runner initialized")

    def gps_callback(self, msg):
        """Callback for GPS updates from ROS2 topic"""
        try:
            # GPS already provides lat/lon directly
            if msg.status.status >= 0:  # Valid GPS fix
                with self.position_lock:
                    self.current_position = {"lat": msg.latitude, "lon": msg.longitude}
                    
        except Exception as e:
            logger_print(f"GPS callback error: {e}")

    def get_current_location(self):
        """Get current live location thread-safely"""
        with self.position_lock:
            return self.current_position.copy()

    def setup_networking(self):
        """Setup UDP sockets"""
        self.manager_ip = "192.168.0.161"
        self.manager_port_rp = 5010  # replan requests
        self.manager_port_ugs = 5011  # status updates
        self.UGV_PORT = 5008  # UGV listen port
        
        self.sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_rx.bind(("", self.UGV_PORT))  # UGV listen port
        self.sock_rx.setblocking(False)

        


    def load_initial_plan(self):
        """Load actions from YAML file"""
        try:
            with open(self.actions_yaml, 'r') as f:
                plan = yaml.safe_load(f).get('actions', [])
                
            with self.lock:
                self.action_queue.extend(plan)
                for i, action in enumerate(plan):
                    aid = action.get("ins_id", f"ugv_{i}")
                    self.ugv_task_status[aid] = {"type": action["type"], "status": "PENDING"}
            
            self.update_status_file()
            logger_print(f"Loaded {len(plan)} actions")
            
        except Exception as e:
            logger_print(f"Failed to load YAML: {e}")

    def update_status_file(self):
        """Safely update status file"""
        try:
            with self.lock:
                status_data = {"ugv_task_status": self.ugv_task_status.copy()}
            
            # Atomic file write
            with tempfile.NamedTemporaryFile(mode='w', dir=self.tmp_dir, delete=False) as f:
                json.dump(status_data, f)
                temp_name = f.name
            os.replace(temp_name, self.status_file)
            
        except Exception as e:
            logger_print(f"Failed to write status: {e}")

    def send_udp_message(self, msg, port):
        """Send UDP message (non-blocking)"""
        try:
            data = json.dumps(msg).encode()
            self.sock_tx.sendto(data, (self.manager_ip, port))
        except Exception as e:
            logger_print(f"UDP send error: {e}")

    def status_broadcaster(self):
        """Background thread for status broadcasting"""
        while rclpy.ok():
            try:
                with self.lock:
                    # Use live location instead of map origin
                    current_loc = self.get_current_location()
                    
                    remaining = list(self.action_queue)
                    current_action_loc = self.current_action.get("value", {}).get("location", None)
                    curr_rp = (current_action_loc['lat'], current_action_loc['lon']) if current_action_loc is not None else None
                    task_status = self.ugv_task_status.copy()
                    #logger_print('remaining:', remaining)

                    
                    if len(remaining) >= 1:
                        next_rp = (remaining[0]['location']['lat'], remaining[0]['location']['lon'])
                    else:
                        next_rp = None
                    ugv_state = {
                        "loc": [current_loc["lat"], current_loc["lon"]],
                        "next_road_point": next_rp,
                        'curr_road_point': curr_rp,
                        "time_elapsed": get_sim_time_from_file() - self.ugv_start_time + 2.00 # as we add 2 secs of sleep for this replanning
                    }

                msg = {
                    "msg_type": "STATUS",
                    "sender": "UGV",
                    "ugv_state": ugv_state,
                    "ugv_remaining_actions": remaining,
                    "ugv_task_status": task_status,
                    "time": get_sim_time_from_file()
                }
                
                self.send_udp_message(msg, self.manager_port_ugs)
                time.sleep(1.0)  # Send every second
                
            except Exception as e:
                logger_print(f"Status broadcast error: {e}")
                time.sleep(1.0)

    def plan_checker(self):
        """Background thread for checking plan updates"""
        while rclpy.ok():
            try:
                # Check for new plans
                data, sender_addr = self.sock_rx.recvfrom(4096)
                msg = json.loads(data.decode())
                
                if msg.get("msg_type") == "UGV_PLAN":
                    plan_id = msg["plan_id"]

                    with self.lock:
                        if plan_id > self.current_plan_id:
                            logger_print(f"📥 New plan {plan_id} received")
                            self.current_plan_id = plan_id
                            logger_print(f" New plan: {msg['new_plan']} \n")
                            
                            # Update plan
                            self.action_queue.clear()
                            self.action_queue.extend(msg["new_plan"])
                            
                            # Reset status
                            self.ugv_task_status.clear()
                            for i, action in enumerate(msg["new_plan"]):
                                aid = action.get("ins_id", f"ugv_{i}")
                                self.ugv_task_status[aid] = {"type": action["type"], "status": "PENDING"}
                    
                    self.update_status_file()

                    # === Send ACK back ===
                    ack = {
                        "msg_type": "ACK",
                        "plan_id": plan_id,
                    }
                    sender_addr_ip = sender_addr[0]  # IP of whoever sent the plan
                    ack_port = 6000              # fixed ACK listener port on manager
                    self.sock_tx.sendto(json.dumps(ack).encode(), (sender_addr_ip, ack_port))
                    logger_print(f"✅ Sent ACK for plan {plan_id} to {(sender_addr_ip, ack_port)}")

            except BlockingIOError:
                # No data available - normal
                time.sleep(0.1)
            except Exception as e:
                logger_print(f"Plan check error: {e}")
                time.sleep(0.1)


    def ros_spinner(self):
        """Background thread for ROS2 spinning to process callbacks"""
        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
            except Exception as e:
                logger_print(f"ROS spin error: {e}")
                time.sleep(0.1)

    def start_background_tasks(self):
        """Start background threads"""
        threading.Thread(target=self.status_broadcaster, daemon=True).start()
        threading.Thread(target=self.plan_checker, daemon=True).start()
        threading.Thread(target=self.ros_spinner, daemon=True).start()

    def send_navigation_goal(self, x, y):
        """Send navigation goal and wait for result"""
        try:
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = "map"
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            goal_msg.pose.pose.orientation.w = 1.0
            
            # Wait for server
            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                logger_print("Navigation server not available")
                return False
            
            # Send goal
            send_goal_future = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)
            
            goal_handle = send_goal_future.result()
            if not goal_handle or not goal_handle.accepted:
                logger_print("Navigation goal rejected")
                return False
            
            # Wait for result (with reasonable timeout)
            get_result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=300.0)  # 5 min timeout
            
            result = get_result_future.result()
            success = result and result.status == 4
            
            if success:
                logger_print("Navigation completed successfully")
            else:
                logger_print(f"Navigation failed with status: {result.status if result else 'timeout'}")
            
            return success
            
        except Exception as e:
            logger_print(f"Navigation error: {e}")
            return False

    def send_replan_request(self, reason, action_id=None):
        """Send replan request to manager"""
        msg = {
            "msg_type": "REPLAN_REQUEST",
            "sender": "UGV",
            "reason": reason,
            "current_action": action_id,
            "time": get_sim_time_from_file()
        }
        self.send_udp_message(msg, self.manager_port_rp)
        logger_print(f"Sent replan request: {reason}")

    def execute_action(self, action):
        """Execute a single action"""
        action_id = action.get("ins_id", f"ugv_{int(get_sim_time_from_file())}")
        action_type = action["type"]
        
        logger_print(f"Executing {action_id}: {action_type}")
        success = False
        
        try:
            if action_type == "move_to_location":
                lat, lon = action['location']['lat'], action['location']['lon']
                logger_print(f' Moving to lat, lon: {lat} {lon}')
                utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)
                map_x = utm_x - self.utm_origin_easting
                map_y = utm_y - self.utm_origin_northing
                logger_print(f' Moving to map {map_x},{map_y}')
                success = self.send_navigation_goal(map_x, map_y)
                
            elif action_type == "allow_take_off_from_UGV":

                to_start_time = get_sim_time_from_file()
                while True:
                        if get_sim_time_from_file() - to_start_time > 600: # timeout
                            logger_print(f"Timeout for {action_id}")
                            break
                        try:
                            data, _ = sock_uav.recvfrom(1024)
                            msg = json.loads(data.decode())
                            uav_status.update(msg)
                            status = uav_status["uav_task_status"].get(action_id, {}).get("status")
                            if status == "SUCCESS":
                                logger_print(f"Takeoff confirmed: {action_id}")
                                success = True
                                break
                        except socket.timeout:
                            logger_print('Waiting for UAV takeoff confirmation...')
                            continue
                        except Exception as e:
                            
                            logger_print(f"Error parsing UAV status: {e}")
                            continue

                # Placeholder for UAV handshake
                # time.sleep(0.5)
                # success = True
                
            elif action_type == "allow_land_on_UGV":
                # Placeholder for UAV handshake  
                time.sleep(0.5)
                success = True
   
            else:
                logger_print(f"Unknown action: {action_type}")
                
        except Exception as e:
            logger_print(f"Action {action_id} failed: {e}")
        
        # Update status
        with self.lock:
            self.ugv_task_status[action_id] = {
                "type": action_type,
                "status": "SUCCESS" if success else "FAILED"
            }
        self.update_status_file()
        
        # Check timing
        elapsed = get_sim_time_from_file() - self.ugv_start_time           
        logger_print(f"\n UGV time elapsed: {elapsed}, action end time {action['end_time']}\n")
        # if len(list(self.action_queue)) >=2 and action['type'] != 'allow_take_off_from_UGV': # if len = 1, that means remaining actions = move+land pair and if len = 0 , then only land. also not applicable for takeoff
        #    if "end_time" in action and elapsed > action["end_time"]:
        #     self.send_replan_request(f"Exceeded end_time {action['end_time']}", action_id)
        #     time.sleep(2.0)  # wait for replan
            
        return success

    def run_actions(self):
        """Main action execution loop"""
        while rclpy.ok():
            try:
                # Check if we have actions to execute
                with self.lock:
                    if not self.action_queue:
                        time.sleep(1.0)
                        continue
                    action = self.action_queue.popleft()
                    self.current_action["value"] = action
                
                # Execute action
                st_time = get_sim_time_from_file()
                success = self.execute_action(action)
                exe_time = get_sim_time_from_file() - st_time

                logger_print(f"mission_time_elapsed: {get_sim_time_from_file()-self.ugv_start_time}")
                logger_print(' Time takes to execute action {}: {:.1f}s'.format(action, exe_time))
                
                if not success:
                    action_id = action.get("ins_id", "unknown")
                    logger_print(f"Action failed: {action_id}")
                    self.send_replan_request(f"Action failed: {action_id}")
                    # Don't clear queue - wait for replan
                    time.sleep(5.0)
                
            except Exception as e:
                logger_print(f"Main loop error: {e}")
                time.sleep(1.0)

    def cleanup(self):
        """Clean up resources"""
        try:
            self.sock_tx.close()
            self.sock_rx.close()
            logger_print("UGV runner cleaned up")
        except Exception as e:
            logger_print(f"Cleanup error: {e}")

def main():
    rclpy.init()
    
    # Configuration
    map_origin = (41.870041, -87.650036)  #  3. (41.870041, -87.650036)   2. (41.86997, -87.65018899999995)   1. #(41.87, -87.650411)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    actions_yaml = os.path.join(script_dir, '../config/ugv_actions.yaml')
    ugv_runner = UGVActionRunner(actions_yaml, map_origin)
    
    try:
        ugv_runner.run_actions()
    except KeyboardInterrupt:
        logger_print("UGV runner stopped by user")
    finally:
        ugv_runner.cleanup()
        ugv_runner.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()