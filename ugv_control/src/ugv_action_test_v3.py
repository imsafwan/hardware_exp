#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import utm
import yaml
import os
import json
import time
import socket
import shutil

# === Config ===
TMP_DIR = "tmp"
if os.path.exists(TMP_DIR):
    shutil.rmtree(TMP_DIR)   # delete the folder if it exists
os.makedirs(TMP_DIR, exist_ok=True)

UGV_STATUS_FILE = os.path.join(TMP_DIR, "ugv_status.json")



def update_status(ugv_task_status):
    """Write UGV status JSON file (read by broadcaster)."""
    try:
        with open(UGV_STATUS_FILE, "w") as f:
            json.dump({"ugv_task_status": ugv_task_status}, f)
    except Exception as e:
        print(f"❌ Failed to write UGV status: {e}")


# ==== UAV Listener ====
UAV_PORT = 5005
sock_uav = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_uav.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock_uav.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock_uav.bind(("", UAV_PORT))
uav_status = {}


class UGVActionRunner(Node):
    def __init__(self, actions_yaml, map_origin):
        super().__init__('ugv_action_runner')
        self.actions_yaml = os.path.expanduser(actions_yaml)
        self.map_origin = map_origin

        utm_x, utm_y, zone_number, _ = utm.from_latlon(map_origin[0], map_origin[1])
        self.utm_origin_easting = utm_x
        self.utm_origin_northing = utm_y
        self.utm_zone_number = zone_number

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_navigation_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending Goal to Nav2: X={x}, Y={y}")

        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available! Navigation goal aborted.")
            return False

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal Rejected!")
            return False

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        return result and result.status == 4

    def run_actions(self):
        ugv_task_status = {}

        try:
            with open(self.actions_yaml, 'r') as f:
                actions = yaml.safe_load(f).get('actions', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load actions YAML: {e}")
            return

        # Init all actions as PENDING
        for i, action in enumerate(actions):
            action_id = action.get("ins_id", f"ugv_action_{i+1}")
            ugv_task_status[action_id] = {"type": action["type"], "status": "PENDING"}
        update_status(ugv_task_status)

        for action in actions:
            action_id = action.get("ins_id", f"ugv_action_{action['type']}")
            success = False
            print(action)

            if action["type"] == "allow_take_off_from_UGV":
                self.get_logger().info(f"Waiting for UAV to take off: {action_id}")
                start_time = time.time()
                #sock_uav.settimeout(1.0)
                while True:
                        
                        if time.time() - start_time > 120:
                            self.get_logger().error(f"⏳ Timeout for {action_id}")
                            break
                        
                        
                        try:
                            data, _ = sock_uav.recvfrom(1024)
                            msg = json.loads(data.decode())
                            uav_status.update(msg)
                            status = uav_status["uav_task_status"].get(action_id, {}).get("status")
                            print('status received:', status)
                            if status == "SUCCESS":
                                self.get_logger().info(f"✅ Takeoff confirmed: {action_id}")
                                success = True
                                break
                        except socket.timeout:
                            # No packet received this second → retry
                            continue
                        except Exception as e:
                            self.get_logger().warn(f"⚠️ Error parsing UAV status: {e}")
                            continue

            elif action["type"] == "move_to_location":
                print('move to location')
                time.sleep(5)  # mock navigation
                success = True
                '''lat, lon = action['location']['lat'], action['location']['lon']
                utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)
                map_x, map_y = utm_x - self.utm_origin_easting, utm_y - self.utm_origin_northing
                success = self.send_navigation_goal(map_x, map_y)'''

            elif action["type"] == "allow_land_on_UGV":
                self.get_logger().info("Allowing UAV to land")
                success = True

            ugv_task_status[action_id]["status"] = "SUCCESS" if success else "FAILED"
            update_status(ugv_task_status)

            if not success:
                self.get_logger().error(f"❌ Stopping sequence at {action_id}")
                break

        

        self.get_logger().info("UGV sortie complete.")


def main():
    rclpy.init()
    map_origin = (41.86996889940987, -87.65044364173002)  # Replace with real origin
    script_dir = os.path.dirname(os.path.abspath(__file__))
    actions_yaml = os.path.join(script_dir, '../config/ugv_actions.yaml')

    ugv_runner = UGVActionRunner(actions_yaml, map_origin)
    try:
        ugv_runner.run_actions()
    finally:
        ugv_runner.destroy_node()
        rclpy.shutdown()
        print('<----done ----->')
        


if __name__ == "__main__":
    main()
