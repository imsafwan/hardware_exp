#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import utm
import yaml
import time
import os
import socket
import json
import threading


# Load namespace dynamically from YAML
def load_namespace():
    yaml_path = os.path.expanduser("~/clearpath/robot.yaml")
    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
            return config['system']['ros2']['namespace']
    except Exception as e:
        print(f"Error loading namespace from YAML: {e}")
        return "default_ns"


# ==== Heartbeat Thread with Persistent Socket ====
def heartbeat_broadcast(ugv_task_status, stop_event):
    BROADCAST_IP =  "192.168.10.255"   # subnet broadcast or 255.255.255.255
    UAV_LISTEN_PORT = 5006
    C_LISTEN_PORT = 5056

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    try:
        while not stop_event.is_set():
            try:
                msg = json.dumps({"ugv_task_status": ugv_task_status})
                sock.sendto(msg.encode(), (BROADCAST_IP, UAV_LISTEN_PORT))
                sock.sendto(msg.encode(), (BROADCAST_IP, C_LISTEN_PORT))
                #print(f"📡 HB → {BROADCAST_IP}:{UAV_LISTEN_PORT}/{C_LISTEN_PORT} | {msg}")
            except Exception as e:
                print(f"❌ Heartbeat broadcast error: {e}")
            time.sleep(1.0)   # 1 Hz
    finally:
        sock.close()
        print("🛑 Heartbeat socket closed.")


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

        namespace = load_namespace()
        self.namespace = namespace
        self.get_logger().info(f"Using namespace: {namespace}")
        self.nav_client = ActionClient(self, NavigateToPose, f'{namespace}/navigate_to_pose')

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
        if result and result.status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info("✅ Navigation Goal Reached!")
            return True
        elif result:
            self.get_logger().error(f"Navigation Failed! Status: {result.status}")
            return False
        else:
            self.get_logger().error("Navigation Failed! No result returned.")
            return False

    def run_actions(self):
        ugv_task_status = {}

        try:
            with open(self.actions_yaml, 'r') as f:
                actions = yaml.safe_load(f).get('actions', [])
        except Exception as e:
            self.get_logger().error(f"Failed to load actions YAML: {e}")
            return

        for i, action in enumerate(actions):
            action_id = action['ins_id'] if 'ins_id' in action else f"ugv_action_{action.get('type', 'unknown')}"
            ugv_task_status[action_id] = {"type": action["type"], "status": "PENDING"}

        # ---- Start heartbeat thread ----
        stop_event = threading.Event()
        hb_thread = threading.Thread(target=heartbeat_broadcast, args=(ugv_task_status, stop_event))
        hb_thread.start()

        try:
            for action in actions:
                action_id = action['ins_id'] if 'ins_id' in action else f"ugv_action_{action.get('type', 'unknown')}"
                success = False

                if action.get('type') == 'allow_take_off_from_UGV':
                    self.get_logger().info(f"Waiting for takeoff allowed for {action_id}")
                    start_time = time.time()
                    while True:
                        if time.time() - start_time > 120:
                            self.get_logger().error(f"⏳ Timeout for {action_id}")
                            break
                        data, _ = sock_uav.recvfrom(1024)
                        try:
                            msg = json.loads(data.decode())
                            uav_status.update(msg)
                            status = uav_status["uav_task_status"].get(action_id, {}).get("status")
                            if status == "SUCCESS":
                                self.get_logger().info(f"✅ Takeoff confirmed: {action_id}")
                                success = True
                                break
                        except Exception as e:
                            self.get_logger().warn(f"⚠️ Error parsing UAV status: {e}")

                elif action.get('type') == 'move_to_location':
                    '''lat, lon = action['location']['lat'], action['location']['lon']
                    utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)
                    map_x, map_y = utm_x - self.utm_origin_easting, utm_y - self.utm_origin_northing
                    self.get_logger().info(f"Navigating to GPS ({lat}, {lon}) -> Map ({map_x}, {map_y})")
                    success = self.send_navigation_goal(map_x, map_y)'''
                    time.sleep(5)
                    success = True
                 
                elif action.get('type') == 'allow_land_on_UGV':
                    self.get_logger().info(f"Allowing land for {action_id}")
                    success = True

                else:
                          self.get_logger().warn(f"Unknown action type: {action.get('type')}")
                          success = False

                ugv_task_status[action_id]["status"] = "SUCCESS" if success else "FAILED"

                if not success:
                    self.get_logger().error("Stopping sequence due to failure.")
                    break
        finally:
            self.get_logger().info("UGV finished its actions. Waiting for UAV to finish...")

            while True:
                try:
                    data, _ = sock_uav.recvfrom(1024)
                    msg = json.loads(data.decode())
                    uav_status.update(msg)

                    task_dict = uav_status.get("uav_task_status", {})
                    if task_dict:
                        # check if every action in UAV is SUCCESS
                        if all(v.get("status") == "SUCCESS" for v in task_dict.values()):
                            self.get_logger().info("✅ All UAV actions completed SUCCESS.")
                            break
                        else:
                            self.get_logger().info(f"⏳ UAV still running: {task_dict}")
                except Exception as e:
                    self.get_logger().warn(f"⚠️ Error receiving UAV status: {e}")
                    continue

            stop_event.set()
            hb_thread.join()
            self.get_logger().info("🛑 Heartbeat stopped. UGV shutting down.")


        

def main():
    rclpy.init()
    map_origin = (41.86996889940987, -87.65044364173002)  # Replace with real map origin
    script_dir = os.path.dirname(os.path.abspath(__file__))
    actions_yaml = os.path.join(script_dir, '../config/ugv_actions.yaml')

    ugv_runner = UGVActionRunner(actions_yaml, map_origin)
    try:
        ugv_runner.run_actions()
    finally:
        ugv_runner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
