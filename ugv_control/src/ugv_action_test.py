#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import utm
import yaml
import time
import os
import socket
import json


# Load namespace dynamically from YAML
def load_namespace():
    yaml_path = os.path.expanduser("~/clearpath/robot.yaml")  # Path to your YAML file
    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
            return config['system']['ros2']['namespace']
    except Exception as e:
        print(f"Error loading namespace from YAML: {e}")
        return "default_ns"  # Fallback namespace


# ==== UDP Broadcast ====
def broadcast_status(full_status):
    BROADCAST_IP = "192.168.10.15" # "255.255.255.255"
    UAV_LISTEN_PORT = 5006
    C_LISTEN_PORT = 5056
    try:
        msg = json.dumps(full_status)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.sendto(msg.encode(), (BROADCAST_IP, UAV_LISTEN_PORT))
        sock.sendto(msg.encode(), (BROADCAST_IP, C_LISTEN_PORT))
        sock.close()
        print('broadcasted')
        #log.info(f"📡 Broadcasted status: {msg}")
    except Exception as e:
        print(f"❌ Failed to broadcast status: {e}")
        #log.error(f"❌ Failed to broadcast status: {e}")


UAV_PORT = 5005
sock_uav = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_uav.bind(("", UAV_PORT))
uav_status = {}


class UGVActionRunner(Node):
    def __init__(self, actions_yaml, map_origin):
        super().__init__('ugv_action_runner')
        self.actions_yaml = os.path.expanduser(actions_yaml)
        self.map_origin = map_origin  # (lat, lon)
        self.utm_origin_easting = None
        self.utm_origin_northing = None
        self.utm_zone_number = None

        # Set UTM origin from map_origin
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

        # Wait for the action server asynchronously
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
            self.get_logger().info("Navigation Goal Reached!")
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
        


        # Initialize mission status with PENDING for all actions
        for i, action in enumerate(actions):
            action_id = action['ins_id'] if 'ins_id' in action else f"ugv_action_{action.get('type', 'unknown')}"
            ugv_task_status[action_id] = {
                "type": action["type"],
                "status": "PENDING"
            }

        for action in actions:
            
            action_id = action['ins_id'] if 'ins_id' in action else f"ugv_action_{action.get('type', 'unknown')}"


            if action.get('type') == 'allow_take_off_from_UGV':
                self.get_logger().info(f"Waiting for takeoff allowed for action: {action_id}")
                #success = True
                start_time = time.time()

                while True:
                    
                    if time.time() - start_time > 120:   # 120 sec timeout
                        self.get_logger().error(f"⏳ Timeout: Takeoff not completed for {action_id} within 120s. Aborting.")
                        return False   

                    data, _ = sock_uav.recvfrom(1024)
                    try:
                        msg = json.loads(data.decode())
                        uav_status.update(msg)

                        status = uav_status["uav_task_status"].get(action_id, {}).get("status")

                        if status == "SUCCESS":
                            self.get_logger().info(f" Takeoff allowed for action: {action_id}")
                            success = True
                            break
                        else:
                            self.get_logger().info(f" Waiting for takeoff success for {action_id}, current status: {status}")
                            success = False               
                    except Exception as e:
                        self.get_logger().warn(f" Error parsing UAV status: {e}")
                        success = False 
                
                                                        


            elif action.get('type') == 'move_to_location':
                # Handle location as a list of dicts (YAML structure)
                
                lat = action['location']['lat']
                lon = action['location']['lon']
                
                utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)
                map_x = utm_x - self.utm_origin_easting
                map_y = utm_y - self.utm_origin_northing
                self.get_logger().info(f"Navigating to GPS ({lat}, {lon}) -> Map ({map_x}, {map_y})")
                success = self.send_navigation_goal(map_x, map_y)
                if not success:
                    self.get_logger().error("Stopping action sequence due to failure.")
                    break
                time.sleep(2)  # Optional: wait before next action


            elif action.get('type') == 'allow_land_on_UGV':
                success = True

                # Wait for UAV status broadcast and check if land is allowed
                # self.get_logger().info(f"Waiting for land allowed for action: {action_id}")
                # while True:
                #     data, addr = sock.recvfrom(1024)
                #     try:
                #         msg = json.loads(data.decode())
                #         uav_status.update(msg)
                #         status = uav_status["uav_task_status"].get(action_id, {}).get("status")
                #         if status == "SUCCESS":
                #             self.get_logger().info(f"Land allowed for action: {action_id}")
                #             break
                #         else:
                #             self.get_logger().info(f"Waiting for land success for {action_id}, current status: {status}")
                #     except Exception as e:
                #         self.get_logger().warn(f"Error parsing UAV status: {e}")
                # continue

            else:
                self.get_logger().warn(f"Unknown action type: {action.get('type')}. Skipping.")
                success = False

            # Update mission status
            ugv_task_status[action_id]["status"] = "SUCCESS" if success else "FAILED"

            # Broadcast the entire mission status so all agents can see progress
            broadcast_status({
                "ugv_task_status": ugv_task_status
            })





def main():
    rclpy.init()
    # Set your map origin here (lat, lon)
    map_origin = (41.86996889940987,  -87.65044364173002)  # Example, replace with your real map origin
    
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
