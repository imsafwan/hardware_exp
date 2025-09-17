#!/usr/bin/env python3
"""
UGV GPS logger node
- Subscribes to NavSatFix topic from Husky
- Logs timestamp, latitude, longitude, altitude to a CSV every 1 second
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import csv
import os
import datetime


class UGVGPSLogger(Node):
    def __init__(self):
        super().__init__('ugv_gps_logger')

        # Log directory
        log_dir = "gps_logs"
        os.makedirs(log_dir, exist_ok=True)

        # Timestamped file name
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.filename = os.path.join(log_dir, f"ugv_gps_{timestamp}.csv")

        # Open CSV file
        self.file = open(self.filename, mode="w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["ros_time_sec", "latitude", "longitude", "altitude"])

        # Subscribe to GPS topic (just buffer latest message)
        self.subscription = self.create_subscription(
            NavSatFix,
            "/a200_0000/sensors/gps_0/fix",
            self.gps_callback,
            10
        )

        self.latest_gps = None  # store latest NavSatFix
        self.timer = self.create_timer(1.0, self.log_latest_gps)  # log once per second

        self.get_logger().info(f"Logging UGV GPS to {self.filename} (1 Hz)")

    def gps_callback(self, msg: NavSatFix):
        """Buffer latest GPS message"""
        self.latest_gps = msg

    def log_latest_gps(self):
        """Write the latest GPS message once per second"""
        if self.latest_gps:
            stamp = self.get_clock().now().nanoseconds * 1e-9
            self.writer.writerow([
                stamp,
                self.latest_gps.latitude,
                self.latest_gps.longitude,
                self.latest_gps.altitude
            ])
            self.file.flush()

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UGVGPSLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("UGV GPS logger stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
