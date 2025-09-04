#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class ClockLogger(Node):
    def __init__(self):
        super().__init__('clock_logger')
        self.subscription = self.create_subscription(
            Clock,
            '/clock',
            self.listener_callback,
            10
        )
        self.filename = "clock_log.txt"
        self.get_logger().info(f"⏱️ Writing latest /clock time (in seconds) to {self.filename}")

    def listener_callback(self, msg: Clock):
        sim_time_sec = msg.clock.sec + msg.clock.nanosec * 1e-9
        with open(self.filename, "w") as f:   # overwrite each time
            f.write(f"{sim_time_sec:.9f}\n")

def main(args=None):
    rclpy.init(args=args)
    node = ClockLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
