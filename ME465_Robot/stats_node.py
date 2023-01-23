import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String

import subprocess


class StatsNode(Node):
    def __init__(self):
        super().__init__("state_node")
        self.battery_publisher = self.create_publisher(
            Float32,
            "/laptop_battery",
            5,
        )
        self.ip_publisher = self.create_publisher(
            String,
            "/ip",
            5,
        )
        self.create_timer(2, self.timer_callback)
    
    def timer_callback(self):
        self.publish_ip()
        self.publish_battery()

    def publish_ip(self):
        msg = String()
        output = subprocess.check_output(["ip", "a", "show", "dev", "wlp2s0"]).decode("utf8")
        msg.data = output.split("\n")[2].strip().split(" ")[1].split("/")[0]
        self.ip_publisher.publish(msg)

    def publish_battery(self):
        with open("/sys/class/power_supply/BAT0/charge_full") as f:
            full = int(f.read())
        with open("/sys/class/power_supply/BAT0/charge_now") as f:
            now = int(f.read())
        msg = Float32()
        msg.data = 100 * now / full
        self.battery_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    stats_node = StatsNode()

    rclpy.spin(stats_node)


if __name__ == '__main__':
    main()
