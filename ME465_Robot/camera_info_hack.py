import rclpy
from sensor_msgs.msg import CameraInfo
from rclpy.node import Node

class CameraInfoNode(Node):
    P = [381.36246688113556, 0.0, 320.5, 0.0, 0.0, 381.36246688113556, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]

    def __init__(self):
        super().__init__("camera_info_node")
        self.pub = self.create_publisher(
            CameraInfo,
            "/camera2/camera_info",
            5,
        )
        self.create_timer(5, self.timer_callback)

    def timer_callback(self):
        msg = CameraInfo(p=self.P)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
