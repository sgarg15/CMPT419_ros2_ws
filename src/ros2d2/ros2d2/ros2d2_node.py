import rclpy
from rclpy.node import Node


class Ros2d2Node(Node):
    """ROS2 node for ros2d2 package."""

    def __init__(self):
        super().__init__("ros2d2_node")
        self.get_logger().info("ros2d2 node started")


def main(args=None):
    rclpy.init(args=args)
    node = Ros2d2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
