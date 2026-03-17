#!/usr/bin/env python3
"""
Capture detection: defender within d_h (horizontal) and d_z (vertical) of attacker.

Subscribes to:
  - /game/defender/state (nav_msgs/Odometry)
  - /game/attacker/state (nav_msgs/Odometry)

Publishes:
  - /game/game_over (std_msgs/Bool) — True when capture occurs

Loads d_h, d_z from config/game_params.yaml.
"""

import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from ros2d2.config_loader import load_game_params


class CaptureDetectionNode(Node):
    """Checks capture: defender within d_h and d_z of attacker. Publishes game_over."""

    def __init__(self):
        super().__init__("capture_detection_node")

        self._params = load_game_params()
        self._defender_state = None
        self._attacker_state = None
        self._game_over = False

        self._defender_sub = self.create_subscription(
            Odometry,
            "/game/defender/state",
            self._defender_cb,
            10,
        )
        self._attacker_sub = self.create_subscription(
            Odometry,
            "/game/attacker/state",
            self._attacker_cb,
            10,
        )

        self._game_over_pub = self.create_publisher(Bool, "/game/game_over", 10)

        self._timer = self.create_timer(0.05, self._check_cb)  # 20 Hz

    def _defender_cb(self, msg: Odometry):
        self._defender_state = msg

    def _attacker_cb(self, msg: Odometry):
        self._attacker_state = msg

    def _check_cb(self):
        if self._game_over:
            return

        if self._defender_state is None or self._attacker_state is None:
            return

        p_d = self._defender_state.pose.pose.position
        p_a = self._attacker_state.pose.pose.position

        dx = p_d.x - p_a.x
        dy = p_d.y - p_a.y
        dz = abs(p_d.z - p_a.z)

        dist_h = math.sqrt(dx * dx + dy * dy)
        dist_z = dz

        if dist_h <= self._params.d_h and dist_z <= self._params.d_z:
            self._game_over = True
            self.get_logger().info(
                "CAPTURE! Defender within d_h=%.2f m (%.2f) and d_z=%.2f m (%.2f)",
                self._params.d_h, dist_h, self._params.d_z, dist_z,
            )
            msg = Bool()
            msg.data = True
            self._game_over_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CaptureDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
