#!/usr/bin/env python3
"""
State fusion node: subscribes to both PX4 vehicle_local_position topics,
converts NED → game frame (z-up), and publishes nav_msgs/Odometry for defender and attacker.

Subscribes to:
  - /px4_1/fmu/out/vehicle_local_position_v1 (defender)
  - /px4_2/fmu/out/vehicle_local_position_v1 (attacker)

Publishes:
  - /game/defender/state (nav_msgs/Odometry, game frame)
  - /game/attacker/state (nav_msgs/Odometry, game frame)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition


def ned_to_game(px4_x: float, px4_y: float, px4_z: float,
                px4_vx: float, px4_vy: float, px4_vz: float) -> tuple:
    """
    Convert PX4 NED (North-East-Down) to game frame (x, y, z-up).

    NED: x=north, y=east, z=down (negative = above ground)
    Game: x=north, y=east, z=up (positive = above ground)
    """
    game_x = px4_x
    game_y = px4_y
    game_z = -px4_z
    game_vx = px4_vx
    game_vy = px4_vy
    game_vz = -px4_vz
    return (game_x, game_y, game_z, game_vx, game_vy, game_vz)


class StateFusionNode(Node):
    """Fuses defender and attacker PX4 state into game-frame Odometry."""

    def __init__(self):
        super().__init__("state_fusion_node")

        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._defender_pos = None
        self._attacker_pos = None

        self._defender_sub = self.create_subscription(
            VehicleLocalPosition,
            "/px4_1/fmu/out/vehicle_local_position_v1",
            self._defender_cb,
            qos_px4,
        )
        self._attacker_sub = self.create_subscription(
            VehicleLocalPosition,
            "/px4_2/fmu/out/vehicle_local_position_v1",
            self._attacker_cb,
            qos_px4,
        )

        self._defender_pub = self.create_publisher(
            Odometry,
            "/game/defender/state",
            10,
        )
        self._attacker_pub = self.create_publisher(
            Odometry,
            "/game/attacker/state",
            10,
        )

        self._timer = self.create_timer(0.05, self._publish_cb)  # 20 Hz

    def _defender_cb(self, msg: VehicleLocalPosition):
        self._defender_pos = msg

    def _attacker_cb(self, msg: VehicleLocalPosition):
        self._attacker_pos = msg

    def _publish_cb(self):
        now = self.get_clock().now().to_msg()

        if self._defender_pos is not None and self._is_valid(self._defender_pos):
            x, y, z, vx, vy, vz = ned_to_game(
                self._defender_pos.x, self._defender_pos.y, self._defender_pos.z,
                self._defender_pos.vx, self._defender_pos.vy, self._defender_pos.vz,
            )
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "world"
            odom.child_frame_id = "defender"
            odom.pose.pose.position.x = float(x)
            odom.pose.pose.position.y = float(y)
            odom.pose.pose.position.z = float(z)
            odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            odom.twist.twist.linear.x = float(vx)
            odom.twist.twist.linear.y = float(vy)
            odom.twist.twist.linear.z = float(vz)
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = 0.0
            self._defender_pub.publish(odom)

        if self._attacker_pos is not None and self._is_valid(self._attacker_pos):
            x, y, z, vx, vy, vz = ned_to_game(
                self._attacker_pos.x, self._attacker_pos.y, self._attacker_pos.z,
                self._attacker_pos.vx, self._attacker_pos.vy, self._attacker_pos.vz,
            )
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "world"
            odom.child_frame_id = "attacker"
            odom.pose.pose.position.x = float(x)
            odom.pose.pose.position.y = float(y)
            odom.pose.pose.position.z = float(z)
            odom.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            odom.twist.twist.linear.x = float(vx)
            odom.twist.twist.linear.y = float(vy)
            odom.twist.twist.linear.z = float(vz)
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = 0.0
            self._attacker_pub.publish(odom)

    def _is_valid(self, msg: VehicleLocalPosition) -> bool:
        """Check if position and velocity are valid."""
        return bool(
            msg.xy_valid and msg.z_valid
            and msg.v_xy_valid and msg.v_z_valid
        )


def main(args=None):
    rclpy.init(args=args)
    node = StateFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
