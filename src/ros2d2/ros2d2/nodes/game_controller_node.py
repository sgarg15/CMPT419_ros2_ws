#!/usr/bin/env python3
"""
Game controller node: subscribes to defender/attacker state, runs simple policies,
publishes desired velocity commands.

Subscribes to:
  - /game/defender/state (nav_msgs/Odometry)
  - /game/attacker/state (nav_msgs/Odometry)

Publishes:
  - /game/defender/velocity (geometry_msgs/Twist)
  - /game/attacker/velocity (geometry_msgs/Twist)

Modes:
  - normal: pure pursuit (defender), straight-line toward -x (attacker)
  - test_scripted: independent scripted motion for validation (no state needed)
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from ros2d2.config_loader import load_game_params
from ros2d2.dynamics import DefenderState, AttackerState
from ros2d2.controllers.simple_policies import pure_pursuit_policy, get_default_attacker_policy


def _scripted_velocity(t: float, role: str) -> tuple:
    """
    Scripted independent motion for two-drone validation test.
    Role: 'defender' or 'attacker'
    Returns (vx, vy, vz) in game frame.
    """
    # 0-3s: hover
    # 3-8s: defender vx=1, attacker vy=1
    # 8s+: stop
    if t < 3.0:
        return (0.0, 0.0, 0.0)
    if t >= 8.0:
        return (0.0, 0.0, 0.0)
    if role == "defender":
        return (1.0, 0.0, 0.0)
    return (0.0, 1.0, 0.0)  # attacker


class GameControllerNode(Node):
    """Runs defender/attacker policies and publishes velocity commands."""

    def __init__(self):
        super().__init__("game_controller_node")

        self.declare_parameter("control_mode", "normal")
        self._control_mode = self.get_parameter("control_mode").get_parameter_value().string_value

        self._params = load_game_params()
        self._attacker_policy = get_default_attacker_policy()

        self._defender_state = None
        self._attacker_state = None
        self._start_time = None

        if self._control_mode == "test_scripted":
            self.get_logger().info("Game controller: test_scripted mode — independent motion test")

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

        self._defender_vel_pub = self.create_publisher(
            Twist,
            "/game/defender/velocity",
            10,
        )
        self._attacker_vel_pub = self.create_publisher(
            Twist,
            "/game/attacker/velocity",
            10,
        )

        self._timer = self.create_timer(0.05, self._control_cb)  # 20 Hz

    def _defender_cb(self, msg: Odometry):
        self._defender_state = msg

    def _attacker_cb(self, msg: Odometry):
        self._attacker_state = msg

    def _odom_to_defender(self, odom: Odometry) -> DefenderState:
        p = odom.pose.pose.position
        v = odom.twist.twist.linear
        return DefenderState(
            x=p.x, y=p.y, z=p.z,
            vx=v.x, vy=v.y, vz=v.z,
        )

    def _odom_to_attacker(self, odom: Odometry) -> AttackerState:
        p = odom.pose.pose.position
        return AttackerState(x=p.x, y=p.y, z=p.z)

    def _control_cb(self):
        if self._start_time is None:
            self._start_time = self.get_clock().now()

        t = (self.get_clock().now() - self._start_time).nanoseconds / 1e9

        if self._control_mode == "test_scripted":
            # Time-based scripted motion; no state needed
            u_D = _scripted_velocity(t, "defender")
            u_A = _scripted_velocity(t, "attacker")
        else:
            # Normal mode: requires both states
            if self._defender_state is None or self._attacker_state is None:
                return
            defender = self._odom_to_defender(self._defender_state)
            attacker = self._odom_to_attacker(self._attacker_state)
            u_D = pure_pursuit_policy(defender, attacker, self._params, t)
            u_A = self._attacker_policy(attacker, defender, self._params, t)

        twist_D = Twist()
        twist_D.linear.x = float(u_D[0])
        twist_D.linear.y = float(u_D[1])
        twist_D.linear.z = float(u_D[2])

        twist_A = Twist()
        twist_A.linear.x = float(u_A[0])
        twist_A.linear.y = float(u_A[1])
        twist_A.linear.z = float(u_A[2])

        self._defender_vel_pub.publish(twist_D)
        self._attacker_vel_pub.publish(twist_A)


def main(args=None):
    rclpy.init(args=args)
    node = GameControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
