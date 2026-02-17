#!/usr/bin/env python
from dataclasses import dataclass
from typing import Optional
import math
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .dubins_flat_planning_algorithm import (
    BoundaryState,
    ControlTraj,
    StateTraj,
    plan_cubic_flat_trajectory,
)


# ==============================
# Helper class: Do not modify!
# ==============================
@dataclass
class Plan:
    state: StateTraj
    ctrl: ControlTraj
    t0_ros: rclpy.time.Time  # start time in ROS clock


# ======================
# Helper: Do not modify!
# ======================
def euler_from_quaternion(x: float, y: float, z: float, w: float) -> np.ndarray:
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    t0 = 2 * (w * x + y * z)

    t1 = 1 - 2 * (x * x + y * y)

    t2 = 2 * (w * y - x * z)
    # Make sure we're in range of asin
    if t2 > 1:
        t2 = 1
    if t2 < -1:
        t2 = -1

    t3 = 2 * (w * z + x * y)

    t4 = 1 - 2 * (y * y + z * z)

    roll = math.atan2(t0, t1)
    pitch = math.asin(t2)
    yaw = math.atan2(t3, t4)

    return np.array([roll, pitch, yaw])


# ====================================
# Complete implementation of this node
# ====================================
class DubinsFlatPlanner(Node):
    def __init__(self) -> None:
        super().__init__("dubins_flat_planner")

        # goal state
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_theta", float(np.pi / 2.0))
        self.declare_parameter("goal_v", 0.5)

        # planning horizon and sampling
        self.declare_parameter("T", 5.0)  # seconds
        self.declare_parameter("plan_dt", 0.01)  # seconds
        self.declare_parameter("control_rate", 50.0)  # Hz

        # NOTE: if robot needs a fixed linear speed for cmd_vel, set use_planned_speed=False and set fixed_speed
        self.declare_parameter("use_planned_speed", True)
        self.declare_parameter("fixed_speed", 0.5)

        # topics
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        odom_topic = str(self.get_parameter("odom_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)

        self._sub_odom = self.create_subscription(
            Odometry, odom_topic, self._on_odom, 10
        )
        self._pub_cmd = self.create_publisher(Twist, cmd_vel_topic, 10)

        rate_hz = float(self.get_parameter("control_rate").value)
        period = 1.0 / rate_hz if rate_hz > 0 else 0.02
        self._timer = self.create_timer(period, self._on_timer)

        self._latest_start_state: Optional[BoundaryState] = None
        self._plan: Optional[Plan] = None
        self._planned_once: bool = False

        self.get_logger().info("DubinsFlatPlanner node up. Waiting for odometry...")

    def _on_odom(self, msg: Odometry) -> None:
        """
        Obtains the state from odometry data.
        """
        # extract pose (x, y, yaw)
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)
        theta = self._yaw_from_pose(msg.pose.pose)

        # extract speed magnitude from odom twist (in base_link frame typically)
        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        v = float(np.hypot(vx, vy))

        self._latest_start_state = BoundaryState(x=px, y=py, theta=theta, v=v)

        # plan once on first odom (open-loop)
        if (not self._planned_once) and (self._latest_start_state is not None):
            self._plan_from_current()

    def _plan_from_current(self) -> None:
        """
        Create an open-loop plan from the current state to the goal, as specified in
        the parameters of the node

        Steps:
        1. Define the goal as a BoundaryState
        2. Call open-loop planner to obtain state and control trajectories
        3. Package result into self._plan and disable further planning
        """
        if self._latest_start_state is None:
            return

        # 1. Define goal
        goal = BoundaryState(
            x=float(self.get_parameter("goal_x").value),
            y=float(self.get_parameter("goal_y").value),
            theta=float(self.get_parameter("goal_theta").value),
            v=float(self.get_parameter("goal_v").value),
        )

        # 2. Call planner
        state_traj = StateTraj()
        ctrl_traj = ControlTraj()
        T = float(self.get_parameter("T").value)

        # STUDENT CODE START
        state_traj, ctrl_traj = plan_cubic_flat_trajectory(
            start=self._latest_start_state,
            goal=goal,
            T=T,
            dt=float(self.get_parameter("plan_dt").value),
        )
        # STUDENT CODE END

        # 3. Package result and disable further planning
        self._plan = Plan(
            state=state_traj, ctrl=ctrl_traj, t0_ros=self.get_clock().now()
        )
        self._planned_once = True

        self.get_logger().info(
            "Planned open-loop trajectory:"
            f" T={T:.3f}s, N={len(state_traj.t)},"
            f" start=({self._latest_start_state.x:.3f},{self._latest_start_state.y:.3f},{self._latest_start_state.theta:.3f},{self._latest_start_state.v:.3f}),"
            f" goal=({goal.x:.3f},{goal.y:.3f},{goal.theta:.3f},{goal.v:.3f})"
        )

    def _on_timer(self) -> None:
        """
        Publish controls corresponding to the planned open-loop trajectory

        Steps:
        1. Sanity checks to see whether planning is actually needed
        2. Find the nearest index of the velocity (linear and angular) command that
           should be published
        3. Publish the command
        """
        # 1. Sanity checks
        if self._plan is None:
            return

        now = self.get_clock().now()
        t_elapsed = (now - self._plan.t0_ros).nanoseconds * 1e-9

        # stop when plan complete
        t_final = float(self._plan.ctrl.t[-1])
        if t_elapsed >= t_final:
            self._publish_cmd(v=0.0, omega=0.0)
            return

        # 2. Get correct velocity command based on time
        v_cmd = 0
        omega_cmd = 0

        # STUDENT CODE START
        idx = np.searchsorted(self._plan.state.t, t_elapsed, side="right") - 1
        v_cmd = float(self._plan.state.v[idx])
        omega_cmd = float(self._plan.ctrl.omega[idx])
        use_planned_speed = bool(self.get_parameter("use_planned_speed").value)
        if not use_planned_speed:
            v_cmd = float(self.get_parameter("fixed_speed").value)
            omega_cmd = 0.0
        # STUDENT CODE END

        # 3. Publish velocity command
        self._publish_cmd(v=v_cmd, omega=omega_cmd)

    def _publish_cmd(self, v: float, omega: float) -> None:
        """
        Publishes the specified linear velocity v and angular velocity w as control
        to cmd_vel_topic
        """
        msg = Twist()
        # STUDENT CODE START
        msg.linear.x = v
        msg.angular.z = omega
        # STUDENT CODE END

        self._pub_cmd.publish(msg)

    # =======================
    # Helper: Do not modify!
    # =======================
    @staticmethod
    def _yaw_from_pose(pose) -> float:
        _, _, yaw = euler_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return float(yaw)


def main() -> None:
    rclpy.init()
    node = DubinsFlatPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._publish_cmd(v=0.0, omega=0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
