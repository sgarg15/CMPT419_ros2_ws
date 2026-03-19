"""ROS2 planner frontend"""

import importlib
import math
from typing import Any, Dict, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from mpc.controller_base import ControllerBackend


def euler_from_quaternion(
    x: float, y: float, z: float, w: float
) -> Tuple[float, float, float]:
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - x * z)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


def load_backend_class(backend_class_path: str):
    if ":" not in backend_class_path:
        raise ValueError(
            f"backend_class must be 'module.path:ClassName'. Got '{backend_class_path}'"
        )

    module_name, class_name = backend_class_path.split(":", 1)
    module = importlib.import_module(module_name.strip())
    backend_cls = getattr(module, class_name.strip(), None)
    if backend_cls is None:
        raise AttributeError(
            f"Class '{class_name}' not found in module '{module_name}'."
        )
    return backend_cls


class MPCPlanner(Node):
    def __init__(self) -> None:
        super().__init__("mpc_planner")

        # fmt: off
        self.declare_parameter("backend_class", "mpc.nmpc_algorithm:NMPCController")

        self.declare_parameter("dt", 0.1)
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("seed", 7)
        self.declare_parameter("goal_x", 4.0)
        self.declare_parameter("goal_y", 1.0)
        self.declare_parameter("goal_theta", 0.0)
        self.declare_parameter("v_min", 0.0)
        self.declare_parameter("v_max", 0.5)
        self.declare_parameter("omega_max", 0.8)
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("x_knots", [0.0, 2.0, 4.0, 6.0, 8.0])
        self.declare_parameter("y_low_knots", [-1.0, -1.4, -1.6, -1.6, -1.4])
        self.declare_parameter("y_high_knots", [1.0, 1.4, 1.6, 1.6, 1.4])
        self.declare_parameter("w_pos", 2.0)
        self.declare_parameter("w_theta", 0.1)
        self.declare_parameter("w_u", 0.05)
        self.declare_parameter("w_corr", 5000.0)

        ## MPPI backend params
        self.declare_parameter("mppi.n_traj", 128)
        self.declare_parameter("mppi.horizon", 32)
        self.declare_parameter("mppi.noise_sigma_v", 0.7)
        self.declare_parameter("mppi.noise_sigma_omega", 0.6)
        self.declare_parameter("mppi.temperature", 1.0)
        self.declare_parameter("mppi.w_du", 0.25)

        ## NMPC backend params
        self.declare_parameter("mpc.N", 80)
        self.declare_parameter("mpc.w_du", 0.2)
        self.declare_parameter("mpc.w_pos_T", 50.0)
        self.declare_parameter("mpc.w_theta_T", 0.2)
        self.declare_parameter("mpc.solver_max_iter", 300)
        self.declare_parameter("mpc.solver_tol", 1.0e-6)
        self.declare_parameter("mpc.solver_constr_viol_tol", 1.0e-4)
        # fmt: on

        self._backend = self._build_backend()

        pose_topic = str(self.get_parameter("pose_topic").value)
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        rate_hz = max(1e-3, float(self.get_parameter("control_rate").value))

        backend_class = str(self.get_parameter("backend_class").value)

        # TODO: Add a subscriber, publisher, and timer
        # =========================
        # STUDENT CODE START
        self._pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self._pose_callback, 10
        )
        self._cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_timer(1.0 / rate_hz, self._timer_callback)
        # STUDENT CODE END

        self.get_logger().info(
            f"MPCPlanner up. backend_class={backend_class}, rate={rate_hz:.1f}Hz"
        )


    def _build_backend(self) -> ControllerBackend:
        backend_class_path = str(self.get_parameter("backend_class").value)
        backend_cls = load_backend_class(backend_class_path)
        config = self._read_backend_config()
        backend: ControllerBackend = backend_cls(config)

        if not isinstance(backend, ControllerBackend):
            raise TypeError(
                f"Backend class '{backend_class_path}' must implement ControllerBackend."
            )
        return backend

    def _read_backend_config(self) -> Dict[str, Any]:
        """Regulate config from different backends."""
        return {
            "dt": float(self.get_parameter("dt").value),
            "seed": int(self.get_parameter("seed").value),
            "goal": np.array(
                [
                    float(self.get_parameter("goal_x").value),
                    float(self.get_parameter("goal_y").value),
                    float(self.get_parameter("goal_theta").value),
                ],
                dtype=float,
            ),
            "v_min": float(self.get_parameter("v_min").value),
            "v_max": float(self.get_parameter("v_max").value),
            "omega_max": float(self.get_parameter("omega_max").value),
            "corridor": {
                "x_knots": list(self.get_parameter("x_knots").value),
                "y_low_knots": list(self.get_parameter("y_low_knots").value),
                "y_high_knots": list(self.get_parameter("y_high_knots").value),
            },
            "weights": {
                "w_pos": float(self.get_parameter("w_pos").value),
                "w_theta": float(self.get_parameter("w_theta").value),
                "w_u": float(self.get_parameter("w_u").value),
                "w_corr": float(self.get_parameter("w_corr").value),
            },
            "mppi": {
                "n_traj": int(self.get_parameter("mppi.n_traj").value),
                "horizon": int(self.get_parameter("mppi.horizon").value),
                "noise_sigma_v": float(self.get_parameter("mppi.noise_sigma_v").value),
                "noise_sigma_omega": float(
                    self.get_parameter("mppi.noise_sigma_omega").value
                ),
                "temperature": float(self.get_parameter("mppi.temperature").value),
                "w_du": float(self.get_parameter("mppi.w_du").value),
            },
            "mpc": {
                "N": int(self.get_parameter("mpc.N").value),
                "w_du": float(self.get_parameter("mpc.w_du").value),
                "w_pos_T": float(self.get_parameter("mpc.w_pos_T").value),
                "w_theta_T": float(self.get_parameter("mpc.w_theta_T").value),
                "solver_max_iter": int(self.get_parameter("mpc.solver_max_iter").value),
                "solver_tol": float(self.get_parameter("mpc.solver_tol").value),
                "solver_constr_viol_tol": float(
                    self.get_parameter("mpc.solver_constr_viol_tol").value
                ),
            },
        }

    # TODO: Implement callback functions for subscriber and timer
    # =========================
    # STUDENT CODE START
    def _pose_callback(self, msg: PoseStamped):
        # Extract the robot's pose from the Odometry message and update the backend
        position = msg.pose.position
        orientation = msg.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        self._current_state = np.array([position.x, position.y, yaw], dtype=float)
    
    def _timer_callback(self):
        
        if not hasattr(self, '_current_state'):
            return
        # Compute the control command from the backend and publish it as a Twist message
        cmd_vel = self._backend.get_action(self._current_state)
        if cmd_vel is None:
            return
        cmd_vel = np.asarray(cmd_vel, dtype=float).reshape(2)
        if np.any(np.isnan(cmd_vel)):
            return

        twist_msg = Twist()
        twist_msg.linear.x = float(np.clip(cmd_vel[0], self.get_parameter('v_min').value, self.get_parameter('v_max').value))
        twist_msg.angular.z = float(np.clip(cmd_vel[1], -self.get_parameter('omega_max').value, self.get_parameter('omega_max').value))
        self._cmd_vel_pub.publish(twist_msg)
    # STUDENT CODE END


def main():
    rclpy.init()
    node = MPCPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()