#!/usr/bin/env python3
"""
PX4 bridge for defender: subscribes to /game/defender/velocity, publishes velocity
setpoints to /px4_1/fmu/in/*. Handles arm/offboard state machine and readiness checks.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

from ros2d2.nodes.px4_bridge_common import (
    game_to_ned,
    build_velocity_heartbeat,
    build_velocity_setpoint,
    build_vehicle_command,
)
from ros2d2.config_loader import load_sim_params


class BridgeState:
    """Readiness state machine for PX4 bridge."""
    WAITING_POSITION = "waiting_position"
    STREAMING = "streaming"
    ARMED_OFFBOARD = "armed_offboard"
    ACTIVE = "active"


class Px4BridgeDefenderNode(Node):
    """Bridges game velocity commands to PX4 defender (instance 1)."""

    def __init__(self):
        super().__init__("px4_bridge_defender_node")

        sim_params = load_sim_params()
        bridge_cfg = sim_params.get("bridge", {})
        self._heartbeat_hz = float(bridge_cfg.get("heartbeat_hz", 20.0))
        self._setpoints_before_arm = int(bridge_cfg.get("arm_delay_sec", 1.0) * self._heartbeat_hz)

        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._velocity_cmd = Twist()
        self._velocity_received = False
        self._velocity_stale_time = None
        self._velocity_timeout_sec = 0.5
        self._game_over = False
        self._local_pos = None
        self._vehicle_status = None
        self._state = BridgeState.WAITING_POSITION
        self._setpoint_counter = 0
        self._target_system = 2  # MAV_SYS_ID for px4_instance=1
        self._dt = 1.0 / self._heartbeat_hz

        self._vel_sub = self.create_subscription(
            Twist,
            "/game/defender/velocity",
            self._velocity_cb,
            10,
        )
        self._game_over_sub = self.create_subscription(
            Bool,
            "/game/game_over",
            self._game_over_cb,
            10,
        )
        self._pos_sub = self.create_subscription(
            VehicleLocalPosition,
            "/px4_1/fmu/out/vehicle_local_position_v1",
            self._position_cb,
            qos_px4,
        )
        self._status_sub = self.create_subscription(
            VehicleStatus,
            "/px4_1/fmu/out/vehicle_status_v1",
            self._status_cb,
            qos_px4,
        )

        self._offboard_pub = self.create_publisher(
            OffboardControlMode,
            "/px4_1/fmu/in/offboard_control_mode",
            qos_px4,
        )
        self._setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/px4_1/fmu/in/trajectory_setpoint",
            qos_px4,
        )
        self._cmd_pub = self.create_publisher(
            VehicleCommand,
            "/px4_1/fmu/in/vehicle_command",
            qos_px4,
        )

        self._timer = self.create_timer(self._dt, self._timer_cb)

    def _velocity_cb(self, msg: Twist):
        self._velocity_cmd = msg
        self._velocity_received = True
        self._velocity_stale_time = self.get_clock().now()

    def _game_over_cb(self, msg: Bool):
        self._game_over = msg.data

    def _position_cb(self, msg: VehicleLocalPosition):
        self._local_pos = msg

    def _status_cb(self, msg: VehicleStatus):
        self._vehicle_status = msg

    def _position_valid(self) -> bool:
        if self._local_pos is None:
            return False
        return bool(self._local_pos.xy_valid and self._local_pos.z_valid)

    def _velocity_valid(self) -> bool:
        if self._local_pos is None:
            return False
        return bool(self._local_pos.v_xy_valid and self._local_pos.v_z_valid)

    def _is_armed_offboard(self) -> bool:
        if self._vehicle_status is None:
            return False
        return (
            self._vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and self._vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def _get_effective_velocity(self):
        """Return (vx, vy, vz) in game frame to command. Zero if game over or no valid command."""
        if self._game_over:
            return (0.0, 0.0, 0.0)
        if not self._velocity_received:
            return (0.0, 0.0, 0.0)
        now = self.get_clock().now()
        if self._velocity_stale_time is not None:
            age = (now - self._velocity_stale_time).nanoseconds / 1e9
            if age > self._velocity_timeout_sec:
                return (0.0, 0.0, 0.0)
        return (
            self._velocity_cmd.linear.x,
            self._velocity_cmd.linear.y,
            self._velocity_cmd.linear.z,
        )

    def _timer_cb(self):
        now = self.get_clock().now()
        ts_ns = int(now.nanoseconds)

        # Always publish heartbeat (required for offboard)
        self._offboard_pub.publish(build_velocity_heartbeat(ts_ns))

        # State machine transitions
        if not self._position_valid():
            if self._state != BridgeState.WAITING_POSITION:
                self.get_logger().info(
                    "Defender: not ready — waiting for valid local position (xy_valid, z_valid)"
                )
            self._state = BridgeState.WAITING_POSITION
        else:
            if self._state == BridgeState.WAITING_POSITION:
                self._state = BridgeState.STREAMING
                self._setpoint_counter = 0
                self.get_logger().info("Defender: position valid, streaming setpoints before arm")

        if self._state == BridgeState.STREAMING:
            self._setpoint_counter += 1
            if self._setpoint_counter >= self._setpoints_before_arm:
                self._publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,
                    param2=6.0,
                )
                self._publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0,
                )
                self._state = BridgeState.ARMED_OFFBOARD
                self.get_logger().info("Defender: engage offboard + arm")

        if self._state == BridgeState.ARMED_OFFBOARD and self._is_armed_offboard():
            self._state = BridgeState.ACTIVE
            self.get_logger().info("Defender: armed and offboard — now commanding velocity")

        # Publish setpoint
        if self._state in (BridgeState.STREAMING, BridgeState.ARMED_OFFBOARD):
            # Before active: send zero velocity (hover in place)
            vx, vy, vz = 0.0, 0.0, 0.0
        else:
            vx, vy, vz = self._get_effective_velocity()

        vx_ned, vy_ned, vz_ned = game_to_ned(vx, vy, vz)

        if self._state == BridgeState.ACTIVE and not self._velocity_received:
            self.get_logger().debug(
                "Defender: no velocity command received yet",
                throttle_duration_sec=2.0,
            )

        setpoint = build_velocity_setpoint(vx_ned, vy_ned, vz_ned, ts_ns)
        self._setpoint_pub.publish(setpoint)

    def _publish_vehicle_command(self, command: int, **params):
        ts_ns = int(self.get_clock().now().nanoseconds)
        msg = build_vehicle_command(
            command,
            self._target_system,
            ts_ns,
            param1=params.get("param1", 0.0),
            param2=params.get("param2", 0.0),
        )
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Px4BridgeDefenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
