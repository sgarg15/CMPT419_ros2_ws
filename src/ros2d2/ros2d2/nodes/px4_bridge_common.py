"""
Shared PX4 bridge logic for velocity-mode offboard control.

Used by px4_bridge_defender_node and px4_bridge_attacker_node.
Provides: frame conversion, velocity-mode heartbeat/setpoint builders, readiness state.
"""

from enum import Enum
from typing import Tuple

# Import px4_msgs lazily so this module can be tested without ROS
try:
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
except ImportError:
    OffboardControlMode = None
    TrajectorySetpoint = None
    VehicleCommand = None


def game_to_ned(vx: float, vy: float, vz: float) -> Tuple[float, float, float]:
    """Convert game frame (z-up) velocity to PX4 NED (z-down)."""
    return (vx, vy, -vz)


def build_velocity_heartbeat(timestamp_ns: int) -> "OffboardControlMode":
    """Build OffboardControlMode for velocity-only control."""
    msg = OffboardControlMode()
    msg.position = False
    msg.velocity = True
    msg.acceleration = False
    msg.attitude = False
    msg.body_rate = False
    msg.timestamp = int(timestamp_ns / 1000)
    return msg


def build_velocity_setpoint(
    vx_ned: float, vy_ned: float, vz_ned: float, timestamp_ns: int
) -> "TrajectorySetpoint":
    """Build TrajectorySetpoint for velocity-only control. Position = NaN."""
    msg = TrajectorySetpoint()
    msg.position = [float("nan"), float("nan"), float("nan")]
    msg.velocity = [float(vx_ned), float(vy_ned), float(vz_ned)]
    msg.acceleration = [float("nan"), float("nan"), float("nan")]
    msg.yaw = float("nan")
    msg.yawspeed = 0.0
    msg.timestamp = int(timestamp_ns / 1000)
    return msg


def build_vehicle_command(
    command: int,
    target_system: int,
    timestamp_ns: int,
    param1: float = 0.0,
    param2: float = 0.0,
    **params,
) -> "VehicleCommand":
    """Build VehicleCommand."""
    msg = VehicleCommand()
    msg.command = command
    msg.param1 = param1
    msg.param2 = param2
    msg.param3 = params.get("param3", 0.0)
    msg.param4 = params.get("param4", 0.0)
    msg.param5 = params.get("param5", 0.0)
    msg.param6 = params.get("param6", 0.0)
    msg.param7 = params.get("param7", 0.0)
    msg.target_system = target_system
    msg.target_component = 1
    msg.source_system = 1
    msg.source_component = 1
    msg.from_external = True
    msg.timestamp = int(timestamp_ns / 1000)
    return msg


def is_position_valid(pos_msg) -> bool:
    """Check if VehicleLocalPosition has valid position."""
    if pos_msg is None:
        return False
    return bool(pos_msg.xy_valid and pos_msg.z_valid)


class BridgeReadinessState(Enum):
    """Readiness states for PX4 bridge. Only ACTIVE should command real motion."""

    WAITING_POSITION = 1  # No valid local position yet
    STREAMING = 2  # Publishing heartbeat + setpoints, waiting to arm
    ARMED_OFFBOARD = 3  # Armed and in offboard, not yet commanding
    ACTIVE = 4  # Ready: arm+offboard done, commanding motion allowed
    GAME_OVER = 5  # Game over: only zero velocity
    NOT_READY = 6  # Generic not ready (e.g. no position after timeout)
