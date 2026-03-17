"""
Simple control policies for the reach-avoid game.

Extracted from test_dynamics for use by game_controller_node.
Pure Python, no ROS dependencies. Policies match the API used by dynamics.simulate().

Defender policy:  (DefenderState, AttackerState, GameParams, t) -> np.ndarray[3]
Attacker policy:  (AttackerState, DefenderState, GameParams, t) -> np.ndarray[3]
"""

import numpy as np

from ..dynamics import GameParams, DefenderState, AttackerState


def pure_pursuit_policy(defender: DefenderState, attacker: AttackerState,
                       params: GameParams, t: float) -> np.ndarray:
    """
    Naive pure pursuit: command velocity toward attacker position.

    Args:
        defender: current defender state
        attacker: current attacker state
        params: game parameters
        t: current time (unused)

    Returns:
        Velocity command [vx_cmd, vy_cmd, vz_cmd] in world frame (z-up)
    """
    diff = attacker.position() - defender.position()
    dist = np.linalg.norm(diff[:2])
    if dist < 1e-6:
        u_h = np.array([0.0, 0.0])
    else:
        direction = diff[:2] / dist
        u_h = direction * params.U_h_D

    dz = diff[2]
    u_z = np.sign(dz) * params.U_z_D if abs(dz) > 0.1 else 0.0

    return np.array([u_h[0], u_h[1], u_z])


def straight_line_attacker(goal_direction: np.ndarray,
                          speed_fraction: float = 1.0):
    """
    Factory: returns an attacker policy that moves in a fixed direction.

    Args:
        goal_direction: 3D unit direction (will be normalized)
        speed_fraction: fraction of max speed (0..1)

    Returns:
        Callable policy(attacker, defender, params, t) -> np.ndarray[3]
    """
    goal_dir = goal_direction / np.linalg.norm(goal_direction)

    def policy(attacker: AttackerState, defender: DefenderState,
               params: GameParams, t: float) -> np.ndarray:
        h_speed = params.U_h_A * speed_fraction
        z_speed = params.U_z_A * speed_fraction
        return np.array([
            goal_dir[0] * h_speed,
            goal_dir[1] * h_speed,
            goal_dir[2] * z_speed,
        ])

    return policy


def evasive_attacker_policy(attacker: AttackerState, defender: DefenderState,
                           params: GameParams, t: float) -> np.ndarray:
    """
    Attacker moves away from defender at max speed (adversarial).

    Args:
        attacker: current attacker state
        defender: current defender state
        params: game parameters
        t: current time (unused)

    Returns:
        Velocity command [vx_cmd, vy_cmd, vz_cmd]
    """
    diff = attacker.position() - defender.position()
    h_dist = np.linalg.norm(diff[:2])
    if h_dist < 1e-6:
        u_h = np.array([params.U_h_A, 0.0])
    else:
        direction = diff[:2] / h_dist
        u_h = direction * params.U_h_A

    dz = diff[2]
    u_z = np.sign(dz) * params.U_z_A if abs(dz) > 0.01 else params.U_z_A

    return np.array([u_h[0], u_h[1], u_z])


# Default attacker policy for runtime: straight line toward -x (e.g. toward a target)
DEFAULT_ATTACKER_GOAL = np.array([-1.0, 0.0, 0.0])
DEFAULT_ATTACKER_SPEED_FRACTION = 1.0


def get_default_attacker_policy():
    """Return the default straight-line attacker policy used in simulation."""
    return straight_line_attacker(DEFAULT_ATTACKER_GOAL, DEFAULT_ATTACKER_SPEED_FRACTION)


if __name__ == "__main__":
    # Quick sanity test: run a short simulation
    from ros2d2.dynamics import simulate, GameParams, DefenderState, AttackerState

    params = GameParams()
    defender_init = DefenderState(x=30.0, y=15.0, z=5.0, vx=0.0, vy=0.0, vz=0.0)
    attacker_init = AttackerState(x=15.0, y=10.0, z=5.0)

    result = simulate(
        defender_init, attacker_init,
        pure_pursuit_policy, get_default_attacker_policy(),
        params, dt=0.01, T=2.0,
    )

    print("simple_policies sanity test OK")
    print(f"  Defender final: {result.defender_states[-1].position()}")
    print(f"  Attacker final: {result.attacker_states[-1].position()}")
