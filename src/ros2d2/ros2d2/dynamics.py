"""
Reach-avoid game dynamics for UAV defender vs attacker.

Based on: "Reach-Avoid Differential Game with Reachability Analysis for UAVs:
A decomposition approach" (Bui, Monckton, Chen, 2025)

Defender: double integrator (6D state: position + velocity in x,y,z)
Attacker: single integrator (3D state: position in x,y,z)

The 9D joint system is decomposed into:
  - Horizontal sub-game (6D): defender (x, y, vx, vy) + attacker (x, y)
  - Vertical sub-game (3D): defender (z, vz) + attacker (z)
"""

import numpy as np
from dataclasses import dataclass, field


@dataclass
class GameParams:
    """Parameters for the reach-avoid game (Table from Section VII.A)."""

    # Proportional coefficients for defender acceleration (Eq. 16)
    kx: float = 0.7
    ky: float = 0.7
    kz: float = 1.5

    # Maximum velocity commands
    U_h_D: float = 6.0   # defender max horizontal speed (m/s)
    U_h_A: float = 3.0   # attacker max horizontal speed (m/s)
    U_z_D: float = 4.0   # defender max vertical speed (m/s)
    U_z_A: float = 2.0   # attacker max vertical speed (m/s)

    # Capture radii (Section VII.A)
    d_h: float = 3.0     # horizontal capture radius (m)
    d_z: float = 1.0     # vertical capture radius (m)


@dataclass
class DefenderState:
    """Defender state: double integrator in 3D (Eq. 16 left)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def velocity(self) -> np.ndarray:
        return np.array([self.vx, self.vy, self.vz])

    def horizontal_state(self) -> np.ndarray:
        """x_D^h = (x, y, vx, vy) for horizontal sub-game (Eq. 19)."""
        return np.array([self.x, self.y, self.vx, self.vy])

    def vertical_state(self) -> np.ndarray:
        """x_D^z = (z, vz) for vertical sub-game (Eq. 23)."""
        return np.array([self.z, self.vz])

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z, self.vx, self.vy, self.vz])

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "DefenderState":
        return cls(x=arr[0], y=arr[1], z=arr[2], vx=arr[3], vy=arr[4], vz=arr[5])


@dataclass
class AttackerState:
    """Attacker state: single integrator in 3D (Eq. 16 right)."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    def horizontal_state(self) -> np.ndarray:
        """x_A^h = (x, y) for horizontal sub-game (Eq. 19)."""
        return np.array([self.x, self.y])

    def vertical_state(self) -> np.ndarray:
        """x_A^z = (z,) for vertical sub-game (Eq. 23)."""
        return np.array([self.z])

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "AttackerState":
        return cls(x=arr[0], y=arr[1], z=arr[2])


def clip_horizontal_control(vx_cmd: float, vy_cmd: float, max_speed: float) -> tuple[float, float]:
    """Clip 2D velocity command to satisfy ||u_h|| <= max_speed (Eq. 17a,b)."""
    speed = np.sqrt(vx_cmd**2 + vy_cmd**2)
    if speed > max_speed:
        scale = max_speed / speed
        return vx_cmd * scale, vy_cmd * scale
    return vx_cmd, vy_cmd


def clip_vertical_control(vz_cmd: float, max_speed: float) -> float:
    """Clip vertical velocity command to |vz_cmd| <= max_speed (Eq. 17c,d)."""
    return np.clip(vz_cmd, -max_speed, max_speed)


def defender_dynamics(state: DefenderState, u_D: np.ndarray, params: GameParams) -> np.ndarray:
    """
    Compute defender state derivative (Eq. 16, left).

    ẋ_D = v_x_D
    ẏ_D = v_y_D
    ż_D = v_z_D
    v̇_x_D = k_x * (v_x_cmd - v_x_D)
    v̇_y_D = k_y * (v_y_cmd - v_y_D)
    v̇_z_D = k_z * (v_z_cmd - v_z_D)

    Args:
        state: current defender state
        u_D: velocity command [vx_cmd, vy_cmd, vz_cmd]
        params: game parameters

    Returns:
        state derivative [ẋ, ẏ, ż, v̇x, v̇y, v̇z]
    """
    vx_cmd, vy_cmd = clip_horizontal_control(u_D[0], u_D[1], params.U_h_D)
    vz_cmd = clip_vertical_control(u_D[2], params.U_z_D)

    dx = state.vx
    dy = state.vy
    dz = state.vz
    dvx = params.kx * (vx_cmd - state.vx)
    dvy = params.ky * (vy_cmd - state.vy)
    dvz = params.kz * (vz_cmd - state.vz)

    return np.array([dx, dy, dz, dvx, dvy, dvz])


def attacker_dynamics(state: AttackerState, u_A: np.ndarray, params: GameParams) -> np.ndarray:
    """
    Compute attacker state derivative (Eq. 16, right).

    ẋ_A = v_x_cmd_A
    ẏ_A = v_y_cmd_A
    ż_A = v_z_cmd_A

    Args:
        state: current attacker state
        u_A: velocity command [vx_cmd, vy_cmd, vz_cmd]
        params: game parameters

    Returns:
        state derivative [ẋ, ẏ, ż]
    """
    vx_cmd, vy_cmd = clip_horizontal_control(u_A[0], u_A[1], params.U_h_A)
    vz_cmd = clip_vertical_control(u_A[2], params.U_z_A)

    return np.array([vx_cmd, vy_cmd, vz_cmd])


# --- Horizontal sub-game dynamics (Eq. 19) ---

def defender_horizontal_dynamics(
    x_h_D: np.ndarray, u_h_D: np.ndarray, params: GameParams
) -> np.ndarray:
    """
    Horizontal defender dynamics (Eq. 19, left).

    State: [x_D, y_D, vx_D, vy_D]
    Control: [vx_cmd, vy_cmd]

    Returns: [ẋ_D, ẏ_D, v̇x_D, v̇y_D]
    """
    vx_cmd, vy_cmd = clip_horizontal_control(u_h_D[0], u_h_D[1], params.U_h_D)
    vx_D, vy_D = x_h_D[2], x_h_D[3]

    return np.array([
        vx_D,
        vy_D,
        params.kx * (vx_cmd - vx_D),
        params.ky * (vy_cmd - vy_D),
    ])


def attacker_horizontal_dynamics(
    x_h_A: np.ndarray, u_h_A: np.ndarray, params: GameParams
) -> np.ndarray:
    """
    Horizontal attacker dynamics (Eq. 19, right).

    State: [x_A, y_A]
    Control: [vx_cmd, vy_cmd]

    Returns: [ẋ_A, ẏ_A]
    """
    vx_cmd, vy_cmd = clip_horizontal_control(u_h_A[0], u_h_A[1], params.U_h_A)
    return np.array([vx_cmd, vy_cmd])


def joint_horizontal_dynamics(
    x_h: np.ndarray, u_h_D: np.ndarray, u_h_A: np.ndarray, params: GameParams
) -> np.ndarray:
    """
    Joint horizontal dynamics f_h (Eq. 19 combined).

    Joint state: [x_D, y_D, vx_D, vy_D, x_A, y_A]
    Returns: [ẋ_D, ẏ_D, v̇x_D, v̇y_D, ẋ_A, ẏ_A]
    """
    d_dot = defender_horizontal_dynamics(x_h[:4], u_h_D, params)
    a_dot = attacker_horizontal_dynamics(x_h[4:6], u_h_A, params)
    return np.concatenate([d_dot, a_dot])


# --- Vertical sub-game dynamics (Eq. 23) ---

def defender_vertical_dynamics(
    x_z_D: np.ndarray, u_z_D: float, params: GameParams
) -> np.ndarray:
    """
    Vertical defender dynamics (Eq. 23, left).

    State: [z_D, vz_D]
    Control: vz_cmd (scalar)

    Returns: [ż_D, v̇z_D]
    """
    vz_cmd = clip_vertical_control(u_z_D, params.U_z_D)
    vz_D = x_z_D[1]

    return np.array([
        vz_D,
        params.kz * (vz_cmd - vz_D),
    ])


def attacker_vertical_dynamics(
    x_z_A: np.ndarray, u_z_A: float, params: GameParams
) -> np.ndarray:
    """
    Vertical attacker dynamics (Eq. 23, right).

    State: [z_A]
    Control: vz_cmd (scalar)

    Returns: [ż_A]
    """
    vz_cmd = clip_vertical_control(u_z_A, params.U_z_A)
    return np.array([vz_cmd])


def joint_vertical_dynamics(
    x_z: np.ndarray, u_z_D: float, u_z_A: float, params: GameParams
) -> np.ndarray:
    """
    Joint vertical dynamics f_z (Eq. 23 combined).

    Joint state: [z_D, vz_D, z_A]
    Returns: [ż_D, v̇z_D, ż_A]
    """
    d_dot = defender_vertical_dynamics(x_z[:2], u_z_D, params)
    a_dot = attacker_vertical_dynamics(x_z[2:3], u_z_A, params)
    return np.concatenate([d_dot, a_dot])


# --- Relative dynamics (Eq. 29, used for invariant sets) ---

def relative_vertical_dynamics(
    x_rel_z: np.ndarray, u_z_D: float, u_z_A: float, params: GameParams
) -> np.ndarray:
    """
    Relative vertical dynamics (Eq. 29).

    State: [z_rel, vz_D] where z_rel = z_D - z_A
    Control: u_z_D (defender vz_cmd), u_z_A (attacker vz_cmd)

    Returns: [ż_rel, v̇z_D]
    """
    vz_cmd_D = clip_vertical_control(u_z_D, params.U_z_D)
    vz_cmd_A = clip_vertical_control(u_z_A, params.U_z_A)
    vz_D = x_rel_z[1]

    return np.array([
        vz_D - vz_cmd_A,
        params.kz * (vz_cmd_D - vz_D),
    ])


# --- Capture conditions (Eq. 18) ---

def horizontal_distance(defender: DefenderState, attacker: AttackerState) -> float:
    """Horizontal distance: sqrt((x_A - x_D)^2 + (y_A - y_D)^2) (Eq. 18a LHS)."""
    return np.sqrt((attacker.x - defender.x)**2 + (attacker.y - defender.y)**2)


def vertical_distance(defender: DefenderState, attacker: AttackerState) -> float:
    """Vertical distance: |z_A - z_D| (Eq. 18b LHS)."""
    return abs(attacker.z - defender.z)


def is_captured_horizontal(defender: DefenderState, attacker: AttackerState, params: GameParams) -> bool:
    """Check horizontal capture condition (Eq. 18a)."""
    return horizontal_distance(defender, attacker) <= params.d_h


def is_captured_vertical(defender: DefenderState, attacker: AttackerState, params: GameParams) -> bool:
    """Check vertical capture condition (Eq. 18b)."""
    return vertical_distance(defender, attacker) <= params.d_z


def is_captured(defender: DefenderState, attacker: AttackerState, params: GameParams) -> bool:
    """Full 3D capture: both horizontal AND vertical conditions met (Eq. 18)."""
    return is_captured_horizontal(defender, attacker, params) and \
           is_captured_vertical(defender, attacker, params)


def euclidean_3d_distance(defender: DefenderState, attacker: AttackerState) -> float:
    """Standard 3D Euclidean distance between defender and attacker."""
    return np.linalg.norm(defender.position() - attacker.position())


# --- Simulation ---

@dataclass
class SimulationResult:
    """Container for simulation trajectory data."""

    time: np.ndarray = field(default_factory=lambda: np.array([]))
    defender_states: list[DefenderState] = field(default_factory=list)
    attacker_states: list[AttackerState] = field(default_factory=list)
    defender_controls: list[np.ndarray] = field(default_factory=list)
    attacker_controls: list[np.ndarray] = field(default_factory=list)

    def defender_positions(self) -> np.ndarray:
        return np.array([s.position() for s in self.defender_states])

    def attacker_positions(self) -> np.ndarray:
        return np.array([s.position() for s in self.attacker_states])

    def horizontal_distances(self) -> np.ndarray:
        return np.array([
            horizontal_distance(d, a)
            for d, a in zip(self.defender_states, self.attacker_states)
        ])

    def vertical_distances(self) -> np.ndarray:
        return np.array([
            vertical_distance(d, a)
            for d, a in zip(self.defender_states, self.attacker_states)
        ])

    def euclidean_distances(self) -> np.ndarray:
        return np.array([
            euclidean_3d_distance(d, a)
            for d, a in zip(self.defender_states, self.attacker_states)
        ])


def simulate(
    defender_init: DefenderState,
    attacker_init: AttackerState,
    defender_policy,
    attacker_policy,
    params: GameParams,
    dt: float = 0.01,
    T: float = 10.0,
) -> SimulationResult:
    """
    Forward Euler simulation of the full 9D joint system.

    Args:
        defender_init: initial defender state
        attacker_init: initial attacker state
        defender_policy: callable(DefenderState, AttackerState, GameParams, t) -> np.ndarray[3]
        attacker_policy: callable(AttackerState, DefenderState, GameParams, t) -> np.ndarray[3]
        params: game parameters
        dt: time step for Euler integration
        T: total simulation time

    Returns:
        SimulationResult with full trajectory data
    """
    n_steps = int(T / dt)
    times = np.linspace(0, T, n_steps + 1)

    result = SimulationResult(time=times)

    d_state = DefenderState(*defender_init.to_array())
    a_state = AttackerState(*attacker_init.to_array())

    for i, t in enumerate(times):
        result.defender_states.append(DefenderState(*d_state.to_array()))
        result.attacker_states.append(AttackerState(*a_state.to_array()))

        if i == n_steps:
            break

        u_D = defender_policy(d_state, a_state, params, t)
        u_A = attacker_policy(a_state, d_state, params, t)

        result.defender_controls.append(u_D)
        result.attacker_controls.append(u_A)

        d_dot = defender_dynamics(d_state, u_D, params)
        a_dot = attacker_dynamics(a_state, u_A, params)

        d_arr = d_state.to_array() + dt * d_dot
        a_arr = a_state.to_array() + dt * a_dot

        d_state = DefenderState.from_array(d_arr)
        a_state = AttackerState.from_array(a_arr)

    return result
