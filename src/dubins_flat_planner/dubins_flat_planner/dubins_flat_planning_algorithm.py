from typing import NamedTuple, Optional, Tuple

import numpy as np
from numpy.typing import NDArray
import matplotlib.pyplot as plt

# ==============================
# Helper classes: Do not modify!
# ==============================
class BoundaryState(NamedTuple):
    x: float
    y: float
    theta: float
    v: float


class StateTraj(NamedTuple):
    """State trajectory."""

    t: NDArray = np.linspace(0.0, 1.0, 10)
    x: NDArray = np.zeros(10)
    y: NDArray = np.zeros(10)
    theta: NDArray = np.zeros(10)
    v: NDArray = np.zeros(10)
    ##
    xdot: NDArray = np.zeros(10)
    ydot: NDArray = np.zeros(10)
    ##
    xddot: NDArray = np.zeros(10)
    yddot: NDArray = np.zeros(10)


class ControlTraj(NamedTuple):
    """Control trajectory."""

    t: NDArray = np.linspace(0.0, 1.0, 10)
    omega: NDArray = np.zeros(10)
    a: NDArray = np.zeros(10)


# =========================
# Implement these functions
# =========================
def get_constraint_matrix(T: float) -> NDArray:
    """Constraint matrix for cubic polynomial position and velocity at t=0 and t=T."""

    constraint_mat = np.zeros((4, 4))

    # STUDENT CODE START
    constraint_mat[0, 0] = 1.0      
    constraint_mat[1, 1] = 1.0  
    constraint_mat[2, 0] = 1.0  
    constraint_mat[2, 1] = T   
    constraint_mat[2, 2] = T**2 
    constraint_mat[2, 3] = T**3 
    constraint_mat[3, 1] = 1.0     
    constraint_mat[3, 2] = 2.0 * T 
    constraint_mat[3, 3] = 3.0 * T**2 
    # STUDENT CODE END

    return constraint_mat


def solve_cubic_coeffs_from_endpoints(
    p0: float, v0: float, pT: float, vT: float, T: float
) -> NDArray:
    r"""Solve cubic coefficients b=[b0,b1,b2,b3] for p(t). Returns coefficients b.

    cubic polynomial:
    \[
      p(t) = b0 + b1 \cdot t + b2 \cdot t^{2} + b3 \cdot t^{3}
    \]
    such that the polynomial satisfies all four boundary conditions:
    - p(0) = p0
    - p'(0) = v0
    - p(T) = pT
    - p'(T) = vT
    """
    assert T > 0, f"T must be positive. got {T}."

    M = np.eye(4)  # Replace these
    rhs = np.ones(4)  # Replace these

    # STUDENT CODE START
    M = get_constraint_matrix(T)
    rhs = np.array([p0, v0, pT, vT])
    # STUDENT CODE END

    return np.linalg.solve(M, rhs)  # (4,)


def eval_cubic_and_derivatives(
    coeff: NDArray, t: NDArray
) -> Tuple[NDArray, NDArray, NDArray]:
    """Evaluate cubic polynomial p(t) and its first two derivatives.

    coeff: shape (4,) for [b0, b1, b2, b3]
    """
    if coeff.shape != (4,):
        raise ValueError(f"Expected coeff.shape == (4,), got {coeff.shape}.")

    p = np.zeros_like(t)
    pdot = np.zeros_like(t)
    pddot = np.zeros_like(t)

    # STUDENT CODE START
    p = coeff[0] + coeff[1]*t + coeff[2]*t**2 + coeff[3]*t**3
    pdot = coeff[1] + 2*coeff[2]*t + 3*coeff[3]*t**2
    pddot = 2*coeff[2] + 6*coeff[3]*t
    # STUDENT CODE END

    return p, pdot, pddot


def plan_cubic_flat_trajectory(
    start: BoundaryState,
    goal: BoundaryState,
    T: float,
    dt: float = 0.01,
    eps: float = 1e-12,
) -> Tuple[StateTraj, ControlTraj]:
    if T <= 0:
        raise ValueError("T must be positive.")

    # HINT: study the function signature of
    #     `solve_cubic_coeffs_from_endpoints()` and `eval_cubic_and_derivatives()`.
    #      You should use first function to find cubic coefficients and evaluate
    #      for flat output representations of state and control at time t using
    #      the second function.

    state_traj = StateTraj()
    ctrl_traj = ControlTraj()

    # STUDENT CODE START
    t = get_time_grid(T, dt)
    
    vx0 = start.v * np.cos(start.theta)
    vxT = goal.v * np.cos(goal.theta)
    coeffs_x = solve_cubic_coeffs_from_endpoints(start.x, vx0, goal.x, vxT, T)
    x, xdot, xddot = eval_cubic_and_derivatives(coeffs_x, t)
    
    vy0 = start.v * np.sin(start.theta)
    vyT = goal.v * np.sin(goal.theta)
    coeffs_y = solve_cubic_coeffs_from_endpoints(start.y, vy0, goal.y, vyT, T)
    y, ydot, yddot = eval_cubic_and_derivatives(coeffs_y, t)
    
    v = np.sqrt(xdot**2 + ydot**2)
    theta = np.arctan2(ydot, xdot)
    
    v_squared = v**2
    v_squared = np.where(v_squared < eps, eps, v_squared)
    omega = (xdot * yddot - ydot * xddot) / v_squared
    a = (xdot * xddot + ydot * yddot) / np.where(v < eps, eps, v)
    
    state_traj = StateTraj(t=t, x=x, y=y, theta=theta, v=v, xdot=xdot, ydot=ydot, xddot=xddot, yddot=yddot)
    ctrl_traj = ControlTraj(t=t, omega=omega, a=a)
    # STUDENT CODE END

    return state_traj, ctrl_traj


# =======================
# Helpers: Do not modify!
# =======================
def get_time_grid(T: float, dt: float) -> NDArray:
    """Inclusive grid from 0 to T."""
    assert T > 0, f"T must be positive. got {T}."
    if dt <= 0:
        raise ValueError("dt must be positive.")

    n = int(np.floor(T / dt))
    t = dt * np.arange(n + 1, dtype=float)
    if t[-1] < T:
        t = np.append(t, T)
    return t


def plot_results(
    state: StateTraj,
    ctrl: ControlTraj,
    show: bool = True,
    filename: Optional[str] = None,
) -> None:
    

    "a post-hoc plot from the trajectory data."
    fig = plt.figure(figsize=(24, 12))
    gs = fig.add_gridspec(2, 4)

    # path in x-y space
    ax_path = fig.add_subplot(gs[:, 0:2])
    ax_path.plot(state.x, state.y, linewidth=3)
    ax_path.set_title("Path of Car")
    ax_path.set_xlabel(r"$x$")
    ax_path.set_ylabel(r"$y$")
    ax_path.grid(True)
    ax_path.set_xlim((-0.1, 1.5))
    ax_path.set_ylim((-1.5, 0.1))
    ax_path.set_aspect("equal", adjustable="box")

    # state trajectories
    ax_state = fig.add_subplot(gs[0, 2:4])
    ax_state.plot(state.t, state.x, linewidth=3, label=r"$x$")
    ax_state.plot(state.t, state.y, linewidth=3, label=r"$y$")
    ax_state.plot(state.t, state.theta, linewidth=3, label=r"$\theta$")
    ax_state.plot(state.t, state.v, linewidth=3, label=r"$v$")
    ax_state.set_title("State Trajectories")
    ax_state.grid(True)
    ax_state.legend()

    # control trajectories
    ax_ctrl = fig.add_subplot(gs[1, 2:4])
    ax_ctrl.plot(ctrl.t, ctrl.omega, linewidth=3, label=r"$\omega$")
    ax_ctrl.plot(ctrl.t, ctrl.a, linewidth=3, label=r"$a$")
    ax_ctrl.set_title("Control Trajectories")
    ax_ctrl.set_xlabel(r"$t$")
    ax_ctrl.grid(True)
    ax_ctrl.legend()

    plt.tight_layout()
    if show:
        plt.show()

    # save
    if filename:
        fig.savefig(filename, dpi=300, bbox_inches="tight")

    plt.close(fig)


# ======================
# Driver: Do not modify!
# ======================
def solve_and_plot() -> None:
    # use planned state/control trajectories on a time grid as in question q3-b
    start = BoundaryState(x=0.0, y=0.0, theta=0.0, v=1.0)
    goal = BoundaryState(x=0.0, y=0.0, theta=np.pi / 2.0, v=1.0)

    state_traj, ctrl_traj = plan_cubic_flat_trajectory(start, goal, T=10.0, dt=0.01)
    plot_results(state_traj, ctrl_traj, show=True, filename="dubins_flat_planner.pdf")


if __name__ == "__main__":
    solve_and_plot()
