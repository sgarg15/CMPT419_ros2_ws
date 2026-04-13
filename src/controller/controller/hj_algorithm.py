import os
from typing import Any, Dict, Tuple

import casadi as ca
import numpy as np
from ament_index_python.packages import get_package_share_directory
from numpy.typing import NDArray
from odp.dynamics import DubinsCar2
from odp.Grid import Grid

from controller.controller_base import ControllerBackend
from nav_helpers.trajectory import StateActionTrajectory

# ============================================================
# Constants / Defaults
# ============================================================

DEFAULT_PARAMS: Dict[str, Any] = {
    # Nominal MPC parameters
    "N": 80,
    "dt": 0.1,
    "v_min": 0.0,
    "v_max": 1.0,
    "omega_max": 1.2,
    "w_pos": 2.0,
    "w_theta": 0.1,
    "w_u": 0.05,
    "w_du": 0.2,
    "w_pos_T": 50.0,
    "w_theta_T": 0.2,
    "solver_opts": {
        "print_level": 0,
        "max_iter": 300,
        "tol": 1e-6,
        "constr_viol_tol": 1e-4,
    },
    # HJ filter parameters
    "epsilon": 0.1,
    # Simulation
    "goal_tolerance": 0.20,
    "sim_steps": 150,
}

DEFAULT_CORRIDOR: Dict[str, Any] = {
    "x_knots": [0.0, 2.0, 4.0, 6.0, 8.0],
    "y_low_knots": [-1.0, -1.4, -1.6, -1.6, -1.4],
    "y_high_knots": [1.0, 1.4, 1.6, 1.6, 1.4],
}

DEFAULT_X0 = np.array([0.5, 0.0, 0.0], dtype=float)
DEFAULT_GOAL = np.array([7.5, 0.0, 0.0], dtype=float)

DEFAULT_OBSTACLE: Dict[str, Any] = {
    "center": np.array([4.3, 0.15], dtype=float),
    "radius": 0.50,
    "safety_margin": 0.20,
}


# ============================================================
# Helper functions
# ============================================================


def dubins_step_numpy(x: NDArray, u: NDArray, dt: float) -> NDArray:
    """
    Forward Euler discretization:
      x = [px, py, theta], u = [v, omega]
    """
    x = np.asarray(x, dtype=float).reshape(3)
    u = np.asarray(u, dtype=float).reshape(2)

    px, py, th = float(x[0]), float(x[1]), float(x[2])
    v, om = float(u[0]), float(u[1])

    return np.array(
        [
            px + v * np.cos(th) * dt,
            py + v * np.sin(th) * dt,
            th + om * dt,
        ],
        dtype=float,
    )


def dubins_step(xk: ca.MX, uk: ca.MX, dt: float) -> ca.MX:
    """
    Symbolic forward Euler discretization:
      x = [px, py, theta], u = [v, omega]
    """
    px_next = xk[0] + uk[0] * ca.cos(xk[2]) * dt
    py_next = xk[1] + uk[0] * ca.sin(xk[2]) * dt
    th_next = xk[2] + uk[1] * dt
    return ca.vertcat(px_next, py_next, th_next)


def pwl_clamped(x, x_knots, y_knots):
    xk = list(map(float, x_knots))
    yk = list(map(float, y_knots))
    assert len(xk) == len(yk) and len(xk) >= 2, "invalid knot arrays"

    y = yk[-1]
    y = ca.if_else(x <= xk[0], yk[0], y)

    for i in range(len(xk) - 1):
        x0, x1 = xk[i], xk[i + 1]
        y0, y1 = yk[i], yk[i + 1]
        seg = ca.logic_and(x >= x0, x < x1)
        alpha = (x - x0) / (x1 - x0)
        yi = y0 + alpha * (y1 - y0)
        y = ca.if_else(seg, yi, y)

    y = ca.if_else(x >= xk[-1], yk[-1], y)
    return y


def y_low(px, corridor_params: Dict[str, Any]):
    return pwl_clamped(px, corridor_params["x_knots"], corridor_params["y_low_knots"])


def y_high(px, corridor_params: Dict[str, Any]):
    return pwl_clamped(px, corridor_params["x_knots"], corridor_params["y_high_knots"])


# ============================================================
# Nominal MPC
# ============================================================


def solve_mpc(
    x0: NDArray,
    goal: NDArray,
    corridor_params: Dict[str, Any],
    params: Dict[str, Any],
) -> Tuple[NDArray, NDArray]:
    """
    solve_mpc(x0, goal, corridor_params, params) -> (X, U)
    """
    N = int(params["N"])
    dt = float(params["dt"])

    v_min = float(params["v_min"])
    v_max = float(params["v_max"])
    omega_max = float(params["omega_max"])

    w_pos = float(params["w_pos"])
    w_theta = float(params["w_theta"])
    w_u = float(params["w_u"])
    w_du = float(params["w_du"])
    w_pos_T = float(params["w_pos_T"])
    w_theta_T = float(params["w_theta_T"])

    x0 = np.asarray(x0, dtype=float).reshape(3)
    goal = np.asarray(goal, dtype=float).reshape(3)

    gx, gy, gth = float(goal[0]), float(goal[1]), float(goal[2])

    x_min = float(corridor_params["x_knots"][0])
    x_max = float(corridor_params["x_knots"][-1])

    x0_dm = ca.DM(x0)

    opti = ca.Opti()
    X = opti.variable(3, N + 1)
    U = opti.variable(2, N)

    J = ca.MX(0)

    # Warm start
    v_guess = (gx - float(x0[0])) / max(N * dt, 1e-9)
    v_guess = float(np.clip(v_guess, v_min, v_max))
    u_guess = np.array([v_guess, 0.0], dtype=float)

    X_init = np.zeros((3, N + 1), dtype=float)
    X_init[:, 0] = x0
    for k in range(N):
        X_init[:, k + 1] = dubins_step_numpy(X_init[:, k], u_guess, dt)
        X_init[0, k + 1] = float(np.clip(X_init[0, k + 1], x_min, x_max))

    U_init = np.zeros((2, N), dtype=float)
    U_init[0, :] = v_guess
    U_init[1, :] = 0.0

    opti.set_initial(X, X_init)
    opti.set_initial(U, U_init)

    # ============================================================
    # Nominal MPC constraints + objective
    # ============================================================

    opti.subject_to(X[:, 0] == x0_dm)

    for k in range(N):
        xk = X[:, k]
        uk = U[:, k]

        opti.subject_to(X[:, k + 1] == dubins_step(xk, uk, dt))
        opti.subject_to(opti.bounded(v_min, uk[0], v_max))
        opti.subject_to(opti.bounded(-omega_max, uk[1], omega_max))

        ep = ca.vertcat(xk[0] - gx, xk[1] - gy)
        eth = xk[2] - gth
        J += w_pos * ca.sumsqr(ep) + w_theta * (eth ** 2) + w_u * ca.sumsqr(uk)

    for k in range(N + 1):
        px = X[0, k]
        py = X[1, k]
        opti.subject_to(opti.bounded(x_min, px, x_max))
        opti.subject_to(py >= y_low(px, corridor_params))
        opti.subject_to(py <= y_high(px, corridor_params))

    xN = X[:, N]
    epN = ca.vertcat(xN[0] - gx, xN[1] - gy)
    ethN = xN[2] - gth
    J += w_pos_T * ca.sumsqr(epN) + w_theta_T * (ethN ** 2)

    for k in range(1, N):
        du = U[:, k] - U[:, k - 1]
        J += w_du * ca.sumsqr(du)

    opti.minimize(J)

    p_opts = {"expand": True, "print_time": False}
    s_opts = dict(params["solver_opts"])
    s_opts.setdefault("max_iter", 2000)
    s_opts.setdefault("acceptable_tol", 1e-3)
    s_opts.setdefault("acceptable_constr_viol_tol", 1e-4)
    s_opts.setdefault("acceptable_iter", 10)
    s_opts.setdefault("mu_strategy", "adaptive")

    opti.solver("ipopt", p_opts, s_opts)

    try:
        sol = opti.solve()
        X_out = sol.value(X).T
        U_out = sol.value(U).T
    except RuntimeError:
        print("Warning: nominal MPC solve failed. Returning zeros.")
        X_out = np.full((N + 1, 3), np.nan)
        U_out = np.zeros((N, 2), dtype=float)

    return X_out, U_out


# ============================================================
# STUDENT FUNCTIONS
# ============================================================


def get_nominal_control(
    x: NDArray,
    goal: NDArray,
    corridor_params: Dict[str, Any],
    params: Dict[str, Any],
) -> NDArray:
    """
    Compute the nominal control using the provided MPC controller.
    """
    u_nom = np.zeros(2, dtype=float)

    _, U_nom = solve_mpc(
        x0=np.asarray(x, dtype=float).reshape(3),
        goal=np.asarray(goal, dtype=float).reshape(3),
        corridor_params=corridor_params,
        params=params,
    )

    if U_nom.shape[0] == 0 or np.any(np.isnan(U_nom[0])):
        u_nom = np.array([0.0, 0.0], dtype=float)
    else:
        u_nom = np.asarray(U_nom[0], dtype=float).reshape(2)

    return u_nom


def least_restrictive_safety_filter(
    x: NDArray,
    u_nom: NDArray,
    g: Grid,
    V: NDArray,
    derivatives: NDArray,
    epsilon: float,
    system_object,
) -> NDArray:
    """ """
    u_safe = np.zeros(2, dtype=float)  # dummy

    # ============================================================
    # STUDENT TODO:
    # first check the current value using the grid's member function get_values()
    # if value is above epsilon, just return nominal control
    # Else, use system_object.optCtrl_inPython to return the optimal control
    # ============================================================
    # STUDENT CODE START
    state = np.asarray(x, dtype=float).reshape(3)

    # Query current value function at x
    v_current = float(g.get_values(V, state))

    if v_current > epsilon:
        # State is safely inside the safe set – nominal control is fine
        u_safe = u_nom.copy()
    else:
        # Close to or inside the unsafe BRT – apply optimal safety control
        # Interpolate spatial derivatives at the current state
        spat_deriv = np.array([
            float(g.get_values(derivatives[0], state)),
            float(g.get_values(derivatives[1], state)),
            float(g.get_values(derivatives[2], state)),
        ], dtype=float)
        u_safe = np.asarray(
            system_object.optCtrl_inPython(state, spat_deriv), dtype=float
        ).reshape(2)
    # STUDENT CODE END
    return u_safe


class HJController(ControllerBackend):
    """Thin wrapper used by ROS2 frontend."""

    def __init__(self, config):
        cfg = dict(config.get("hj", {}))

        u_min = np.array(
            [float(cfg.get("v_min", -0.2)), float(cfg.get("w_min", -1.2))], dtype=float
        )
        u_max = np.array(
            [float(cfg.get("v_max", 1.0)), float(cfg.get("w_max", 1.2))], dtype=float
        )

        self._u_min = u_min
        self._u_max = u_max
        self._params = cfg
        package_share_path = get_package_share_directory("controller")

        # ============================================================
        # STUDENT TODO:
        # Load the HJ reachability result from previous question
        # Pass it to the safety filter function you implemented above
        # Stack all the derivatives information together in axis=0
        # and simulate the closed-loop system.
        # ============================================================

        # STUDENT CODE START
        import math
        data_path = os.path.join(package_share_path, "data", "hj_value_function.npz")
        data = np.load(data_path)

        self._V = data["V"]
        self._dV = np.stack(
            [data["dV_dx1"], data["dV_dx2"], data["dV_dx3"]], axis=0
        )

        # Reconstruct the grid
        self._g = Grid(
            minBounds=data["grid_min"].tolist(),
            maxBounds=data["grid_max"].tolist(),
            dims=3,
            pts_each_dim=data["grid_pts"].tolist(),
            periodicDims=[2],
        )

        # Rebuild the Dubins car object with the same bounds used during HJ computation
        v_min = float(cfg.get("v_min", 0.0))
        v_max = float(cfg.get("v_max", 1.0))
        w_max = float(cfg.get("w_max", 1.2))
        self._car = DubinsCar2(
            x=[0, 0, 0],
            uMin=[v_min, -w_max],
            uMax=[v_max,  w_max],
            uMode="max",
            dMode="min",
        )
        # STUDENT CODE END

    def get_action(
        self, observation: NDArray, traj: StateActionTrajectory = None
    ) -> NDArray:
        obs = np.asarray(observation, dtype=float).reshape(3)
        action = np.zeros((2,), dtype=float)
        epsilon = float(self._params["epsilon"])

        # ============================================================
        # STUDENT TODO:
        # Use least_restrictive_safety_filter() to compute the safe action
        # ============================================================

        # STUDENT CODE START
        action = least_restrictive_safety_filter(
            x=obs,
            u_nom=np.zeros(2, dtype=float),  # no nominal from traj here; use zero
            g=self._g,
            V=self._V,
            derivatives=self._dV,
            epsilon=epsilon,
            system_object=self._car,
        )
        # STUDENT CODE END

        return np.clip(action, self._u_min, self._u_max), None, None


# ============================================================
# Closed-loop simulation
# ============================================================


def simulate_least_restrictive_hj_filtered_controller(
    x0: NDArray,
    goal: NDArray,
    corridor_params: Dict[str, Any],
    obstacle: Dict[str, Any],
    params: Dict[str, Any],
) -> Tuple[NDArray, NDArray, NDArray]:
    """
    Closed-loop simulation:
      1) compute nominal MPC action
      2) filter it through the HJ-least restrictive switching filter
      3) simulate one step
    """
    dt = float(params["dt"])
    sim_steps = int(params["sim_steps"])
    goal_tolerance = float(params["goal_tolerance"])
    epsilon = float(params["epsilon"])

    x = np.asarray(x0, dtype=float).reshape(3).copy()

    X_hist = [x.copy()]
    U_nom_hist = []
    U_safe_hist = []

    reached_goal = False
    package_share_path = get_package_share_directory("controller")

    # ============================================================
    # STUDENT TODO:
    # Load the HJ reachability result from previous question
    # Pass it to the safety filter function you implemented above
    # Stack all the derivatives information together in axis=0
    # and simulate the closed-loop system.
    # ============================================================
    # STUDENT CODE START
    import math

    data_path = os.path.join(package_share_path, "data", "hj_value_function.npz")
    data = np.load(data_path)

    V = data["V"]
    derivatives = np.stack(
        [data["dV_dx1"], data["dV_dx2"], data["dV_dx3"]], axis=0
    )

    g = Grid(
        minBounds=data["grid_min"].tolist(),
        maxBounds=data["grid_max"].tolist(),
        dims=3,
        pts_each_dim=data["grid_pts"].tolist(),
        periodicDims=[2],
    )

    v_min = float(params.get("v_min", 0.0))
    v_max = float(params.get("v_max", 1.0))
    omega_max = float(params.get("omega_max", 1.2))
    my_car = DubinsCar2(
        x=[0, 0, 0],
        uMin=[v_min, -omega_max],
        uMax=[v_max,  omega_max],
        uMode="max",
        dMode="min",
    )
    # STUDENT CODE END

    for _ in range(sim_steps):
        u_nom = get_nominal_control(x, goal, corridor_params, params)
        u_safe = least_restrictive_safety_filter(
            x, u_nom, g, V, derivatives, epsilon, my_car
        )

        x = dubins_step_numpy(x, u_safe, dt)

        X_hist.append(x.copy())
        U_nom_hist.append(u_nom.copy())
        U_safe_hist.append(u_safe.copy())

        if np.linalg.norm(x[:2] - np.asarray(goal[:2], dtype=float)) <= goal_tolerance:
            reached_goal = True
            break

    if reached_goal:
        print("Reached the goal.")
    else:
        print("Did not reach the goal within the simulation horizon.")

    if len(U_nom_hist) == 0:
        U_nom_arr = np.zeros((0, 2), dtype=float)
        U_safe_arr = np.zeros((0, 2), dtype=float)
    else:
        U_nom_arr = np.asarray(U_nom_hist, dtype=float)
        U_safe_arr = np.asarray(U_safe_hist, dtype=float)

    return np.asarray(X_hist, dtype=float), U_nom_arr, U_safe_arr


# ============================================================
# Plotting
# ============================================================


def plot_trajectory(
    X: NDArray,
    corridor_params: Dict[str, Any],
    goal: NDArray,
    obstacle: Dict[str, Any],
):
    import matplotlib.pyplot as plt

    if np.any(np.isnan(X)):
        print("No valid trajectory to plot.")
        return

    px = X[:, 0]
    py = X[:, 1]

    xk = corridor_params["x_knots"]
    yl = corridor_params["y_low_knots"]
    yh = corridor_params["y_high_knots"]

    plt.figure(figsize=(10, 5.2))
    plt.plot(xk, yl, "b--", linewidth=2, label="Lower boundary")
    plt.plot(xk, yh, "b--", linewidth=2, label="Upper boundary")
    plt.plot(px, py, "r.-", linewidth=2, markersize=6, label="Filtered trajectory")

    cx, cy = obstacle["center"]
    r_obs = float(obstacle["radius"])
    r_safe = float(obstacle["radius"]) + float(obstacle["safety_margin"])

    obstacle_circle = plt.Circle(
        (cx, cy), r_obs, color="gray", alpha=0.6, label="Obstacle"
    )
    safe_circle = plt.Circle(
        (cx, cy),
        r_safe,
        fill=False,
        linestyle="--",
        linewidth=3,
        color="k",
        label="Safety margin (for robot center)",
    )

    plt.gca().add_patch(obstacle_circle)
    plt.gca().add_patch(safe_circle)

    plt.plot(X[0, 0], X[0, 1], "go", markersize=12, label="Start")
    plt.plot(goal[0], goal[1], "g*", markersize=18, label="Goal")

    plt.xlabel("px")
    plt.ylabel("py")
    plt.title("HJ-Filtered Dubins Car in Corridor")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig("hj_filtered_corridor.png", dpi=200)
    plt.show()


if __name__ == "__main__":
    X, U_nom, U_safe = simulate_least_restrictive_hj_filtered_controller(
        x0=DEFAULT_X0,
        goal=DEFAULT_GOAL,
        corridor_params=DEFAULT_CORRIDOR,
        obstacle=DEFAULT_OBSTACLE,
        params=DEFAULT_PARAMS,
    )
    plot_trajectory(X, DEFAULT_CORRIDOR, DEFAULT_GOAL, DEFAULT_OBSTACLE)
