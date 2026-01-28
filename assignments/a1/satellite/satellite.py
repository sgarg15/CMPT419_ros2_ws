import numpy as np
import matplotlib.pyplot as plt
from typing import Optional, Callable, Tuple
from numpy.typing import NDArray

# =======================
# Constants: Do not modify!
# =======================
G = 6.67e-11
M = 5.97e24
K = G * M
T = 92.68 * 60
EARTH_RADIUS = 6378e3


def satellite_dynamics(t: float, x: NDArray) -> NDArray:
    """
    Dynamics of the satellite system, given as xdot = f(t, x, u)

    Args:
      t: Scalar representing time
      x: Array representing a single state

    Returns:
      dx: Time derivative f(t, x, u)
    """
    dx = np.zeros((4))

    # STUDENT CODE START
    r = x[0]
    rDot = x[1]
    theta = x[2]
    thetaDot = x[3]    

    dx[0] = rDot
    dx[1] = r * thetaDot**2 - K / r**2
    dx[2] = thetaDot
    dx[3] = -2 * rDot * thetaDot / r
    # STUDENT CODE END

    return dx


def RK4(
    dynamics: Callable, tspan: Tuple[float, float], x0: NDArray
) -> Tuple[NDArray, NDArray]:
    """
    RK4 implementation

    Args:
      dynamics: function handle representing dynamics
      tspan: 2-element array specifying initial and final time
      x0: initial condition

    Returns:
      t: 1D array representing the discretized time points
      x: 2D array representing states over time. Each row is the state at a time point
    """

    t = np.linspace(tspan[0], tspan[1], num=1001)
    x = np.zeros((np.size(t), np.size(x0)))

    # STUDENT CODE START
    h = t[1] - t[0]
    x[0, :] = x0

    for i in range(len(t) - 1):
        t_i = t[i]
        x_i = x[i, :]
        k1 = dynamics(t_i, x_i)
        k2 = dynamics(t_i + h/2, x_i + h/2 * k1)
        k3 = dynamics(t_i + h/2, x_i + h/2 * k2)
        k4 = dynamics(t_i + h, x_i + h * k3)
        
        x[i+1, :] = x_i + (h/6) * (k1 + 2*k2 + 2*k3 + k4)
    # STUDENT CODE END
    return t, x


# =======================
# Helper: Do not modify!
# =======================
def plot_results(
    t: NDArray,
    x: NDArray,
    show: bool = True,
    filename: Optional[str] = None,
):
    # Create figure and subplots
    fig = plt.figure(figsize=(24, 12))
    gs = fig.add_gridspec(2, 4)

    ax_path = fig.add_subplot(gs[:, 0:2])
    ax_r = fig.add_subplot(gs[0, 2])
    ax_rdot = fig.add_subplot(gs[0, 3])
    ax_theta = fig.add_subplot(gs[1, 2])
    ax_thetadot = fig.add_subplot(gs[1, 3])
    ax_state = [ax_r, ax_rdot, ax_theta, ax_thetadot]

    # Spatial plot
    theta = np.linspace(0, 2 * np.pi, 100)
    ax_path.plot(
        x[:, 0] * np.cos(x[:, 2]) / 1e3,
        x[:, 0] * np.sin(x[:, 2]) / 1e3,
        "-",
        label="Satellite trajectory",
    )
    ax_path.plot(
        EARTH_RADIUS * np.cos(theta) / 1e3,
        EARTH_RADIUS * np.sin(theta) / 1e3,
        label="Earth",
    )
    ax_path.set_title("Satellite position in space w.r.t. Earth centre")
    ax_path.legend()

    titles = [
        r"$r$ (km above Earth's surface)",
        r"$\dot r$ (m/s)",
        r"$\theta$ (rad)",
        r"$\dot \theta$ (rad/s)",
    ]

    # State plots
    x[:, 0] = (x[:, 0] - EARTH_RADIUS) / 1e3
    for i in range(4):
        ax_state[i].plot(t / 60, x[:, i], "-")
        ax_state[i].set_xlabel("t (minutes)")
        ax_state[i].set_title(titles[i])

    plt.tight_layout()

    if show:
        plt.show()

    if filename:
        fig.savefig(filename, dpi=300, bbox_inches="tight")

    plt.close(fig)


# ======================
# Driver: Do not modify!
# ======================
if __name__ == "__main__":
    tspan = np.array([0, 60 * 1000])
    x0 = np.array([410e3 + EARTH_RADIUS, 0, 0, 2 * np.pi / T])

    [t, x] = RK4(satellite_dynamics, tspan, x0)

    plot_results(t, x, show=True, filename="satellite.pdf")
