import imp
import math
import os

import numpy as np

# Specify the  file that includes dynamic systems
from odp.dynamics import DubinsCar2

# Utility functions to initialize the problem
from odp.Grid import Grid

# Plot options
from odp.Plots import PlotOptions, visualize_plots
from odp.Shapes import *

# Solver core
from odp.solver import HJSolver, computeSpatDerivArray

# STUDENT CODE START

# ── Grid ──────────────────────────────────────────────────────────────────────
# State: [px, py, theta]
# px   : corridor spans [0, 8]; add margin → [-0.5, 8.5]
# py   : corridor spans ≈ [-1.6, 1.6]; add margin → [-2.5, 2.5]
# theta: periodic in [-π, π]
g = Grid(
    minBounds=np.array([-0.5, -2.5, -math.pi]),
    maxBounds=np.array([8.5,  2.5,   math.pi]),
    dims=3,
    pts_each_dim=np.array([100, 60, 36]),
    periodicDims=[2],
)

# ── Dynamics ──────────────────────────────────────────────────────────────────
# uMode="max": the robot (our controller) maximises V to stay OUTSIDE the BRT
# (i.e., avoid the obstacle). optCtrl_inPython will then return the control
# that maximises V at the boundary, keeping the robot safe.
my_car = DubinsCar2(
    x=[0, 0, 0],
    uMin=[0.0, -1.2],
    uMax=[1.0,  1.2],
    dMax=[0.0, 0.0, 0.0],
    uMode="max",
    dMode="min",
)

# ── Unsafe set (obstacle + safety margin) ─────────────────────────────────────
# Obstacle centre (4.3, 0.15), radius 0.50, safety margin 0.20
# → effective radius = 0.70; theta dimension is ignored (cylinder)
obstacle_center = np.array([4.3, 0.15, 0.0])
obstacle_radius = 0.50 + 0.20          # 0.70

unsafe_set = CylinderShape(
    grid=g,
    ignore_dims=[2],                    # ignore theta
    center=obstacle_center,
    radius=obstacle_radius,
)

# ── Time horizon ──────────────────────────────────────────────────────────────
lookback_length = 3.0
t_step = 0.05
small_number = 1e-5
tau = np.arange(start=0, stop=lookback_length + small_number, step=t_step)

# ── Solve ─────────────────────────────────────────────────────────────────────
# "minVWithV0": keep the minimum over time (standard BRT – avoidance).
compMethods = {"TargetSetMode": "minVWithV0"}
result = HJSolver(
    my_car, g, unsafe_set, tau, compMethods,
    saveAllTimeSteps=False,
    accuracy="medium",
)

# result has shape (*grid.pts_each_dim) when saveAllTimeSteps=False
V = result

# ── Spatial derivatives (one array per dimension) ─────────────────────────────
dV_dx1 = computeSpatDerivArray(g, V, deriv_dim=1, accuracy="medium")
dV_dx2 = computeSpatDerivArray(g, V, deriv_dim=2, accuracy="medium")
dV_dx3 = computeSpatDerivArray(g, V, deriv_dim=3, accuracy="medium")

# ── Save to controller share/data directory ───────────────────────────────────
try:
    from ament_index_python.packages import get_package_share_directory
    save_dir = os.path.join(get_package_share_directory("controller"), "data")
except Exception:
    save_dir = os.path.join(os.path.dirname(__file__), "data")

os.makedirs(save_dir, exist_ok=True)
save_path = os.path.join(save_dir, "hj_value_function.npz")

np.savez(
    save_path,
    V=V,
    dV_dx1=dV_dx1,
    dV_dx2=dV_dx2,
    dV_dx3=dV_dx3,
    grid_min=g.min,
    grid_max=g.max,
    grid_pts=g.pts_each_dim,
)
print(f"Saved HJ value function to {save_path}")

# STUDENT CODE END
