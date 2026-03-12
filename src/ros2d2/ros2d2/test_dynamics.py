"""
Standalone test/demo of the reach-avoid game dynamics.

Runs several scenarios to verify the defender (double integrator) and
attacker (single integrator) dynamics, then generates trajectory plots.

Scenarios:
  1. Defender step response — verify double integrator settling behavior
  2. Head-on horizontal pursuit — defender chases attacker in x-y plane
  3. Vertical pursuit — defender above, attacker below climbing
  4. Full 3D pursuit — combined horizontal + vertical with capture check
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from dynamics import (
    GameParams,
    DefenderState,
    AttackerState,
    defender_dynamics,
    attacker_dynamics,
    simulate,
    SimulationResult,
    horizontal_distance,
    vertical_distance,
    is_captured,
)

OUTPUT_DIR = Path(__file__).parent.parent / "plots"


def plot_step_response(params: GameParams):
    """Scenario 1: Defender step response in each axis."""

    dt = 0.01
    T = 6.0
    times = np.arange(0, T, dt)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    labels = ["x", "y", "z"]
    k_vals = [params.kx, params.ky, params.kz]
    max_speeds = [params.U_h_D, params.U_h_D, params.U_z_D]

    for ax_idx in range(3):
        k = k_vals[ax_idx]
        v_cmd = max_speeds[ax_idx]

        positions = np.zeros(len(times))
        velocities = np.zeros(len(times))

        for i in range(1, len(times)):
            accel = k * (v_cmd - velocities[i - 1])
            velocities[i] = velocities[i - 1] + dt * accel
            positions[i] = positions[i - 1] + dt * velocities[i - 1]

        # Analytical steady state: v -> v_cmd, time constant = 1/k
        tau = 1.0 / k
        v_analytical = v_cmd * (1 - np.exp(-k * times))

        ax = axes[ax_idx]
        ax.plot(times, velocities, "b-", label=f"v_{labels[ax_idx]} (Euler)", linewidth=2)
        ax.plot(times, v_analytical, "r--", label=f"v_{labels[ax_idx]} (analytical)", linewidth=1.5)
        ax.axhline(v_cmd, color="gray", linestyle=":", alpha=0.5, label=f"v_cmd = {v_cmd}")
        ax.axvline(tau, color="green", linestyle="--", alpha=0.5, label=f"τ = 1/k = {tau:.2f}s")
        ax.set_ylabel(f"v_{labels[ax_idx]} (m/s)")
        ax.legend(loc="right")
        ax.set_title(f"{labels[ax_idx]}-axis: k_{labels[ax_idx]}={k}, U_max={v_cmd} m/s")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Scenario 1: Defender Double Integrator Step Response", fontsize=14)
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "scenario1_step_response.png", dpi=150)
    print("  Saved scenario1_step_response.png")
    return fig


def pure_pursuit_policy(defender, attacker, params, t):
    """Naive pure pursuit: command velocity toward attacker position."""
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


def straight_line_attacker(goal_direction: np.ndarray, speed_fraction: float = 1.0):
    """Attacker moves in a fixed direction at a fraction of max speed."""
    goal_dir = goal_direction / np.linalg.norm(goal_direction)

    def policy(attacker, defender, params, t):
        h_speed = params.U_h_A * speed_fraction
        z_speed = params.U_z_A * speed_fraction
        return np.array([
            goal_dir[0] * h_speed,
            goal_dir[1] * h_speed,
            goal_dir[2] * z_speed,
        ])

    return policy


def evasive_attacker_policy(attacker, defender, params, t):
    """Attacker moves away from defender at max speed (adversarial)."""
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


def plot_horizontal_pursuit(params: GameParams):
    """Scenario 2: Horizontal pursuit with pure pursuit controller."""

    defender_init = DefenderState(x=30.0, y=15.0, z=5.0, vx=0.0, vy=0.0, vz=0.0)
    attacker_init = AttackerState(x=15.0, y=10.0, z=5.0)

    attacker_policy = straight_line_attacker(
        goal_direction=np.array([-1.0, 0.0, 0.0]), speed_fraction=1.0
    )

    result = simulate(
        defender_init, attacker_init,
        pure_pursuit_policy, attacker_policy,
        params, dt=0.01, T=12.0,
    )

    dp = result.defender_positions()
    ap = result.attacker_positions()
    h_dist = result.horizontal_distances()

    capture_idx = None
    for i in range(len(result.time)):
        if h_dist[i] <= params.d_h:
            capture_idx = i
            break

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    ax = axes[0]
    ax.plot(dp[:, 0], dp[:, 1], "b-", label="Defender", linewidth=2)
    ax.plot(ap[:, 0], ap[:, 1], "r-", label="Attacker", linewidth=2)
    ax.plot(dp[0, 0], dp[0, 1], "bs", markersize=10, label="D start")
    ax.plot(ap[0, 0], ap[0, 1], "r^", markersize=10, label="A start")
    if capture_idx is not None:
        ax.plot(dp[capture_idx, 0], dp[capture_idx, 1], "go", markersize=12, label=f"Capture t={result.time[capture_idx]:.1f}s")
        circle = plt.Circle((dp[capture_idx, 0], dp[capture_idx, 1]), params.d_h, fill=False, color="green", linestyle="--")
        ax.add_patch(circle)
    ax.axvline(x=3, color="green", linestyle="-", alpha=0.5, linewidth=3, label="Target x=3")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Horizontal Trajectories (x-y plane)")
    ax.legend()
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    ax = axes[1]
    ax.plot(result.time, h_dist, "k-", linewidth=2)
    ax.axhline(params.d_h, color="green", linestyle="--", label=f"d_h = {params.d_h}m")
    if capture_idx is not None:
        ax.axvline(result.time[capture_idx], color="red", linestyle=":", label=f"Capture at t={result.time[capture_idx]:.1f}s")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Horizontal Distance (m)")
    ax.set_title("Horizontal Distance Over Time")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.suptitle("Scenario 2: Horizontal Pure Pursuit (Attacker → Target)", fontsize=14)
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "scenario2_horizontal_pursuit.png", dpi=150)
    print("  Saved scenario2_horizontal_pursuit.png")
    return fig


def plot_vertical_pursuit(params: GameParams):
    """Scenario 3: Vertical pursuit matching the paper's vertical game scenarios."""

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Sub-scenario A: Defender above attacker, falling down (Fig 7a in paper)
    defender_init_a = DefenderState(x=0, y=0, z=8.0, vx=0, vy=0, vz=-params.U_z_D)
    attacker_init_a = AttackerState(x=0, y=0, z=2.0)

    def attacker_up(attacker, defender, params, t):
        return np.array([0.0, 0.0, params.U_z_A])

    result_a = simulate(
        defender_init_a, attacker_init_a,
        pure_pursuit_policy, attacker_up,
        params, dt=0.01, T=8.0,
    )

    # Sub-scenario B: Defender below attacker, climbing (Fig 7b in paper)
    defender_init_b = DefenderState(x=0, y=0, z=2.0, vx=0, vy=0, vz=3.0)
    attacker_init_b = AttackerState(x=0, y=0, z=8.0)

    def attacker_down(attacker, defender, params, t):
        return np.array([0.0, 0.0, -params.U_z_A])

    result_b = simulate(
        defender_init_b, attacker_init_b,
        pure_pursuit_policy, attacker_down,
        params, dt=0.01, T=8.0,
    )

    for idx, (result, label) in enumerate([(result_a, "A: Defender above"), (result_b, "B: Defender below")]):
        dp = result.defender_positions()
        ap = result.attacker_positions()
        v_dist = result.vertical_distances()

        ax = axes[idx, 0]
        ax.plot(result.time, dp[:, 2], "b-", label="Defender z", linewidth=2)
        ax.plot(result.time, ap[:, 2], "r-", label="Attacker z", linewidth=2)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("z (m)")
        ax.set_title(f"{label}: Altitude Over Time")
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[idx, 1]
        ax.plot(result.time, v_dist, "k-", linewidth=2)
        ax.axhline(params.d_z, color="green", linestyle="--", label=f"d_z = {params.d_z}m")
        capture_idx = None
        for i in range(len(result.time)):
            if v_dist[i] <= params.d_z:
                capture_idx = i
                break
        if capture_idx is not None:
            ax.axvline(result.time[capture_idx], color="red", linestyle=":", label=f"Capture at t={result.time[capture_idx]:.1f}s")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("|z_D - z_A| (m)")
        ax.set_title(f"{label}: Vertical Distance")
        ax.legend()
        ax.grid(True, alpha=0.3)

    fig.suptitle("Scenario 3: Vertical Pursuit (Defender double integrator, Attacker single integrator)", fontsize=14)
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "scenario3_vertical_pursuit.png", dpi=150)
    print("  Saved scenario3_vertical_pursuit.png")
    return fig


def plot_full_3d_pursuit(params: GameParams):
    """Scenario 4: Full 3D pursuit with capture condition check."""

    defender_init = DefenderState(x=35.0, y=15.0, z=3.0, vx=0.0, vy=0.0, vz=0.0)
    attacker_init = AttackerState(x=20.0, y=12.0, z=7.0)

    attacker_policy = straight_line_attacker(
        goal_direction=np.array([-1.0, -0.2, 0.5]), speed_fraction=1.0
    )

    result = simulate(
        defender_init, attacker_init,
        pure_pursuit_policy, attacker_policy,
        params, dt=0.01, T=15.0,
    )

    dp = result.defender_positions()
    ap = result.attacker_positions()
    h_dist = result.horizontal_distances()
    v_dist = result.vertical_distances()
    e_dist = result.euclidean_distances()

    h_captured = h_dist <= params.d_h
    v_captured = v_dist <= params.d_z
    both_captured = h_captured & v_captured

    capture_idx = None
    for i in range(len(result.time)):
        if both_captured[i]:
            capture_idx = i
            break

    fig = plt.figure(figsize=(16, 10))

    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax3d.plot(dp[:, 0], dp[:, 1], dp[:, 2], "b-", label="Defender", linewidth=2)
    ax3d.plot(ap[:, 0], ap[:, 1], ap[:, 2], "r-", label="Attacker", linewidth=2)
    ax3d.scatter(*dp[0], c="blue", marker="s", s=80, label="D start")
    ax3d.scatter(*ap[0], c="red", marker="^", s=80, label="A start")
    if capture_idx is not None:
        ax3d.scatter(*dp[capture_idx], c="green", marker="o", s=120, label=f"Capture t={result.time[capture_idx]:.1f}s")
    ax3d.set_xlabel("x (m)")
    ax3d.set_ylabel("y (m)")
    ax3d.set_zlabel("z (m)")
    ax3d.set_title("3D Trajectories")
    ax3d.legend(fontsize=8)

    ax_dist = fig.add_subplot(2, 2, 2)
    ax_dist.plot(result.time, h_dist, "b-", label="Horizontal", linewidth=2)
    ax_dist.plot(result.time, v_dist, "r-", label="Vertical", linewidth=2)
    ax_dist.plot(result.time, e_dist, "k--", label="Euclidean 3D", linewidth=1.5, alpha=0.7)
    ax_dist.axhline(params.d_h, color="blue", linestyle=":", alpha=0.5, label=f"d_h={params.d_h}")
    ax_dist.axhline(params.d_z, color="red", linestyle=":", alpha=0.5, label=f"d_z={params.d_z}")
    if capture_idx is not None:
        ax_dist.axvline(result.time[capture_idx], color="green", linestyle="--", label=f"Full capture t={result.time[capture_idx]:.1f}s")
    ax_dist.set_xlabel("Time (s)")
    ax_dist.set_ylabel("Distance (m)")
    ax_dist.set_title("Distances Over Time")
    ax_dist.legend(fontsize=8)
    ax_dist.grid(True, alpha=0.3)

    ax_xy = fig.add_subplot(2, 2, 3)
    ax_xy.plot(dp[:, 0], dp[:, 1], "b-", label="Defender", linewidth=2)
    ax_xy.plot(ap[:, 0], ap[:, 1], "r-", label="Attacker", linewidth=2)
    ax_xy.plot(dp[0, 0], dp[0, 1], "bs", markersize=10)
    ax_xy.plot(ap[0, 0], ap[0, 1], "r^", markersize=10)
    if capture_idx is not None:
        circle = plt.Circle((dp[capture_idx, 0], dp[capture_idx, 1]), params.d_h, fill=False, color="green", linestyle="--")
        ax_xy.add_patch(circle)
        ax_xy.plot(dp[capture_idx, 0], dp[capture_idx, 1], "go", markersize=10)
    ax_xy.set_xlabel("x (m)")
    ax_xy.set_ylabel("y (m)")
    ax_xy.set_title("Top-down (x-y)")
    ax_xy.set_aspect("equal")
    ax_xy.legend()
    ax_xy.grid(True, alpha=0.3)

    ax_cap = fig.add_subplot(2, 2, 4)
    ax_cap.fill_between(result.time, 0, h_captured.astype(float), alpha=0.3, color="blue", label="H captured")
    ax_cap.fill_between(result.time, 0, v_captured.astype(float), alpha=0.3, color="red", label="V captured")
    ax_cap.fill_between(result.time, 0, both_captured.astype(float), alpha=0.5, color="green", label="BOTH (full capture)")
    ax_cap.set_xlabel("Time (s)")
    ax_cap.set_ylabel("Captured?")
    ax_cap.set_title("Capture Status (Decomposed)")
    ax_cap.set_yticks([0, 1])
    ax_cap.set_yticklabels(["No", "Yes"])
    ax_cap.legend()
    ax_cap.grid(True, alpha=0.3)

    fig.suptitle("Scenario 4: Full 3D Pursuit (Pure Pursuit Defender vs Straight-Line Attacker)", fontsize=14)
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "scenario4_full_3d_pursuit.png", dpi=150)
    print("  Saved scenario4_full_3d_pursuit.png")
    return fig


def plot_defender_velocity_profile(params: GameParams):
    """Bonus: Show defender velocity magnitude evolution with different commands."""

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))

    # Horizontal: bang-bang-like control switching
    dt = 0.01
    T = 8.0
    times = np.arange(0, T, dt)

    vx, vy = 0.0, 0.0
    vx_hist, vy_hist = [], []
    speed_hist = []

    for t in times:
        if t < 3.0:
            vx_cmd, vy_cmd = params.U_h_D, 0.0
        elif t < 5.0:
            vx_cmd, vy_cmd = -params.U_h_D * 0.5, params.U_h_D * 0.866
        else:
            vx_cmd, vy_cmd = 0.0, 0.0

        vx += dt * params.kx * (vx_cmd - vx)
        vy += dt * params.ky * (vy_cmd - vy)
        vx_hist.append(vx)
        vy_hist.append(vy)
        speed_hist.append(np.sqrt(vx**2 + vy**2))

    ax = axes[0]
    ax.plot(times, vx_hist, "b-", label="vx", linewidth=2)
    ax.plot(times, vy_hist, "r-", label="vy", linewidth=2)
    ax.plot(times, speed_hist, "k--", label="|v|", linewidth=1.5)
    ax.axhline(params.U_h_D, color="gray", linestyle=":", alpha=0.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (m/s)")
    ax.set_title(f"Horizontal: kx={params.kx}, ky={params.ky}")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Vertical: demonstrate second-order lag
    vz = 0.0
    vz_hist = []
    for t in times:
        if t < 2.0:
            vz_cmd = params.U_z_D
        elif t < 4.0:
            vz_cmd = -params.U_z_D
        else:
            vz_cmd = 0.0

        vz += dt * params.kz * (vz_cmd - vz)
        vz_hist.append(vz)

    ax = axes[1]
    ax.plot(times, vz_hist, "b-", label="vz", linewidth=2)
    ax.axhline(params.U_z_D, color="gray", linestyle=":", alpha=0.3)
    ax.axhline(-params.U_z_D, color="gray", linestyle=":", alpha=0.3)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("vz (m/s)")
    ax.set_title(f"Vertical: kz={params.kz}")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.suptitle("Defender Velocity Profiles (Double Integrator Lag Behavior)", fontsize=14)
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "scenario_velocity_profiles.png", dpi=150)
    print("  Saved scenario_velocity_profiles.png")
    return fig


def main():
    params = GameParams()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print("=" * 60)
    print("Reach-Avoid Game Dynamics — Standalone Test")
    print("=" * 60)
    print(f"\nParameters:")
    print(f"  kx={params.kx}, ky={params.ky}, kz={params.kz}")
    print(f"  Defender max speed: horizontal={params.U_h_D} m/s, vertical={params.U_z_D} m/s")
    print(f"  Attacker max speed: horizontal={params.U_h_A} m/s, vertical={params.U_z_A} m/s")
    print(f"  Speed ratio: {params.U_h_D / params.U_h_A:.1f}x horizontal, {params.U_z_D / params.U_z_A:.1f}x vertical")
    print(f"  Capture radii: d_h={params.d_h} m, d_z={params.d_z} m")
    print()

    print("Scenario 1: Defender step response...")
    plot_step_response(params)

    print("Scenario 2: Horizontal pursuit...")
    plot_horizontal_pursuit(params)

    print("Scenario 3: Vertical pursuit...")
    plot_vertical_pursuit(params)

    print("Scenario 4: Full 3D pursuit...")
    plot_full_3d_pursuit(params)

    print("Bonus: Velocity profiles...")
    plot_defender_velocity_profile(params)

    print(f"\nAll plots saved to {OUTPUT_DIR.resolve()}")
    print("Done!")

    plt.show()


if __name__ == "__main__":
    main()
