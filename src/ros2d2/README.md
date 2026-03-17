# ros2d2 — Reach-Avoid Game for 2-Drone Simulation

Two-drone reach-avoid simulation: defender (double integrator) vs attacker (single integrator), with world-frame velocity control, using PX4 SITL and Gazebo Classic.

## Prerequisites

- **ROS 2** (Humble or newer)
- **PX4-Autopilot** built with Gazebo Classic:
  ```bash
  cd PX4-Autopilot
  make px4_sitl_default gazebo-classic
  ```
- **Micro-XRCE-DDS-Agent** (for PX4–ROS 2 bridge)
- **px4_msgs** (from ws_offboard_control or px4_ros_com)

## Quick Start — Full Simulation

One command to run everything (Gazebo, 2× PX4, Agent, game nodes, RViz):

```bash
source /opt/ros/humble/setup.bash
source /workspaces/ros2_ws/install/setup.bash
ros2 launch ros2d2 sim_full.launch.py
```

- Gazebo opens with 2 iris drones; PX4 SITL and Micro-XRCE-DDS-Agent start automatically.
- After ~15 s, game nodes start (state fusion, controller, capture detection, bridges, markers).
- RViz shows blue (D) and red (A) markers. When the defender catches the attacker (within d_h and d_z), capture is logged, game-over is published, and both drones stop.

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `sim_gazebo_px4_only.launch.py` | Gazebo + 2× PX4 SITL + Micro-XRCE-Agent only (no game logic) |
| `sim_gazebo_rviz.launch.py` | Above + static TF + drone markers + RViz |
| `sim_drones.launch.py` | Game nodes: state fusion, controller, capture detection, bridges |
| `sim_2drones_minimal.launch.py` | Forwards to sim_drones |
| `sim_full.launch.py` | One-shot: Gazebo + RViz, then game nodes after ~15 s |

## How to Run (Two-Terminal Workflow)

**Terminal 1 — Simulation & visualization:**
```bash
source /opt/ros/humble/setup.bash
source /workspaces/ros2_ws/install/setup.bash
ros2 launch ros2d2 sim_gazebo_rviz.launch.py
```

**Terminal 2 — Game nodes (after Gazebo is up, ~10–15 s):**
```bash
source /opt/ros/humble/setup.bash
source /workspaces/ros2_ws/install/setup.bash
ros2 launch ros2d2 sim_drones.launch.py
```

## Launch Arguments

### sim_gazebo_px4_only / sim_gazebo_rviz

| Argument | Default | Description |
|----------|---------|--------------|
| `px4_dir` | auto-detected | Path to PX4-Autopilot |
| `world` | `empty` | World name in PX4 sitl_gazebo-classic/worlds/ |
| `spawn_defender_x` | `0.0` | Defender (px4_1) spawn X (m) |
| `spawn_defender_y` | `3.0` | Defender (px4_1) spawn Y (m) |
| `spawn_attacker_x` | `0.0` | Attacker (px4_2) spawn X (m) |
| `spawn_attacker_y` | `0.0` | Attacker (px4_2) spawn Y (m) |

**Example — custom spawn:**
```bash
ros2 launch ros2d2 sim_gazebo_px4_only.launch.py \
  spawn_defender_x:=5.0 spawn_defender_y:=0.0 \
  spawn_attacker_x:=0.0 spawn_attacker_y:=0.0
```

### sim_gazebo_rviz

| Argument | Default | Description |
|----------|---------|-------------|
| `use_rviz` | `true` | Launch RViz (drone markers) |

## Configuration

Game parameters and spawn positions are in:

- **`config/game_params.yaml`** — d_h, d_z, U_h_D, U_h_A, U_z_D, U_z_A, kx, ky, kz, spawn coordinates
- **`config/sim_params.yaml`** — takeoff height, bridge timing
- **Launch args** — override spawn positions without editing YAML

Edit `config/game_params.yaml` to tune capture radii (d_h, d_z), max speeds, etc.

## ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/game/defender/state` | nav_msgs/Odometry | Defender position & velocity (game frame, z-up) |
| `/game/attacker/state` | nav_msgs/Odometry | Attacker position (game frame, z-up) |
| `/game/defender/velocity` | geometry_msgs/Twist | Desired defender velocity (game controller output) |
| `/game/attacker/velocity` | geometry_msgs/Twist | Desired attacker velocity |
| `/game/game_over` | std_msgs/Bool | True when capture occurs (defender within d_h and d_z of attacker) |
| `/game/drone_markers` | visualization_msgs/MarkerArray | RViz markers for drones |

## Capture Detection

When the defender is within **d_h** (horizontal) and **d_z** (vertical) of the attacker:

1. `/game/game_over` is published with `data: true`
2. Log message: `CAPTURE! Defender within d_h=... and d_z=...`
3. Bridge nodes stop integrating velocity — both drones hold position

Tune d_h and d_z in `config/game_params.yaml`.

## Verify Setup

**After sim_gazebo_px4_only or sim_full is running (~10–15 s):**
```bash
ros2 topic list | grep -E "px4|game"
ros2 topic echo /px4_1/fmu/out/vehicle_local_position_v1 --once
ros2 topic echo /game/defender/state --once
ros2 topic echo /game/game_over --once
```

## Build

```bash
cd /workspaces/ros2_ws
colcon build --packages-select ros2d2
source install/setup.bash
```

## Original Single-Drone Test

Terminal 1:
```bash
cd PX4-Autopilot
make px4_sitl gazebo-classic
```

Terminal 2:
```bash
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888
```

Terminal 3:
```bash
source /opt/ros/humble/setup.bash
source ws_offboard_control/install/setup.bash
ros2 run px4_ros_com offboard_control
```
