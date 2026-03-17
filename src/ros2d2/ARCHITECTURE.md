# ROS 2 + Gazebo + PX4 2-Drone Reach-Avoid Simulation — Architecture & Implementation Plan

**Document Version:** 1.0  
**Scope:** PHASE 1 — Architecture and implementation plan only (no code).

---

## Section 1: Current Architecture Diagnosis

### 1.1 What Exists Today

| Component | Location | Current State |
|-----------|----------|---------------|
| **ros2d2 dynamics** | `ros2d2/dynamics.py` | Mature: `DefenderState`, `AttackerState`, `GameParams`, defender/attacker dynamics, control clipping, capture checks, `simulate()` |
| **ros2d2 tests** | `ros2d2/test_dynamics.py` | Mature: step response, horizontal/vertical/full 3D pursuit with pure pursuit + straight-line attacker |
| **ros2d2 node** | `ros2d2/ros2d2_node.py` | Stub: logs "ros2d2 node started" and spins |
| **Offboard control** | `ws_offboard_control/px4_ros_com` | Single-drone, position-only takeoff/land; hardcoded `/fmu/in/*` topics |
| **PX4 SITL + Gazebo** | `make px4_sitl gazebo-classic` | Single-vehicle default |
| **Micro-XRCE-DDS-Agent** | `MicroXRCEAgent udp4 -p 8888` | Single agent, no multi-vehicle namespacing |
| **Gazebo world** | mpc `empty_room.world`, PX4 `empty.world` | Existing worlds but not integrated with ros2d2 |

### 1.2 Inferred Pain Points and Coupling Problems

1. **Single-vehicle assumption everywhere**
   - `ws_offboard_control` publishes to `/fmu/in/*` with no namespace.
   - No concept of defender vs attacker, or two PX4 instances.

2. **State source ambiguity**
   - Dynamics and tests use an idealized model (double/single integrator).
   - Real state will come from PX4 `VehicleLocalPosition` (NED frame).
   - No explicit design for where state comes from (Gazebo vs PX4, frame, validity checks).

3. **No unified game state**
   - Controller logic (pure pursuit, straight-line attacker) lives in `test_dynamics.py`.
   - No ROS node that subscribes to both drones, fuses state, and exposes a single game state.

4. **Command interface mismatch**
   - dynamics outputs world-frame velocity `[vx, vy, vz]` in a z-up convention.
   - PX4 uses NED (z-down); `TrajectorySetpoint` supports velocity but current offboard uses position.
   - Offboard heartbeat uses `position=True` only; velocity control requires `velocity=True` and `position=[NaN, NaN, NaN]`.

5. **Launch and bringup fragmentation**
   - Manual 3-terminal flow: (1) PX4 SITL, (2) MicroXRCEAgent, (3) offboard_control.
   - No integrated launch for simulation + ROS nodes.
   - No automated 2-drone spawn or namespace setup.

6. **ws_offboard_control vs ros2d2 responsibility blur**
   - Offboard logic (arm, offboard mode, heartbeats, setpoints) is embedded in a demo node.
   - No separation between "bridge to PX4" and "game controller that produces commands."

7. **Frame convention mismatch**
   - `dynamics.py`: z positive = up (paper convention).
   - PX4 `VehicleLocalPosition`: NED — z positive = down.
   - Transform layer is missing.

---

## Section 2: Recommended Target Architecture

### 2.1 Layered View

```
┌─────────────────────────────────────────────────────────────────────────┐
│ SIMULATION LAYER                                                        │
│   - Gazebo world (ros2d2-owned simple world)                            │
│   - 2× iris spawned via PX4 sitl_multiple_run or custom launch          │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ FLIGHT-CONTROL LAYER                                                     │
│   - PX4 instance 1 (defender):  px4_instance=1 → /px4_1/...             │
│   - PX4 instance 2 (attacker): px4_instance=2 → /px4_2/...              │
│   - Micro-XRCE-DDS-Agent (single agent, both connect over UDP)          │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                    ┌───────────────┴───────────────┐
                    ▼                               ▼
┌──────────────────────────────────┐  ┌──────────────────────────────────┐
│ px4_bridge_defender_node          │  │ px4_bridge_attacker_node          │
│ - Sub: /game/defender/velocity    │  │ - Sub: /game/attacker/velocity     │
│ - Pub: /px4_1/fmu/in/*            │  │ - Pub: /px4_2/fmu/in/*            │
│ - Arm, offboard, heartbeat, cmd    │  │ - Arm, offboard, heartbeat, cmd   │
└──────────────────────────────────┘  └──────────────────────────────────┘
                    │                               │
                    └───────────────┬───────────────┘
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ GAME / CONTROL LAYER (ros2d2)                                           │
│   state_fusion_node:                                                    │
│     - Sub: /px4_1/fmu/out/vehicle_local_position                        │
│     - Sub: /px4_2/fmu/out/vehicle_local_position                        │
│     - Pub: /game/defender/state, /game/attacker/state (Odometry)        │
│                                                                         │
│   game_controller_node:                                                 │
│     - Sub: /game/defender/state, /game/attacker/state                  │
│     - Pub: /game/defender/velocity                                      │
│     - Pub: /game/attacker/velocity                                      │
│     - Logic: simple policies first (pure pursuit, straight-line)        │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 Node Responsibilities

| Node | Responsibility | Does NOT |
|------|----------------|----------|
| **state_fusion_node** | Sub to both PX4 positions; NED→game frame; publish defender/attacker `Odometry` on `/game/*/state` | Do control logic; talk to PX4 |
| **game_controller_node** | Sub to defender/attacker Odometry; run defender/attacker policies; publish desired velocities | Handle PX4; arm/disarm; manage offboard |
| **px4_bridge_defender_node** | Sub to `/game/defender/velocity`; arm, offboard, heartbeat; publish setpoints to `/px4_1/fmu/in/*` | Compute policies; know attacker |
| **px4_bridge_attacker_node** | Same for attacker → `/px4_2/fmu/in/*` | Same |

### 2.3 Launch-Time vs Node Responsibilities

| Responsibility | Owner |
|----------------|-------|
| Start Gazebo with world | Launch |
| Spawn 2 PX4 instances + 2 iris | Launch (or delegate to sitl_multiple_run) |
| Start Micro-XRCE-DDS-Agent | Launch |
| Start state_fusion, game_controller, bridge nodes | Launch |
| Arm, offboard, heartbeat, setpoint rate | Bridge nodes |
| Coordinate arm order (e.g. defender first, attacker second) | Bridge nodes (or optional coordinator) |
| World file path, spawn positions | Launch args / config |

### 2.4 What Stays in ws_offboard_control vs Moves to ros2d2

| Item | Recommendation |
|------|----------------|
| **px4_ros_com, px4_msgs** | Stay in ws_offboard_control (or as workspace dependency). These are PX4 interface packages. |
| **offboard_control.py (current demo)** | Keep as reference/demo for single vehicle. Do not extend for 2-drone. |
| **Bridge nodes** | New nodes in **ros2d2**. They are game-specific: they subscribe to game topics and publish to namespaced PX4 topics. |
| **state_fusion, game_controller** | In **ros2d2** — core game logic. |
| **World file** | Owned by **ros2d2** (e.g. `ros2d2/worlds/`) for reach-avoid use case. |
| **Launch files** | **ros2d2/launch/** for the integrated 2-drone simulation. |

---

## Section 3: Topic/Interface Design

### 3.1 Namespace Strategy

- **PX4 topics (from Micro-XRCE-DDS):** Use PX4’s built-in multi-vehicle namespacing.
  - Instance 1: `/px4_1/fmu/in/*`, `/px4_1/fmu/out/*`
  - Instance 2: `/px4_2/fmu/in/*`, `/px4_2/fmu/out/*`

- **Game topics:** Use `/game/` prefix for game-layer topics.
  - Keeps them distinct from PX4 and future obstacles/targets.

### 3.2 Concrete Topic Names

| Purpose | Topic | Message Type | Publisher | Subscriber |
|---------|-------|--------------|-----------|------------|
| Defender state (raw) | `/px4_1/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | PX4 via DDS | state_fusion_node |
| Attacker state (raw) | `/px4_2/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | PX4 via DDS | state_fusion_node |
| Defender game state | `/game/defender/state` | `nav_msgs/Odometry` | state_fusion_node | game_controller_node |
| Attacker game state | `/game/attacker/state` | `nav_msgs/Odometry` | state_fusion_node | game_controller_node |
| Defender desired velocity | `/game/defender/velocity` | `geometry_msgs/Twist` | game_controller_node | px4_bridge_defender_node |
| Attacker desired velocity | `/game/attacker/velocity` | `geometry_msgs/Twist` | game_controller_node | px4_bridge_attacker_node |
| Defender PX4 offboard mode | `/px4_1/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | px4_bridge_defender_node | PX4 |
| Defender PX4 setpoint | `/px4_1/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | px4_bridge_defender_node | PX4 |
| Defender PX4 vehicle cmd | `/px4_1/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | px4_bridge_defender_node | PX4 |
| Attacker PX4 offboard mode | `/px4_2/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | px4_bridge_attacker_node | PX4 |
| Attacker PX4 setpoint | `/px4_2/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | px4_bridge_attacker_node | PX4 |
| Attacker PX4 vehicle cmd | `/px4_2/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | px4_bridge_attacker_node | PX4 |

### 3.3 Optional Debug/Logging Topics

| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/game/debug/defender_pose` | `geometry_msgs/PoseStamped` | Defender position in game frame (RViz) |
| `/game/debug/attacker_pose` | `geometry_msgs/PoseStamped` | Attacker position in game frame (RViz) |
| `/game/debug/distances` | `std_msgs/Float32MultiArray` | h_dist, v_dist, capture status |

---

## Section 4: Folder/Package Structure

### 4.1 Target Layout

```
ros2d2/
├── package.xml
├── setup.py
├── resource/
│   └── ros2d2
├── ros2d2/
│   ├── __init__.py
│   ├── dynamics.py              # UNCHANGED (keep as-is)
│   ├── controllers/
│   │   ├── __init__.py
│   │   ├── simple_policies.py    # pure_pursuit, straight_line_attacker (from test_dynamics)
│   │   └── base.py              # (optional) base class / interface for future HJ
│   ├── nodes/
│   │   ├── __init__.py
│   │   ├── state_fusion_node.py
│   │   ├── game_controller_node.py
│   │   ├── px4_bridge_defender_node.py
│   │   └── px4_bridge_attacker_node.py
│   └── ros2d2_node.py           # DEPRECATE or repurpose as optional launcher
├── launch/
│   ├── sim_2drones.launch.py     # Full stack: Gazebo + PX4 x2 + Agent + ROS nodes
│   ├── sim_2drones_minimal.launch.py  # ROS nodes only (assumes sim already running)
│   └── sim_gazebo_px4_only.launch.py  # Gazebo + PX4 x2 + Agent (no game nodes)
├── worlds/
│   └── empty_2drones.world      # Simple world for 2-drone (ground, sun, empty)
├── config/
│   ├── game_params.yaml          # GameParams as YAML (optional override)
│   └── sim_params.yaml           # Spawn positions, takeoff height, etc.
├── test/
│   ├── test_dynamics.py         # UNCHANGED
│   └── ...
└── ARCHITECTURE.md              # This document
```

### 4.2 File Contents (High Level)

| File | Purpose |
|------|---------|
| `dynamics.py` | No changes. Keep existing API. |
| `controllers/simple_policies.py` | Extract `pure_pursuit_policy`, `straight_line_attacker` from test_dynamics. Pure Python, no ROS. |
| `nodes/state_fusion_node.py` | Subs to both PX4 positions; converts NED→game frame; builds DefenderState/AttackerState; publishes Odometry on `/game/defender/state` and `/game/attacker/state`. |
| `nodes/game_controller_node.py` | Subs to defender/attacker Odometry; calls policies; publishes Twist for defender and attacker. |
| `nodes/px4_bridge_defender_node.py` | Subs to `/game/defender/velocity`; arm/offboard state machine; publishes to `/px4_1/fmu/in/*`; `target_system=2`. |
| `nodes/px4_bridge_attacker_node.py` | Same for attacker; `target_system=3`. |
| `worlds/empty_2drones.world` | Minimal SDF: ground, sun, empty space. |
| `launch/sim_2drones.launch.py` | Orchestrates Gazebo, PX4 x2, Agent, then all 4 ROS nodes. |
| `config/sim_params.yaml` | Spawn positions, takeoff height, bridge timing (arm delay, etc.). |

### 4.3 Dependencies

Add to `package.xml` and `setup.py`:

- `px4_msgs` (from ws_offboard_control)
- `geometry_msgs`
- `std_msgs`
- `launch`, `launch_ros`
- `ament_cmake` or equivalent for installing worlds/config

---

## Section 5: Launch Design

### 5.1 Launch Files

| Launch File | Role |
|-------------|------|
| `sim_gazebo_px4_only.launch.py` | Start Gazebo with `empty_2drones.world`, spawn 2 PX4 SITL instances (via subprocess or `sitl_multiple_run.sh`), start Micro-XRCE-DDS-Agent. No game nodes. |
| `sim_2drones_minimal.launch.py` | Start only the 4 ROS nodes (state_fusion, game_controller, bridge_defender, bridge_attacker). Use when sim already running. |
| `sim_2drones.launch.py` | Full bringup: calls `sim_gazebo_px4_only` then `sim_2drones_minimal` with delay. |

### 5.2 Bringup Order

1. **Gazebo**
   - `gzserver` with `empty_2drones.world`.
   - Optional: `gzclient` for GUI.

2. **PX4 instances**
   - Use `sitl_multiple_run.sh -n 2 -m iris -w <world>` OR custom Python that spawns 2 PX4 processes with `px4_instance=1` and `px4_instance=2`.
   - Ensure `PX4_UXRCE_DDS_NS` is set per instance: `px4_1`, `px4_2` (or rely on default for instance>0).

3. **Micro-XRCE-DDS-Agent**
   - Single process: `MicroXRCEAgent udp4 -p 8888`.
   - Both PX4 instances connect to it.

4. **ROS nodes (after ~5–10 s delay)**
   - state_fusion_node
   - game_controller_node
   - px4_bridge_defender_node
   - px4_bridge_attacker_node

### 5.3 Critical Details

- PX4 multi-vehicle: `sitl_multiple_run.sh` uses instance 1, 2, ... (not 0). So:
  - Instance 1 → namespace `px4_1`, MAV_SYS_ID=2
  - Instance 2 → namespace `px4_2`, MAV_SYS_ID=3
- `target_system` in VehicleCommand: 2 for defender, 3 for attacker.
- Bridge nodes should wait for `vehicle_local_position` to be valid before arming.

---

## Section 6: Milestone Implementation Order

### M1: Package Setup and Config (No Sim)
- Add `controllers/`, `nodes/`, `launch/`, `worlds/`, `config/`.
- Extract `pure_pursuit_policy` and `straight_line_attacker` into `controllers/simple_policies.py`.
- Add `game_params.yaml` and `sim_params.yaml`.
- **Test:** `python3 -m ros2d2.controllers.simple_policies` (or unit test) that policies run without ROS.

### M2: World and Gazebo-Only Bringup
- Create `empty_2drones.world` (minimal).
- Create launch that starts Gazebo + `sitl_multiple_run.sh -n 2` + Agent.
- **Test:** Two iris in Gazebo, PX4 running, Agent up. Verify `/px4_1/fmu/out/vehicle_local_position` and `/px4_2/fmu/out/vehicle_local_position` with `ros2 topic echo`.

### M3: state_fusion_node
- Implement node: subs to both PX4 positions, NED→game frame, build DefenderState/AttackerState.
- Publish `/game/defender/state` and `/game/attacker/state` as `nav_msgs/Odometry`.
- **Test:** With M2 running, start only state_fusion. Echo both Odometry topics, verify values when moving drones manually (if possible) or just check structure.

### M4: game_controller_node
- Implement node: sub to `/game/defender/state` and `/game/attacker/state`, run pure pursuit + straight-line attacker.
- Publish `/game/defender/velocity` and `/game/attacker/velocity` (Twist).
- **Test:** With M2+M3, start game_controller. Echo velocity topics, verify commands change as drones move.

### M5: px4_bridge_defender_node (Velocity Mode)
- Implement bridge: sub to `/game/defender/velocity`, publish velocity setpoints to `/px4_1/fmu/in/*`.
- Use `OffboardControlMode(velocity=True, position=False)` and `TrajectorySetpoint` with `position=[NaN,NaN,NaN]`, velocity filled.
- Implement arm/offboard state machine.
- **Test:** With M2+M3+M4, start only defender bridge. Defender should move toward attacker. Attacker remains idle (no attacker bridge yet).

### M6: px4_bridge_attacker_node
- Same as M5 for attacker.
- **Test:** Full closed loop: both drones move. Defender pursues, attacker goes straight line. Observe capture or separation.

### M7: Integrated Launch and Cleanup
- Implement `sim_2drones.launch.py` and `sim_2drones_minimal.launch.py`.
- Add config args for spawn positions, takeoff height.
- **Test:** Single command brings up full stack; both drones fly closed-loop.

### M8: Debug Topics and Robustness (Optional)
- Add `/game/debug/*` topics.
- Add validity checks (position/velocity valid flags from PX4).
- Tune timing (heartbeat rate, arm delay).

---

## Section 7: Risk List and Mitigation

| Risk | Mitigation |
|------|------------|
| **PX4 namespace confusion** | Use explicit `px4_1`, `px4_2` via `PX4_UXRCE_DDS_NS` or default for multi-vehicle. Document mapping: defender=1, attacker=2. |
| **Wrong setpoint topic** | Bridge nodes use remapping or hardcoded `/px4_<n>/fmu/in/trajectory_setpoint`. No shared logic between defender/attacker topic names. |
| **World vs body frame** | Game layer and bridges use **world-frame** velocities. PX4 `TrajectorySetpoint.velocity` is in NED world frame. No body-frame conversion. |
| **NED vs game frame (z)** | state_fusion converts: `game_z = -px4_z`, `game_vz = -px4_vz`. Bridges convert back: `px4_vz = -game_vz`. Centralize in state_fusion and bridges. |
| **Offboard activation timing** | PX4 requires setpoints before switching to offboard. Bridges publish heartbeat+setpoint for ~1 s before sending DO_SET_MODE. Rate: 10–20 Hz. |
| **State source ambiguity** | Use PX4 `VehicleLocalPosition` only. Check `xy_valid`, `z_valid`, `v_xy_valid`, `v_z_valid` before using. Ignore Gazebo topics for control. |
| **Launch complexity** | Split into 3 launch files. Use `IncludeLaunchDescription` and `TimerAction` for sequencing. Prefer calling `sitl_multiple_run.sh` over reimplementing. |
| **Agent port** | Use default 8888. If multiple agents needed later, use different ports and document. |
| **target_system wrong** | Defender bridge: `target_system=2`. Attacker bridge: `target_system=3`. Match MAV_SYS_ID from PX4 multi-vehicle. |

---

## Section 8: Message-Type Recommendation

**Recommendation: Use standard messages first. Add a custom `GameState` later if needed.**

| Topic | First Version | Rationale |
|-------|---------------|-----------|
| `/game/state` | `geometry_msgs/PoseArray` (pose[0]=defender, pose[1]=attacker) + `geometry_msgs/Twist` for velocities (or second PoseArray with velocity in position slots as hack) | Avoid new package. PoseArray is awkward for (pos_D, vel_D, pos_A). |
| Better option | `geometry_msgs/PoseStamped` x2 on `/game/defender/state`, `/game/attacker/state` + put velocity in custom header or use TwistStamped | Simpler. |
| **Practical choice** | Publish **two** topics: `/game/defender/state` (PoseStamped for position + Twist for velocity in one callback) or use `nav_msgs/Odometry` which has pose + twist | `nav_msgs/Odometry` is ideal: position, orientation, linear velocity. Use one per drone. |
| **Final recommendation** | `/game/defender/state` and `/game/attacker/state` as `nav_msgs/Odometry` | Standard, has pose+twist, no new interfaces. state_fusion publishes both; game_controller subscribes to both. |

For desired velocity:
- Use `geometry_msgs/Twist` for `/game/defender/velocity` and `/game/attacker/velocity` (linear only; angular unused).

---

## Section 9: First-Working-Version Scope

**Absolute minimum for "first working version":**

1. **Simulation**
   - Gazebo with minimal world.
   - 2× iris via PX4 multi-vehicle.
   - Micro-XRCE-DDS-Agent.

2. **Nodes**
   - state_fusion_node (both positions → Odometry x2).
   - game_controller_node (pure pursuit defender, straight-line attacker).
   - px4_bridge_defender_node.
   - px4_bridge_attacker_node.

3. **Out of scope for first version**
   - Target region / obstacles.
   - HJ value functions / paper switching logic.
   - Capture detection in the loop (just let them fly).
   - Landing automation (optional manual land or let run).
   - Custom `GameState` message.
   - RViz visualization (nice-to-have).

4. **Success criterion**
   - Both drones arm, go offboard.
   - Defender moves toward attacker.
   - Attacker moves in a fixed direction.
   - Both respond to velocity commands in a stable way.

---

## Section 10: Concrete Next Action Items for Implementation

After your approval, implementation order:

1. **Create folder structure** and `controllers/simple_policies.py` (extract from test_dynamics).
2. **Create `empty_2drones.world`** and verify Gazebo + 2-drone PX4 bringup.
3. **Implement state_fusion_node** with NED→game transform and Odometry publishing.
4. **Implement game_controller_node** using simple policies.
5. **Implement px4_bridge_defender_node** with velocity-mode setpoints.
6. **Implement px4_bridge_attacker_node**.
7. **Create launch files** and perform end-to-end test.
8. **Tune timing** (arm delay, heartbeat rate) and add basic error handling.

---

*End of Architecture Document. Awaiting approval before implementation.*
