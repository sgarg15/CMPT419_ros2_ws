# Dubins Flat Planner
Open-loop trajectory planner for Dubins-like dynamics using differential flatness

## Quickstart for environment setup
Follow these steps to set up the starter ROS2 package in your workspace.

1. Build your ROS2 workspace according to instructions in assignment 0
2. Unzip package into the `src/` folder
3. Build and source the workspace
```bash
cd /workspaces/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

4. Launch the simulation
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

5. Run the `dubins_flat_planner` node
```bash
ros2 run dubins_flat_planner dubins_flat_planner --ros-args --params-file src/dubins_flat_planner/params/default_dubins_flat_planner_params.yaml
```

## Completing the Assignment
Although the package builds and node runs, the implementation of the node and the backend algorithm that it depends on are not complete.

1. Finish the implementation of the backend in `dubins_flat_planner/dubins_flat_planner_algorithm.py`  and ROS2 node in `dubins_flat_planner/dubins_flat_planner.py` by completing the sections marked as `# STUDENT CODE START` and `# STUDENT CODE END`. 
Do not make modifications anywhere else.

   The expected behaviour of the robot should be to roughly travel along a planned trajectory from its current state to the goal.

2. Generate plots, by running the `dubins_flat_planner_algorithm.py` script, to visualize the result which is saved in `dubins_flat_planner.pdf`. 
   
   Attach your plots to the assignment write-up pdf.

```bash
# From ros2_ws/
python3 src/dubins_flat_planner/dubins_flat_planner/dubins_flat_planning_algorithm.py
```

