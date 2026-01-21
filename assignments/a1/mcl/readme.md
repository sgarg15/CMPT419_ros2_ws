# Monte Carlo Localization
Map-based localization via Monte Carlo Localization.

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

4. Launch the simulation and MCL node. This will also open an `rviz2` window that shows a particle cloud (blue arrows) that represents the distribution over the estimated state.
```bash
ros2 launch mcl mcl.launch.py
```

5. Set an initial pose for the robot

6. Teleoperate the robot
```bash
ros2 run turtlebot3_teleop teleop_keyboard 
```

## Important background
* The map is expressed as an `OccupancyGrid` message. More information can be found from the command line via
```bash
ros2 interface show nav_msgs/msg/OccupancyGrid
```

* `OccupancyGrid` expresses the probability of any grid cell being an obstacle. This is important for computing the likelihood of a measurement given a particle. 

* The lidar measurements are expressed as `LaserScan` message. More information can be found from the command line via
```bash
ros2 interface show sensor_msgs/msg/LaserScan
```

* All ROS2 messages can be inspected in a similar fashion. 
* Nav2 provides the Adaptive Monte Carlo Localization (`amcl`) node, which is the industry standard for localization with 2D lidar measurements.

## Completing the Assignment
Although the package builds and node runs, the implementation of the node and the backend algorithm that it depends on are not complete.

1. Finish the implementation of the backend in `mcl/mcl_algorithm.py`  and ROS2 node in `mcl/mcl.py` by completing the sections marked as `# STUDENT CODE START` and `# STUDENT CODE END`. 
Do not make modifications anywhere else.

2. After finishing your implementation, the small blue arrows, which represent particles, should roughly track the robot's actual position when the robot is being teleoperated, or controlled any other way such as through the Dubins Flat Planner.