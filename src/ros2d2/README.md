# Running Test on Pre-Built Things:

Terminal 1
cd /workspaces/ros2_ws/src/PX4-Autopilot
make px4_sitl gazebo-classic

Terminal 2
source /opt/ros/humble/setup.bash
MicroXRCEAgent udp4 -p 8888

Terminal 3
cd /workspaces/ros2_ws/src/ws_offboard_control
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run px4_ros_com offboard_control