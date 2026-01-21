import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np

from mcl.mcl_algorithm import MCLAlgorithm

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (
    PoseArray,
    Pose,
    TransformStamped,
    PoseWithCovarianceStamped,
)
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy

import tf2_ros
import math
import random

# ======================
# Helpers: Do not modify!
# ======================

def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ "
    Args: roll, pitch yaw
    Returns: Quaternion [w, x, y, z]
    """
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def euler_from_quaternion(x: float, y: float, z: float, w: float) -> np.ndarray:
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    t0 = 2 * (w * x + y * z)

    t1 = 1 - 2 * (x * x + y * y)

    t2 = 2 * (w * y - x * z)
    # Make sure we're in range of asin
    if t2 > 1:
        t2 = 1
    if t2 < -1:
        t2 = -1

    t3 = 2 * (w * z + x * y)

    t4 = 1 - 2 * (y * y + z * z)

    roll = math.atan2(t0, t1)
    pitch = math.asin(t2)
    yaw = math.atan2(t3, t4)

    return np.array([roll, pitch, yaw])

# ============================
# Class to finish implementing
# ============================

class MCL(Node):

    def __init__(self):
        super().__init__("mcl")

        self.declare_parameter("motion_noise_trans", 0.02)
        self.declare_parameter("motion_noise_rot", 0.01)
        self.declare_parameter("particle_noise_trans", 0.05)
        self.declare_parameter("particle_noise_rot", 0.1)
        self.declare_parameter("num_particles", 500)
        self.declare_parameter("update_dist_thr", 0.1)
        self.declare_parameter("update_angle_thr", 0.2)

        motion_noise_trans = self.get_parameter("motion_noise_trans").value
        motion_noise_rot = self.get_parameter("motion_noise_rot").value
        particle_noise_trans = self.get_parameter("particle_noise_trans").value
        particle_noise_rot = self.get_parameter("particle_noise_rot").value

        num_particles = self.get_parameter("num_particles").value
        self.update_dist_thr = self.get_parameter("update_dist_thr").value
        self.update_angle_thr = self.get_parameter("update_angle_thr").value

        # Initialize backend
        self.mcl = MCLAlgorithm(
            num_particles,
            motion_noise_trans,
            motion_noise_rot,
            particle_noise_trans,
            particle_noise_rot,
        )

        # Track time for dt calculation
        self.last_odom_time = None
        self.last_resample_odom = None
        self.current_odom_pose = [0.0, 0.0, 0.0]

        # --- ROS 2 Infrastructure ---
        self.create_subscription(
            Odometry, "/odom", self.odom_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.initial_pose_callback, 10
        )

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)

        self.particle_pub = self.create_publisher(PoseArray, "/particle_cloud", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("MCL Node Started")



    def odom_callback(self, msg: Odometry):
        """
        Obtains the last control received, and plugs it into the motion model to 
        perform the prediction step. 

        Steps:
        1. Parses the Odometry message to obtain the current robot pose in the odom 
           frame. This is used in scan_callback. Do not modify this part
        2. Parses the Odometry message to obtain the current v and w, and use them to
           call the motion model
        """
        # 1. Update state from Odometry Message
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        current_yaw = self.get_yaw_from_pose(msg.pose.pose)
        self.current_odom_pose = [current_x, current_y, current_yaw]

        # Get current timestamp
        current_time = Time.from_msg(msg.header.stamp)

        if self.last_odom_time is None:
            self.last_odom_time = current_time
            return

        # Calculate dt (time difference in seconds)
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = current_time

        # 2. Extract control and call motion model
        # STUDENT CODE START
        # STUDENT CODE END

    def scan_callback(self, msg: LaserScan):
        """
        Obtains the last set of observations, and use them to perform measurement 
        update steps

        Steps:
        1. Sanity checks: Estimate distance moved since measurement update was last 
           performed, and only perform update if robot has moved sufficiently far. Do 
           not modify this part
        2. Convert the LaserScan message into a list of (range, angle) tuples. Skip any
           invalid ranges such as inf or nan.
        3. Pass in converted observations to mcl.sensor_model_update()
        """
        # 1. Sanity checks
        if self.mcl.map_data is None:
            return

        if self.last_resample_odom is None:
            self.last_resample_odom = self.current_odom_pose
            return

        curr_x, curr_y, curr_yaw = self.current_odom_pose
        last_r_x, last_r_y, last_r_yaw = self.last_resample_odom

        d_dist = math.sqrt((curr_x - last_r_x) ** 2 + (curr_y - last_r_y) ** 2)
        d_angle = abs(
            math.atan2(math.sin(curr_yaw - last_r_yaw), math.cos(curr_yaw - last_r_yaw))
        )

        # Update threshold check
        if d_dist < self.update_dist_thr and d_angle < self.update_angle_thr:
            self.publish_particles()
            self.publish_tf()
            return
        
        self.last_resample_odom = self.current_odom_pose

        # 2. Parse LaserScan
        # STUDENT CODE START
        # STUDENT CODE END

        # 3. Sensor model update
        # STUDENT CODE START
        # STUDENT CODE END

        self.publish_particles()
        self.publish_tf()

    # =======================
    # Helpers: Do not modify!
    # =======================
    def map_callback(self, msg: OccupancyGrid):
        origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.mcl.set_map(
            msg.data, msg.info.resolution, origin, msg.info.width, msg.info.height
        )
        self.get_logger().info(
            f"Map Received! Size: {msg.info.width}x{msg.info.height}"
        )

    def initial_pose_callback(self, msg):
        self.get_logger().info(f"Received Initial Pose")
        init_x = msg.pose.pose.position.x
        init_y = msg.pose.pose.position.y
        init_yaw = self.get_yaw_from_pose(msg.pose.pose)

        self.mcl.particles = []
        for _ in range(self.mcl.num_particles):
            new_x = random.gauss(init_x, 0.2)
            new_y = random.gauss(init_y, 0.2)
            new_theta = random.gauss(init_yaw, 0.1)
            self.mcl.particles.append(
                [new_x, new_y, new_theta, 1.0 / self.mcl.num_particles]
            )

        self.publish_particles()
        self.publish_tf()

    def publish_particles(self):
        msg = PoseArray()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in self.mcl.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            q = quaternion_from_euler(0, 0, p[2])
            pose.orientation.w = q[0]
            pose.orientation.x = q[1]
            pose.orientation.y = q[2]
            pose.orientation.z = q[3]
            msg.poses.append(pose)
        self.particle_pub.publish(msg)

    def publish_tf(self):
        mean_x = np.mean([p[0] for p in self.mcl.particles])
        mean_y = np.mean([p[1] for p in self.mcl.particles])
        mean_sin = np.mean([math.sin(p[2]) for p in self.mcl.particles])
        mean_cos = np.mean([math.cos(p[2]) for p in self.mcl.particles])
        mean_theta = math.atan2(mean_sin, mean_cos)

        odom_x = self.current_odom_pose[0]
        odom_y = self.current_odom_pose[1]
        odom_yaw = self.current_odom_pose[2]

        map_odom_yaw = mean_theta - odom_yaw
        tx = mean_x - (
            odom_x * math.cos(map_odom_yaw) - odom_y * math.sin(map_odom_yaw)
        )
        ty = mean_y - (
            odom_x * math.sin(map_odom_yaw) + odom_y * math.cos(map_odom_yaw)
        )

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, map_odom_yaw)
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        self.tf_broadcaster.sendTransform(t)

    def get_yaw_from_pose(self, pose):
        _, _, yaw = euler_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,   
        )
        return yaw


def main():
    rclpy.init()
    node = MCL()
    rclpy.spin(node)
    rclpy.shutdown()
