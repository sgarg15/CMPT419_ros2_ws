#!/usr/bin/env python3
"""
Publishes RViz markers to distinguish defender (blue) and attacker (red).
Subscribe to /game/defender/state and /game/attacker/state; publish to /game/drone_markers.
Run rviz2 and add a Marker display on /game/drone_markers to see labels.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray


class DroneMarkersNode(Node):
    """Publishes colored sphere + text markers for defender and attacker."""

    def __init__(self):
        super().__init__("drone_markers_node")

        self._defender_odom = None
        self._attacker_odom = None

        self._defender_sub = self.create_subscription(
            Odometry, "/game/defender/state", self._defender_cb, 10
        )
        self._attacker_sub = self.create_subscription(
            Odometry, "/game/attacker/state", self._attacker_cb, 10
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, "/game/drone_markers", 10
        )
        self._timer = self.create_timer(0.1, self._publish_markers)  # 10 Hz

    def _defender_cb(self, msg: Odometry):
        self._defender_odom = msg

    def _attacker_cb(self, msg: Odometry):
        self._attacker_odom = msg

    def _make_sphere_marker(self, ns: str, mid: int, x: float, y: float, z: float,
                            r: float, g: float, b: float) -> Marker:
        m = Marker()
        m.header.frame_id = "world"
        m.ns = ns
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = 0.4
        m.scale.y = 0.4
        m.scale.z = 0.4
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.9
        return m

    def _make_text_marker(self, ns: str, mid: int, x: float, y: float, z: float,
                          text: str, r: float, g: float, b: float) -> Marker:
        m = Marker()
        m.header.frame_id = "world"
        m.ns = ns
        m.id = mid
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z + 0.35
        m.pose.orientation.w = 1.0
        m.scale.z = 0.5
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 1.0
        m.text = text
        return m

    def _publish_markers(self):
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Fallback: large green sphere + text at origin when no drone data
        if self._defender_odom is None and self._attacker_odom is None:
            # Big green sphere (impossible to miss)
            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.header.stamp = now
            sphere.ns = "status"
            sphere.id = 0
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = 0.0
            sphere.pose.position.y = 0.0
            sphere.pose.position.z = 2.0
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.5
            sphere.scale.y = 0.5
            sphere.scale.z = 0.5
            sphere.color.r = 0.2
            sphere.color.g = 1.0
            sphere.color.b = 0.2
            sphere.color.a = 0.9
            arr.markers.append(sphere)
            # Text label
            txt = Marker()
            txt.header.frame_id = "map"
            txt.header.stamp = now
            txt.ns = "status"
            txt.id = 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = 0.0
            txt.pose.position.y = 0.0
            txt.pose.position.z = 4.0
            txt.scale.z = 0.5
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 1.0
            txt.color.a = 1.0
            txt.text = "Waiting... Run sim_drones"
            arr.markers.append(txt)

        if self._defender_odom is not None:
            p = self._defender_odom.pose.pose.position
            m1 = self._make_sphere_marker("defender", 0, p.x, p.y, p.z, 0.2, 0.4, 1.0)
            m2 = self._make_text_marker("defender", 1, p.x, p.y, p.z, "D", 0.2, 0.4, 1.0)
            m1.header.stamp = now
            m2.header.stamp = now
            arr.markers.extend([m1, m2])

        if self._attacker_odom is not None:
            p = self._attacker_odom.pose.pose.position
            m1 = self._make_sphere_marker("attacker", 2, p.x, p.y, p.z, 1.0, 0.2, 0.2)
            m2 = self._make_text_marker("attacker", 3, p.x, p.y, p.z, "A", 1.0, 0.2, 0.2)
            m1.header.stamp = now
            m2.header.stamp = now
            arr.markers.extend([m1, m2])

        self._marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = DroneMarkersNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
