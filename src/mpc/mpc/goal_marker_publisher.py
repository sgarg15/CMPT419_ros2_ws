"""Publishes the MPC goal and corridor as visualization markers for RViz."""

import math
from typing import List

import rclpy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from rclpy.node import Node


def _sample_corridor_points(
    x_knots: List[float],
    y_low_knots: List[float],
    y_high_knots: List[float],
    step: float = 0.15,
) -> List[tuple]:
    """Sample points along lower and upper corridor boundaries. Returns (x, y) pairs."""
    points = []
    for i in range(len(x_knots) - 1):
        x0, x1 = x_knots[i], x_knots[i + 1]
        yl0, yl1 = y_low_knots[i], y_low_knots[i + 1]
        yh0, yh1 = y_high_knots[i], y_high_knots[i + 1]
        n = max(1, int((x1 - x0) / step))
        for j in range(n + 1):
            t = j / n
            x = x0 + t * (x1 - x0)
            yl = yl0 + t * (yl1 - yl0)
            yh = yh0 + t * (yh1 - yh0)
            points.append((x, yl))
            points.append((x, yh))
    return points


def quaternion_from_yaw(yaw: float):
    """Create a quaternion from yaw (rotation around z-axis)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GoalMarkerPublisher(Node):
    """Publishes the goal position as a Marker for RViz visualization."""

    def __init__(self):
        super().__init__("goal_marker_publisher")

        self.declare_parameter("goal_x", 3.5)
        self.declare_parameter("goal_y", 2.5)
        self.declare_parameter("goal_theta", 0.0)
        self.declare_parameter("marker_topic", "/goal_marker")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter(
            "x_knots",
            [-5.0, -3.0, -1.0, 1.0, 3.0, 5.0],
        )
        self.declare_parameter(
            "y_low_knots",
            [2.0, 2.4, 2.6, 2.6, 2.4, 2.0],
        )
        self.declare_parameter(
            "y_high_knots",
            [4.0, 4.4, 4.6, 4.6, 4.4, 4.0],
        )

        goal_x = float(self.get_parameter("goal_x").value)
        goal_y = float(self.get_parameter("goal_y").value)
        goal_theta = float(self.get_parameter("goal_theta").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        rate_hz = float(self.get_parameter("publish_rate").value)
        x_knots = [float(x) for x in self.get_parameter("x_knots").value]
        y_low_knots = [float(y) for y in self.get_parameter("y_low_knots").value]
        y_high_knots = [float(y) for y in self.get_parameter("y_high_knots").value]

        self._marker_pub = self.create_publisher(Marker, marker_topic, 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_marker)

        self._goal_x = goal_x
        self._goal_y = goal_y
        self._goal_theta = goal_theta
        self._x_knots = x_knots
        self._y_low_knots = y_low_knots
        self._y_high_knots = y_high_knots

        self.get_logger().info(
            f"Publishing goal marker at ({goal_x}, {goal_y}, {goal_theta})"
        )

    def _publish_marker(self) -> None:
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "mpc_goal"
        msg.id = 0
        msg.type = Marker.ARROW
        msg.action = Marker.ADD

        msg.pose.position.x = self._goal_x
        msg.pose.position.y = self._goal_y
        msg.pose.position.z = 0.1

        q = quaternion_from_yaw(self._goal_theta)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        msg.scale.x = 0.5
        msg.scale.y = 0.1
        msg.scale.z = 0.1

        msg.color.r = 0.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.color.a = 1.0

        self._marker_pub.publish(msg)

        # Corridor boundary: tiny green dots along lower and upper boundaries
        if len(self._x_knots) >= 2 and len(self._y_low_knots) == len(self._x_knots) and len(self._y_high_knots) == len(self._x_knots):
            corridor_pts = _sample_corridor_points(
                self._x_knots, self._y_low_knots, self._y_high_knots, step=0.15
            )
            if corridor_pts:
                corr_msg = Marker()
                corr_msg.header.frame_id = "map"
                corr_msg.header.stamp = self.get_clock().now().to_msg()
                corr_msg.ns = "corridor"
                corr_msg.id = 0
                corr_msg.type = Marker.POINTS
                corr_msg.action = Marker.ADD
                corr_msg.scale.x = 0.08
                corr_msg.scale.y = 0.08
                corr_msg.scale.z = 0.02
                corr_msg.color.r = 0.0
                corr_msg.color.g = 1.0
                corr_msg.color.b = 0.0
                corr_msg.color.a = 0.9
                for x, y in corridor_pts:
                    p = Point()
                    p.x = float(x)
                    p.y = float(y)
                    p.z = 0.05
                    corr_msg.points.append(p)
                self._marker_pub.publish(corr_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
