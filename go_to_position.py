#!/usr/bin/env python3
"""
Dynamic local navigator / safety controller for quad_final.

The global A* path tells the drone where to go.  This node is the second,
independent safety layer: every control cycle it looks at the latest RGB-D
point cloud and 360-degree LiDAR sectors, computes a fused clearance, and
modifies the desired velocity before publishing /quadcopter/cmd_vel.

Important: Gazebo's MulticopterVelocityControl expects linear velocity in the
vehicle BODY frame, so world-frame path velocity is transformed into body
coordinates using the actual odometry yaw.
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

MAX_HORIZONTAL_SPEED = 1.5
MAX_VERTICAL_SPEED = 0.7
GOAL_TOLERANCE = 0.65
LOOKAHEAD = 1.6
MIN_CLEARANCE = 0.72
SLOW_CLEARANCE = 2.0
STOP_CLEARANCE = 0.85
EMERGENCY_CLEARANCE = 0.55
SENSOR_TIMEOUT = 0.55
CONTROL_RATE = 30.0


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def percentile_valid(values, percentile, minimum=0.15, maximum=10.0):
    values = np.asarray(values, dtype=np.float64)
    if values.size == 0:
        return None
    values = values[np.isfinite(values)]
    values = values[(values > minimum) & (values < maximum)]
    if values.size < 5:
        return None
    return float(np.percentile(values, percentile))


class SmartNavigator(Node):
    def __init__(self):
        super().__init__('smart_navigator')

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.24
        self.current_yaw = 0.0
        self.current_vx_world = 0.0
        self.current_vy_world = 0.0
        self.have_odom = False

        self.path = []
        self.path_index = 0
        self.goal = None

        self.camera_front = None
        self.camera_left = None
        self.camera_right = None
        self.lidar_front = None
        self.lidar_left = None
        self.lidar_right = None
        self.last_camera_time = 0.0
        self.last_lidar_time = 0.0
        self.previous_front = None
        self.previous_front_time = None
        self.closing_speed = 0.0

        self.cmd_pub = self.create_publisher(
            Twist, '/quadcopter/cmd_vel', 10
        )
        self.status_pub = self.create_publisher(
            String, '/quadcopter/avoidance_status', 10
        )
        self.debug_pub = self.create_publisher(
            MarkerArray, '/quadcopter/avoidance_debug', 10
        )

        self.create_subscription(
            Odometry, '/quadcopter/gps_odometry', self.position_callback, 10
        )
        self.create_subscription(
            Odometry, '/quadcopter/odometry', self.gazebo_odom_callback, 10
        )
        self.create_subscription(
            Path, '/quadcopter/planned_path', self.path_callback, 10
        )
        self.create_subscription(
            PointCloud2, '/quadcopter/camera/points', self.camera_callback, qos_sensor
        )
        self.create_subscription(
            LaserScan, '/quadcopter/scan', self.lidar_callback, qos_sensor
        )
        self.timer = self.create_timer(1.0 / CONTROL_RATE, self.control_loop)

        self.get_logger().info(
            'Smart navigator ready: global A* + RGB-D + 360 LiDAR local safety.'
        )

    def position_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.have_odom = True

    def gazebo_odom_callback(self, msg):
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.current_vx_world = msg.twist.twist.linear.x
        self.current_vy_world = msg.twist.twist.linear.y
        # Gazebo publishes these velocities in world coordinates for the
        # odometry message used here.

    def path_callback(self, msg):
        if not msg.poses:
            return
        self.path = [
            (
                p.pose.position.x,
                p.pose.position.y,
                p.pose.position.z,
            )
            for p in msg.poses
        ]
        self.goal = self.path[-1]
        # A replanned path starts at the current pose.  Search the whole new
        # list instead of carrying the old waypoint index across a new path.
        self.path_index = self._nearest_path_index(start_index=0)

    def _nearest_path_index(self, start_index=None):
        if not self.path:
            return 0
        start = self.path_index if start_index is None else start_index
        start = min(max(start, 0), len(self.path) - 1)
        best = start
        best_dist = math.inf
        for i in range(start, len(self.path)):
            dx = self.path[i][0] - self.current_x
            dy = self.path[i][1] - self.current_y
            d = math.hypot(dx, dy)
            if d < best_dist:
                best = i
                best_dist = d
        return best

    def camera_callback(self, msg):
        try:
            if hasattr(pc2, 'read_points_numpy'):
                pts = pc2.read_points_numpy(
                    msg, field_names=['x', 'y', 'z'], skip_nans=True
                )
                pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
            else:
                pts = np.asarray(
                    list(pc2.read_points(
                        msg,
                        field_names=('x', 'y', 'z'),
                        skip_nans=True,
                    )),
                    dtype=np.float64,
                ).reshape(-1, 3)

            pts = pts[np.all(np.isfinite(pts), axis=1)]
            # Gazebo's model convention is X forward / Y left / Z up.
            pts = pts[(pts[:, 0] > 0.2) & (pts[:, 0] < 10.0)]
            if pts.size == 0:
                return

            # Convert the relative camera height into a world-height band.
            # This suppresses ground returns at z=0 without hard-coding the
            # drone's navigation altitude.
            world_z = self.current_z + pts[:, 2]
            z_band = (world_z > 0.25) & (world_z < self.current_z + 1.5)
            pts = pts[z_band]
            if pts.size == 0:
                return

            x = pts[:, 0]
            y = pts[:, 1]
            # Narrower front ROI = safer than treating the entire camera FOV
            # as an obstacle in front of the drone.
            self.camera_front = percentile_valid(
                x[(np.abs(y) < 0.9)], 15
            )
            self.camera_left = percentile_valid(
                x[(y > 0.25) & (y < 2.8)], 15
            )
            self.camera_right = percentile_valid(
                x[(y < -0.25) & (y > -2.8)], 15
            )
            self.last_camera_time = self.get_clock().now().nanoseconds / 1e9
        except Exception as exc:
            self.get_logger().warn(
                f'Camera processing failed: {exc}',
                throttle_duration_sec=2.0,
            )

    def lidar_callback(self, msg):
        try:
            ranges = np.asarray(msg.ranges, dtype=np.float64)
            count = len(ranges)

            def sector(center_deg, width_deg):
                if count == 0 or msg.angle_increment == 0:
                    return None
                center = math.radians(center_deg)
                half = math.radians(width_deg) / 2.0
                indices = []
                for i, value in enumerate(ranges):
                    angle = msg.angle_min + i * msg.angle_increment
                    a = wrap_angle(angle - center)
                    if abs(a) <= half:
                        indices.append(value)
                return percentile_valid(indices, 10)

            self.lidar_front = sector(0.0, 50.0)
            self.lidar_left = sector(90.0, 70.0)
            self.lidar_right = sector(-90.0, 70.0)

            now = self.get_clock().now().nanoseconds / 1e9
            front = self.lidar_front
            if front is not None:
                if self.previous_front is not None and self.previous_front_time is not None:
                    dt = max(now - self.previous_front_time, 1e-3)
                    raw_closing = (self.previous_front - front) / dt
                    self.closing_speed = 0.65 * self.closing_speed + 0.35 * raw_closing
                self.previous_front = front
                self.previous_front_time = now
            self.last_lidar_time = now
        except Exception as exc:
            self.get_logger().warn(
                f'LiDAR processing failed: {exc}',
                throttle_duration_sec=2.0,
            )

    def fused_clearance(self, camera_value, lidar_value):
        now = self.get_clock().now().nanoseconds / 1e9
        values = []
        if now - self.last_camera_time < SENSOR_TIMEOUT and camera_value is not None:
            values.append(camera_value)
        if now - self.last_lidar_time < SENSOR_TIMEOUT and lidar_value is not None:
            values.append(lidar_value)
        if not values:
            return None
        # For collision avoidance use the conservative minimum.  A median or
        # weighted mean is better for mapping, but not for a safety override.
        return min(values)

    def lookahead_point(self):
        if not self.path:
            return None
        # First advance past points the drone is already at.
        while self.path_index < len(self.path) - 1:
            px, py, _ = self.path[self.path_index]
            if math.hypot(px - self.current_x, py - self.current_y) > 0.55:
                break
            self.path_index += 1

        px, py = self.current_x, self.current_y
        remaining = LOOKAHEAD
        for i in range(self.path_index, len(self.path)):
            wx, wy, _ = self.path[i]
            seg = math.hypot(wx - px, wy - py)
            if seg >= remaining and seg > 1e-6:
                t = remaining / seg
                return px + t * (wx - px), py + t * (wy - py), self.path[i][2]
            remaining -= seg
            px, py = wx, wy
        return self.path[-1]

    def control_loop(self):
        if not self.have_odom:
            return

        front = self.fused_clearance(self.camera_front, self.lidar_front)
        left = self.fused_clearance(self.camera_left, self.lidar_left)
        right = self.fused_clearance(self.camera_right, self.lidar_right)

        # Sensor fail-safe: no trustworthy near-field data means hold rather
        # than flying blindly.  Once either sensor is fresh, navigation resumes.
        now = self.get_clock().now().nanoseconds / 1e9
        camera_fresh = (now - self.last_camera_time) < SENSOR_TIMEOUT
        lidar_fresh = (now - self.last_lidar_time) < SENSOR_TIMEOUT
        sensor_ok = camera_fresh or lidar_fresh

        look = self.lookahead_point()
        if look is None:
            self._publish_zero('NO_PATH')
            return

        lx, ly, lz = look
        dist_to_goal = math.hypot(self.goal[0] - self.current_x,
                                  self.goal[1] - self.current_y) if self.goal else math.inf
        if dist_to_goal <= GOAL_TOLERANCE:
            self._publish_zero('GOAL_REACHED')
            return

        dx = lx - self.current_x
        dy = ly - self.current_y
        horiz = math.hypot(dx, dy)
        if horiz < 1e-3:
            self._publish_zero('HOLD')
            return

        dir_x = dx / horiz
        dir_y = dy / horiz

        # Cruise speed drops as frontal clearance falls or as the obstacle is
        # closing rapidly.  This is independent of A* replanning latency.
        speed = MAX_HORIZONTAL_SPEED
        if front is not None:
            if front <= EMERGENCY_CLEARANCE:
                speed = 0.0
            elif front < STOP_CLEARANCE:
                speed *= max(0.0, (front - EMERGENCY_CLEARANCE) /
                             (STOP_CLEARANCE - EMERGENCY_CLEARANCE))
            elif front < SLOW_CLEARANCE:
                speed *= 0.35 + 0.65 * (
                    (front - STOP_CLEARANCE) /
                    (SLOW_CLEARANCE - STOP_CLEARANCE)
                )

            closing = max(0.0, self.closing_speed)
            if closing > 0.2:
                speed *= max(0.25, 1.0 - min(0.75, 0.20 * closing))

        # World-frame nominal velocity toward the lookahead target.
        vx_world = speed * dir_x
        vy_world = speed * dir_y

        # Transform world velocity into body frame (+X forward, +Y left).
        cy = math.cos(self.current_yaw)
        sy = math.sin(self.current_yaw)
        vx_body = cy * vx_world + sy * vy_world
        vy_body = -sy * vx_world + cy * vy_world

        # Reactive side repulsion.  If the left wall is closer, vy should be
        # negative (move right); if the right wall is closer, vy positive.
        if left is not None and right is not None:
            imbalance = left - right
            if min(left, right) < 1.8:
                vy_body += max(-0.85, min(0.85, 0.75 * imbalance))
        elif left is not None and left < 1.2:
            vy_body -= min(0.9, 1.2 * (1.2 - left))
        elif right is not None and right < 1.2:
            vy_body += min(0.9, 1.2 * (1.2 - right))

        # Hard front safety override.  Choose the more open side and begin a
        # reverse motion only when the forward corridor is critically close.
        if front is not None and front < STOP_CLEARANCE:
            if left is not None and right is not None:
                preferred_sign = 1.0 if left > right else -1.0
                vy_body = preferred_sign * min(1.0, max(0.35, abs(vy_body)))
            vx_body = min(vx_body, 0.0)

        if front is not None and front < EMERGENCY_CLEARANCE:
            vx_body = -0.45
            if left is not None and right is not None:
                vy_body = 1.0 if left > right else -1.0
            else:
                vy_body = 0.7

        # If sensors are stale, stop.  Do not keep replaying the previous
        # command after a perception failure.
        if not sensor_ok:
            self._publish_zero('SENSOR_STALE')
            return

        # Limit horizontal command magnitude to the configured simulator cap.
        hspeed = math.hypot(vx_body, vy_body)
        if hspeed > MAX_HORIZONTAL_SPEED:
            scale = MAX_HORIZONTAL_SPEED / hspeed
            vx_body *= scale
            vy_body *= scale

        # Yaw points the body toward the desired world direction.  This makes
        # the velocity command stable even while the drone turns in the maze.
        desired_yaw = math.atan2(dir_y, dir_x)
        yaw_error = wrap_angle(desired_yaw - self.current_yaw)
        yaw_rate = max(-0.7, min(0.7, 1.2 * yaw_error))
        if front is not None and front < 1.4:
            if left is not None and right is not None:
                open_sign = 1.0 if left > right else -1.0
                yaw_rate += 0.4 * open_sign
                yaw_rate = max(-1.0, min(1.0, yaw_rate))

        z_error = (self.goal[2] if self.goal else self.current_z) - self.current_z
        vz = max(-MAX_VERTICAL_SPEED, min(MAX_VERTICAL_SPEED, 0.8 * z_error))

        cmd = Twist()
        cmd.linear.x = float(vx_body)
        cmd.linear.y = float(vy_body)
        cmd.linear.z = float(vz)
        cmd.angular.z = float(yaw_rate)
        self.cmd_pub.publish(cmd)

        status = String()
        status.data = (
            f'front={front if front is not None else float("nan"):.2f} '
            f'left={left if left is not None else float("nan"):.2f} '
            f'right={right if right is not None else float("nan"):.2f} '
            f'closing={self.closing_speed:.2f} '
            f'cmd=({vx_body:.2f},{vy_body:.2f},{vz:.2f}) '
            f'yaw_rate={yaw_rate:.2f}'
        )
        self.status_pub.publish(status)
        self._publish_debug(front, left, right, vx_body, vy_body)

    def _publish_zero(self, reason):
        self.cmd_pub.publish(Twist())
        status = String()
        status.data = f'{reason}: front={self.camera_front} lidar={self.lidar_front}'
        self.status_pub.publish(status)

    def _publish_debug(self, front, left, right, vx_body, vy_body):
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'quadcopter/base_link'
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        values = [('front', front), ('left', left), ('right', right)]
        for i, (name, distance) in enumerate(values):
            if distance is None:
                continue
            marker = Marker()
            marker.header.frame_id = 'quadcopter/base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'clearance'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            if name == 'front':
                marker.pose.position.x = distance
                marker.pose.position.y = 0.0
            elif name == 'left':
                marker.pose.position.x = 0.5
                marker.pose.position.y = distance
            else:
                marker.pose.position.x = 0.5
                marker.pose.position.y = -distance
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.16
            marker.scale.y = 0.16
            marker.scale.z = 0.16
            marker.color.g = 1.0
            marker.color.a = 0.8
            markers.markers.append(marker)

        arrow = Marker()
        arrow.header.frame_id = 'quadcopter/base_link'
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = 'avoidance_vector'
        arrow.id = 10
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = 0.08
        arrow.scale.y = 0.14
        arrow.scale.z = 0.14
        arrow.color.b = 1.0
        arrow.color.a = 1.0
        from geometry_msgs.msg import Point
        arrow.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=float(vx_body), y=float(vy_body), z=0.0),
        ]
        markers.markers.append(arrow)

        self.debug_pub.publish(markers)


def main():
    rclpy.init()
    node = SmartNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
