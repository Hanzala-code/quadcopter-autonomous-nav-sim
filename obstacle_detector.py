#!/usr/bin/env python3
"""
Camera point-cloud obstacle detector.

The previous detector waited for three matching voxel keys but did not
provide any notion of obstacle motion.  This node now focuses on robust
3-D clustering and leaves temporal tracking/prediction to obstacle_tracker.py.
"""

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException

VOXEL_SIZE = 0.35
MIN_POINTS_PER_VOXEL = 3
MIN_VOXELS_PER_CLUSTER = 3
GROUND_CLEARANCE = 0.12
CEILING_HEIGHT = 8.0
ARENA_MIN = -14.5
ARENA_MAX = 14.5
MIN_OBSTACLE_FOOTPRINT = 0.20
CONFIRM_FRAMES = 2
DECAY_FRAMES = 6

NEIGHBOR_OFFSETS = [
    (dx, dy, dz)
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    for dz in (-1, 0, 1)
    if (dx, dy, dz) != (0, 0, 0)
]


def apply_transform(points, transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math_sqrt = (x*x + y*y + z*z + w*w) ** 0.5
    if not (0.95 < norm < 1.05):
        raise ValueError(f'invalid quaternion norm {norm}')

    # Normalize once so small simulator / bridge drift cannot accumulate.
    x /= norm
    y /= norm
    z /= norm
    w /= norm
    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)
    out = (R @ points.T).T
    out[:, 0] += t.x
    out[:, 1] += t.y
    out[:, 2] += t.z
    return out


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            MarkerArray, '/quadcopter/detected_obstacles', 10
        )
        self.sub = self.create_subscription(
            PointCloud2,
            '/quadcopter/camera/points',
            self.cloud_callback,
            5,
        )
        self.confirm = {}
        self.last_stamp = None

        self.get_logger().info('Camera obstacle detector ready.')

    def cloud_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.08),
            )
        except TransformException:
            try:
                transform = self.tf_buffer.lookup_transform(
                    'world', msg.header.frame_id, Time(),
                    timeout=Duration(seconds=0.03),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f'Camera TF unavailable: {exc}',
                    throttle_duration_sec=2.0,
                )
                return

        try:
            if hasattr(pc2, 'read_points_numpy'):
                raw = pc2.read_points_numpy(
                    msg, field_names=['x', 'y', 'z'], skip_nans=True
                )
                raw = np.asarray(raw, dtype=np.float64).reshape(-1, 3)
            else:
                raw = np.asarray(
                    list(pc2.read_points(
                        msg,
                        field_names=('x', 'y', 'z'),
                        skip_nans=True,
                    )),
                    dtype=np.float64,
                ).reshape(-1, 3)
        except Exception as exc:
            self.get_logger().warn(
                f'Point-cloud decode failed: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        if raw.size == 0:
            self._publish_confirmed({})
            return

        mask = np.all(np.isfinite(raw), axis=1)
        pts = raw[mask]

        # The SDF camera follows Gazebo's +X-forward model convention.
        # Limit work to the useful forward volume; world clipping is done
        # after TF conversion.
        cam_mask = (
            (pts[:, 0] > 0.15)
            & (pts[:, 0] < 10.0)
            & (np.abs(pts[:, 1]) < 5.0)
            & (np.abs(pts[:, 2]) < 5.0)
        )
        pts = pts[cam_mask]
        if pts.size == 0:
            self._publish_confirmed({})
            return

        try:
            pts_world = apply_transform(pts, transform)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=2.0)
            return

        world_mask = (
            (pts_world[:, 2] > GROUND_CLEARANCE)
            & (pts_world[:, 2] < CEILING_HEIGHT)
            & (pts_world[:, 0] > ARENA_MIN)
            & (pts_world[:, 0] < ARENA_MAX)
            & (pts_world[:, 1] > ARENA_MIN)
            & (pts_world[:, 1] < ARENA_MAX)
        )
        pts_world = pts_world[world_mask]
        if pts_world.size == 0:
            self._publish_confirmed({})
            return

        clusters = self.cluster_points(pts_world)
        keyed = {self.key_for_cluster(c): c for c in clusters}
        self._publish_confirmed(keyed)

    @staticmethod
    def key_for_cluster(bb):
        cx = 0.5 * (bb[0] + bb[1])
        cy = 0.5 * (bb[2] + bb[3])
        return round(cx / VOXEL_SIZE), round(cy / VOXEL_SIZE)

    @staticmethod
    def cluster_points(points):
        voxel_index = np.floor(points / VOXEL_SIZE).astype(np.int32)
        unique, inverse, counts = np.unique(
            voxel_index, axis=0, return_inverse=True, return_counts=True
        )

        keep = counts >= MIN_POINTS_PER_VOXEL
        occupied = {tuple(v) for v in unique[keep]}
        if not occupied:
            return []

        voxel_points = {
            tuple(unique[i]): np.where(inverse == i)[0]
            for i in np.where(keep)[0]
        }

        visited = set()
        clusters = []
        for voxel in occupied:
            if voxel in visited:
                continue
            queue = deque([voxel])
            visited.add(voxel)
            component = []
            while queue:
                cur = queue.popleft()
                component.append(cur)
                for dx, dy, dz in NEIGHBOR_OFFSETS:
                    nb = (cur[0] + dx, cur[1] + dy, cur[2] + dz)
                    if nb in occupied and nb not in visited:
                        visited.add(nb)
                        queue.append(nb)

            if len(component) < MIN_VOXELS_PER_CLUSTER:
                continue

            idx = np.concatenate([voxel_points[v] for v in component])
            cluster = points[idx]
            xmin, ymin, zmin = cluster.min(axis=0)
            xmax, ymax, zmax = cluster.max(axis=0)
            if ((xmax - xmin) < MIN_OBSTACLE_FOOTPRINT
                    and (ymax - ymin) < MIN_OBSTACLE_FOOTPRINT):
                continue

            clusters.append((
                float(xmin), float(xmax),
                float(ymin), float(ymax),
                float(zmin), float(zmax),
            ))
        return clusters

    def _publish_confirmed(self, current):
        next_confirm = {}
        for key in set(self.confirm) | set(current):
            if key in current:
                next_confirm[key] = min(self.confirm.get(key, 0) + 1, CONFIRM_FRAMES + 3)
            else:
                count = self.confirm.get(key, 0) - 1
                if count >= -DECAY_FRAMES:
                    next_confirm[key] = count
        self.confirm = next_confirm

        confirmed = [
            current[key]
            for key, count in self.confirm.items()
            if count >= CONFIRM_FRAMES and key in current
        ]
        self._publish_markers(confirmed)

    def _publish_markers(self, clusters):
        msg = MarkerArray()
        clear = Marker()
        clear.header.frame_id = 'world'
        clear.action = Marker.DELETEALL
        msg.markers.append(clear)

        for i, (xmin, xmax, ymin, ymax, zmin, zmax) in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'camera_obstacles'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = 0.5 * (xmin + xmax)
            marker.pose.position.y = 0.5 * (ymin + ymax)
            marker.pose.position.z = 0.5 * (zmin + zmax)
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(xmax - xmin, 0.08)
            marker.scale.y = max(ymax - ymin, 0.08)
            marker.scale.z = max(zmax - zmin, 0.08)
            marker.color.r = 1.0
            marker.color.g = 0.25
            marker.color.b = 0.1
            marker.color.a = 0.55
            msg.markers.append(marker)

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
