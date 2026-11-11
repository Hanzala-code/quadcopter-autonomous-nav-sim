#!/usr/bin/env python3
"""
Temporal obstacle tracker / short-horizon predictor.

Receives confirmed camera detections as 3-D boxes and publishes predicted
2-D boxes for the global planner.  A matched obstacle gets a velocity estimate
from successive centroids.  The prediction horizon is deliberately short so
that a noisy estimate cannot create a giant virtual obstacle.
"""

import math

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

MATCH_DISTANCE = 1.75
ALPHA_VELOCITY = 0.55
PREDICTION_HORIZON = 0.80
MAX_TRACK_SPEED = 4.0
MAX_MISSED = 5


class Track:
    def __init__(self, track_id, box, now):
        self.track_id = track_id
        self.box = box
        self.last_time = now
        self.vx = 0.0
        self.vy = 0.0
        self.missed = 0

    @property
    def center(self):
        xmin, xmax, ymin, ymax, _, _ = self.box
        return 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)


class ObstacleTracker(Node):
    def __init__(self):
        super().__init__('obstacle_tracker')

        self.tracks = {}
        self.next_id = 1

        self.pub = self.create_publisher(
            MarkerArray, '/quadcopter/tracked_obstacles', 10
        )
        self.viz_pub = self.create_publisher(
            MarkerArray, '/quadcopter/obstacle_tracks_debug', 10
        )
        self.sub = self.create_subscription(
            MarkerArray,
            '/quadcopter/detected_obstacles',
            self.callback,
            10,
        )

        self.get_logger().info(
            f'Obstacle tracker ready (prediction={PREDICTION_HORIZON:.2f}s).'
        )

    def callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        boxes = []
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL or marker.type != Marker.CUBE:
                continue
            cx = marker.pose.position.x
            cy = marker.pose.position.y
            cz = marker.pose.position.z
            boxes.append((
                cx - marker.scale.x / 2,
                cx + marker.scale.x / 2,
                cy - marker.scale.y / 2,
                cy + marker.scale.y / 2,
                cz - marker.scale.z / 2,
                cz + marker.scale.z / 2,
            ))

        unmatched_tracks = set(self.tracks)
        unmatched_boxes = set(range(len(boxes)))

        # Greedy nearest-neighbour association.  The camera rate is high
        # relative to plausible obstacle speeds, so a gated greedy match is
        # sufficient and avoids adding SciPy as a runtime dependency.
        candidates = []
        for tid, track in self.tracks.items():
            tx, ty = track.center
            for idx, box in enumerate(boxes):
                bx = 0.5 * (box[0] + box[1])
                by = 0.5 * (box[2] + box[3])
                d = math.hypot(bx - tx, by - ty)
                if d <= MATCH_DISTANCE:
                    candidates.append((d, tid, idx))
        candidates.sort()

        for _, tid, idx in candidates:
            if tid not in unmatched_tracks or idx not in unmatched_boxes:
                continue
            track = self.tracks[tid]
            box = boxes[idx]
            bx = 0.5 * (box[0] + box[1])
            by = 0.5 * (box[2] + box[3])
            dt = max(now - track.last_time, 1e-3)
            raw_vx = (bx - track.center[0]) / dt
            raw_vy = (by - track.center[1]) / dt
            raw_speed = math.hypot(raw_vx, raw_vy)
            if raw_speed > MAX_TRACK_SPEED:
                scale = MAX_TRACK_SPEED / raw_speed
                raw_vx *= scale
                raw_vy *= scale

            track.vx = (1 - ALPHA_VELOCITY) * track.vx + ALPHA_VELOCITY * raw_vx
            track.vy = (1 - ALPHA_VELOCITY) * track.vy + ALPHA_VELOCITY * raw_vy
            track.box = box
            track.last_time = now
            track.missed = 0
            unmatched_tracks.remove(tid)
            unmatched_boxes.remove(idx)

        for idx in sorted(unmatched_boxes):
            self.tracks[self.next_id] = Track(self.next_id, boxes[idx], now)
            self.next_id += 1

        for tid in list(unmatched_tracks):
            self.tracks[tid].missed += 1
            if self.tracks[tid].missed > MAX_MISSED:
                del self.tracks[tid]

        self.publish_predictions(now)

    def publish_predictions(self, now):
        out = MarkerArray()
        dbg = MarkerArray()

        clear = Marker()
        clear.header.frame_id = 'world'
        clear.action = Marker.DELETEALL
        out.markers.append(clear)
        clear_dbg = Marker()
        clear_dbg.header.frame_id = 'world'
        clear_dbg.action = Marker.DELETEALL
        dbg.markers.append(clear_dbg)

        for tid, track in self.tracks.items():
            if track.missed > 1:
                continue

            xmin, xmax, ymin, ymax, zmin, zmax = track.box
            cx, cy = track.center
            px = cx + track.vx * PREDICTION_HORIZON
            py = cy + track.vy * PREDICTION_HORIZON

            # Prediction inflation: moving targets get a little additional
            # horizontal margin proportional to predicted travel.
            travel = math.hypot(
                track.vx * PREDICTION_HORIZON,
                track.vy * PREDICTION_HORIZON,
            )
            margin = min(0.60, 0.18 + 0.20 * travel)

            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'predicted_dynamic_obstacles'
            marker.id = tid
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = px
            marker.pose.position.y = py
            marker.pose.position.z = 0.5 * (zmin + zmax)
            marker.pose.orientation.w = 1.0
            marker.scale.x = max(xmax - xmin + 2 * margin, 0.1)
            marker.scale.y = max(ymax - ymin + 2 * margin, 0.1)
            marker.scale.z = max(zmax - zmin, 0.1)
            marker.color.r = 1.0
            marker.color.g = 0.05
            marker.color.b = 0.05
            marker.color.a = 0.45
            out.markers.append(marker)

            # Velocity vector for RViz debugging.
            arrow = Marker()
            arrow.header.frame_id = 'world'
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'obstacle_velocity'
            arrow.id = tid
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.scale.x = 0.06
            arrow.scale.y = 0.12
            arrow.scale.z = 0.12
            arrow.color.g = 1.0
            arrow.color.a = 0.95
            arrow.points = []
            from geometry_msgs.msg import Point
            p0 = Point(x=cx, y=cy, z=marker.pose.position.z + 0.35)
            p1 = Point(
                x=cx + track.vx * 0.5,
                y=cy + track.vy * 0.5,
                z=marker.pose.position.z + 0.35,
            )
            arrow.points = [p0, p1]
            dbg.markers.append(arrow)

        self.pub.publish(out)
        self.viz_pub.publish(dbg)


def main():
    rclpy.init()
    node = ObstacleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
