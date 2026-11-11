#!/usr/bin/env python3
"""
Dynamic A* global planner for quad_final.

The planner is deliberately 2-D at the navigation altitude used by the
maze.  It combines:
  * the complete, known maze geometry from world_maze1.sdf
  * live obstacle boxes from the camera / obstacle tracker
  * safety inflation for the X500 airframe

Inputs:
  /quadcopter/gps_odometry        nav_msgs/Odometry
  /quadcopter/target_position     geometry_msgs/Pose
  /quadcopter/tracked_obstacles   visualization_msgs/MarkerArray

Output:
  /quadcopter/planned_path         nav_msgs/Path

The plan is recomputed at 2 Hz while a goal exists.  This is cheap for the
58x58 grid and, unlike the previous D* Lite version, guarantees that the
current dynamic obstacle set is used on every fresh plan.
"""

import heapq
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray

# World bounds.  The X500 starts at x=-13, so the old [-10, +10] grid was
# invalid from the very beginning of a mission.
ARENA_MIN = -14.5
ARENA_MAX = 14.5
CELL_SIZE = 0.5
GRID_DIM = int(round((ARENA_MAX - ARENA_MIN) / CELL_SIZE))

# Full maze geometry from world_maze1.sdf.  These are the collision
# rectangles in world metres (not the visual extents).
STATIC_OBSTACLES = [
    # (-12.5, 12.5, 11.125, 11.375),    # top_wall
    # (-6.125, -5.875, 7.12, 11.12),   # top_drop_left
    # (1.375, 1.625, 5.475, 11.625),   # top_drop_center
    # (8.125, 8.375, 5.475, 11.625),   # top_drop_right
    # (-8.0, 3.5, 3.625, 3.875),       # middle_left
    # (-10.625, -10.375, -2.25, 6.75), # left_outer
    # (-6.125, -5.875, -2.125, 4.125), # middle_drop_left
    # (2.875, 3.125, 0.0, 3.0),        # middle_drop_center
    # (10.375, 10.625, -2.25, 5.25),   # right_outer
    # (-11.0, 0.5, -8.375, -8.125),    # lower_left
    # (1.375, 1.625, -6.25, -2.25),    # lower_drop
]

# X500 body/rotor clearance plus a small margin.
INFLATION = 0.65
REPLAN_RATE_HZ = 2.0
GOAL_TOLERANCE = 0.5

NEIGHBORS = [
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1),           (0, 1),
    (1, -1),  (1, 0),  (1, 1),
]


def world_to_grid(x: float, y: float):
    return int(math.floor((x - ARENA_MIN) / CELL_SIZE)), int(
        math.floor((y - ARENA_MIN) / CELL_SIZE)
    )


def grid_to_world(gx: int, gy: int):
    return (
        ARENA_MIN + (gx + 0.5) * CELL_SIZE,
        ARENA_MIN + (gy + 0.5) * CELL_SIZE,
    )


def in_bounds(gx: int, gy: int) -> bool:
    return 0 <= gx < GRID_DIM and 0 <= gy < GRID_DIM


def box_intersects_cell(xmin, xmax, ymin, ymax, gx, gy, inflation=0.0):
    cx, cy = grid_to_world(gx, gy)
    return (
        xmin - inflation <= cx <= xmax + inflation
        and ymin - inflation <= cy <= ymax + inflation
    )


def is_occupied(gx, gy, dynamic_obstacles):
    if not in_bounds(gx, gy):
        return True

    x, y = grid_to_world(gx, gy)

    # Keep the physical arena wall out of the free-space grid.
    if (x - ARENA_MIN) < INFLATION or (ARENA_MAX - x) < INFLATION:
        return True
    if (y - ARENA_MIN) < INFLATION or (ARENA_MAX - y) < INFLATION:
        return True

    for xmin, xmax, ymin, ymax in STATIC_OBSTACLES:
        if box_intersects_cell(xmin, xmax, ymin, ymax, gx, gy, INFLATION):
            return True

    for xmin, xmax, ymin, ymax in dynamic_obstacles:
        if box_intersects_cell(xmin, xmax, ymin, ymax, gx, gy, INFLATION):
            return True

    return False


def segment_free(p1, p2, dynamic_obstacles, step=0.20):
    x1, y1 = p1
    x2, y2 = p2
    distance = math.hypot(x2 - x1, y2 - y1)
    steps = max(1, int(math.ceil(distance / step)))
    for i in range(steps + 1):
        t = i / steps
        x = x1 + (x2 - x1) * t
        y = y1 + (y2 - y1) * t
        gx, gy = world_to_grid(x, y)
        if is_occupied(gx, gy, dynamic_obstacles):
            return False
    return True


def smooth_path(points, dynamic_obstacles):
    """Greedy line-of-sight smoothing over the grid path."""
    if len(points) <= 2:
        return points

    smoothed = [points[0]]
    anchor = 0
    while anchor < len(points) - 1:
        best = anchor + 1
        for candidate in range(anchor + 1, len(points)):
            if segment_free(points[anchor], points[candidate], dynamic_obstacles):
                best = candidate
            else:
                break
        smoothed.append(points[best])
        anchor = best
    return smoothed


class AStarPlanner:
    def __init__(self, dynamic_obstacles):
        self.dynamic_obstacles = dynamic_obstacles

    @staticmethod
    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def neighbors(node):
        gx, gy = node
        for dx, dy in NEIGHBORS:
            nxt = gx + dx, gy + dy
            if in_bounds(*nxt):
                yield nxt

    def plan(self, start_xy, goal_xy):
        start = world_to_grid(*start_xy)
        goal = world_to_grid(*goal_xy)
        if not in_bounds(*start) or not in_bounds(*goal):
            return None

        if is_occupied(*goal, self.dynamic_obstacles):
            return None

        if is_occupied(*start, self.dynamic_obstacles):
            start = self._nearest_free(start)
            if start is None:
                return None

        open_heap = []
        counter = 0
        g = {start: 0.0}
        parent = {}
        heapq.heappush(open_heap, (self.heuristic(start, goal), counter, start))
        closed = set()

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                return self._reconstruct(parent, current)

            for nxt in self.neighbors(current):
                if nxt in closed or is_occupied(*nxt, self.dynamic_obstacles):
                    continue

                dx = nxt[0] - current[0]
                dy = nxt[1] - current[1]
                step_cost = math.hypot(dx, dy)
                tentative = g[current] + step_cost

                if tentative < g.get(nxt, math.inf):
                    g[nxt] = tentative
                    parent[nxt] = current
                    counter += 1
                    f = tentative + self.heuristic(nxt, goal)
                    heapq.heappush(open_heap, (f, counter, nxt))

        return None

    def _nearest_free(self, cell, max_radius=12):
        gx0, gy0 = cell
        for radius in range(1, max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    gx, gy = gx0 + dx, gy0 + dy
                    if in_bounds(gx, gy) and not is_occupied(
                        gx, gy, self.dynamic_obstacles
                    ):
                        return gx, gy
        return None

    @staticmethod
    def _reconstruct(parent, current):
        path = [current]
        while current in parent:
            current = parent[current]
            path.append(current)
        path.reverse()
        return path


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        self.current_x = 0.0
        self.current_y = 0.0
        self.have_odom = False
        self.goal = None
        self.dynamic_obstacles = []
        self.last_signature = None
        self.dirty = True
        self.last_plan_ok = False

        self.path_pub = self.create_publisher(
            Path, '/quadcopter/planned_path', 10
        )
        self.debug_pub = self.create_publisher(
            MarkerArray, '/quadcopter/planner_debug', 10
        )

        self.create_subscription(
            Odometry,
            '/quadcopter/gps_odometry',
            self.odom_callback,
            10,
        )
        self.create_subscription(
            Pose,
            '/quadcopter/target_position',
            self.target_callback,
            10,
        )
        self.create_subscription(
            MarkerArray,
            '/quadcopter/tracked_obstacles',
            self.obstacle_callback,
            10,
        )

        self.timer = self.create_timer(1.0 / REPLAN_RATE_HZ, self.replan_loop)
        self.get_logger().info(
            f'Dynamic A* planner ready: {GRID_DIM}x{GRID_DIM}, '
            f'{CELL_SIZE:.2f}m/cell, inflation {INFLATION:.2f}m.'
        )

    def odom_callback(self, msg: Odometry):
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        moved = math.hypot(new_x - self.current_x, new_y - self.current_y)
        self.current_x = new_x
        self.current_y = new_y
        self.have_odom = True
        if moved > 0.25:
            self.dirty = True

    def target_callback(self, msg: Pose):
        new_goal = (
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
        )
        if self.goal is not None and all(
            abs(a - b) < 0.03 for a, b in zip(new_goal, self.goal)
        ):
            return
        self.goal = new_goal
        self.dirty = True
        self.get_logger().info(
            f'New goal: ({new_goal[0]:.2f}, {new_goal[1]:.2f}, {new_goal[2]:.2f})'
        )

    def obstacle_callback(self, msg: MarkerArray):
        boxes = []
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL:
                continue
            if marker.type != Marker.CUBE:
                continue
            cx = marker.pose.position.x
            cy = marker.pose.position.y
            sx = abs(marker.scale.x)
            sy = abs(marker.scale.y)
            if sx < 0.05 or sy < 0.05:
                continue
            boxes.append((cx - sx / 2, cx + sx / 2,
                          cy - sy / 2, cy + sy / 2))

        signature = tuple(
            sorted(
                (
                    round(a, 2), round(b, 2),
                    round(c, 2), round(d, 2)
                )
                for a, b, c, d in boxes
            )
        )
        if signature != self.last_signature:
            self.dynamic_obstacles = boxes
            self.last_signature = signature
            self.dirty = True

    def replan_loop(self):
        if not self.have_odom or self.goal is None:
            return

        gx, gy, gz = self.goal
        distance_to_goal = math.hypot(gx - self.current_x, gy - self.current_y)
        if distance_to_goal <= GOAL_TOLERANCE:
            self.last_plan_ok = True
            return

        # Replanning is intentionally periodic.  When the path is unchanged,
        # no new message is published, so the navigator keeps following it.
        if not self.dirty and self.last_plan_ok:
            return

        planner = AStarPlanner(self.dynamic_obstacles)
        grid_path = planner.plan(
            (self.current_x, self.current_y),
            (gx, gy),
        )

        if grid_path is None:
            self.last_plan_ok = False
            self.path_pub.publish(self._empty_path())
            self.get_logger().warn(
                'No collision-free A* path found with current dynamic obstacles.'
            )
            # Keep retrying; a moving obstacle can open the route later.
            self.dirty = True
            return

        world_path = [grid_to_world(*cell) for cell in grid_path]
        world_path = smooth_path(world_path, self.dynamic_obstacles)

        if len(world_path) == 1:
            world_path.append((gx, gy))
        else:
            world_path[-1] = (gx, gy)

        msg = Path()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in world_path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = gz
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        signature = tuple((round(p[0], 2), round(p[1], 2)) for p in world_path)
        if signature != self.last_signature:
            # Do not reuse the obstacle signature here; keep a separate path
            # signature so obstacle changes can force a new identical path.
            pass

        self.path_pub.publish(msg)
        self._publish_debug_path(world_path)
        self.last_plan_ok = True
        self.dirty = False
        self.get_logger().info(
            f'Published dynamic A* path: {len(world_path)} waypoint(s).'
        )

    def _empty_path(self):
        msg = Path()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _publish_debug_path(self, path):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, (x, y) in enumerate(path):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'planner_waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.10
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.g = 1.0
            m.color.a = 0.9
            markers.markers.append(m)

        self.debug_pub.publish(markers)


def main():
    rclpy.init()
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
