#!/usr/bin/env python3
"""Optional moving obstacle driver for maze_world testing.

Gazebo Sim's UserCommands system exposes /world/<world>/set_pose, so an
external process can move a test box without adding another simulator plugin.
This node deliberately stays optional and is disabled by default in launch.
"""

import math
import subprocess

import rclpy
from rclpy.node import Node


class DynamicObstacleDemo(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_demo')
        self.declare_parameters('', [
            ('world_name', 'quad_world'),
            ('model_name', 'dynamic_test_box'),
            ('x_center', -2.0),
            ('y_center', -4.8),
            ('x_amplitude', 2.5),
            ('speed', 0.6),
            ('z', 1.1),
        ])
        self.world = self.get_parameter('world_name').value
        self.model = self.get_parameter('model_name').value
        self.x_center = float(self.get_parameter('x_center').value)
        self.y_center = float(self.get_parameter('y_center').value)
        self.amplitude = float(self.get_parameter('x_amplitude').value)
        self.speed = float(self.get_parameter('speed').value)
        self.z = float(self.get_parameter('z').value)
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.timer = self.create_timer(0.15, self.update)
        self.get_logger().info('Dynamic obstacle demo enabled.')

    def update(self):
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.start_time
        x = self.x_center + self.amplitude * math.sin(self.speed * t)
        req = (
            f'name: "{self.model}" '
            f'position: {{x: {x:.3f}, y: {self.y_center:.3f}, z: {self.z:.3f}}}'
        )
        cmd = [
            'gz', 'service',
            '-s', f'/world/{self.world}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '100',
            '--req', req,
        ]
        try:
            subprocess.run(
                cmd,
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=0.25,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired):
            self.get_logger().warn(
                'Could not call gz set_pose; disabling demo timer.',
                throttle_duration_sec=5.0,
            )
            self.timer.cancel()


def main():
    rclpy.init()
    node = DynamicObstacleDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
