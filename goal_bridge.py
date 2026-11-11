#!/usr/bin/env python3
"""Bridge RViz 2D Goal Pose messages into the Gazebo target topic."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped


class GoalBridge(Node):
    def __init__(self):
        super().__init__('goal_bridge')
        self.declare_parameter('default_altitude', 2.0)
        self.default_altitude = float(self.get_parameter('default_altitude').value)
        self.pub = self.create_publisher(Pose, '/quadcopter/target_position', 10)
        self.sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.callback, 10
        )
        self.get_logger().info(
            'RViz goal bridge ready: /goal_pose -> /quadcopter/target_position'
        )

    def callback(self, msg):
        pose = Pose()
        pose.position = msg.pose.position
        pose.orientation = msg.pose.orientation
        self.pub.publish(pose)
        self.get_logger().info(
            f'Goal bridged: ({pose.position.x:.2f}, '
            f'{pose.position.y:.2f}, {pose.position.z:.2f})'
        )


def main():
    rclpy.init()
    node = GoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
