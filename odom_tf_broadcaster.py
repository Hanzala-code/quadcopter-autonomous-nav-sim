#!/usr/bin/env python3
"""Time-synchronised TF broadcaster from Gazebo odometry.

This replaces the previous zero-stamped GPS TF workaround.  The Gazebo
odometry already carries the drone orientation, so TF can now represent the
actual yaw that the body-frame velocity controller is using.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry,
            '/quadcopter/odometry',
            self.callback,
            20,
        )
        self.get_logger().info(
            'Broadcasting world -> quadcopter/base_link from Gazebo odometry.'
        )

    def callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'quadcopter/base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
