#!/usr/bin/env python3
"""
GPS -> local Odometry converter.

The NavSat sensor reports latitude/longitude/altitude — not the local
x/y/z meters our controller needs. This node converts one into the
other using a flat-Earth (equirectangular) approximation, which is
entirely accurate at the scale of meters (not kilometers).

IMPORTANT: REF_LAT / REF_LON below must exactly match the
<latitude_deg>/<longitude_deg> in the world file's <spherical_coordinates>
block — that's the point NavSat treats as (0, 0). If they don't match,
your converted x/y will be offset from where the drone actually is.

Subscribes:  /quadcopter/navsat        (sensor_msgs/NavSatFix)
Publishes:   /quadcopter/gps_odometry  (nav_msgs/Odometry)
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

# Gazebo Transport — publishing here (in addition to ROS2) is what makes
# this data visible inside the Gazebo GUI (e.g. via the TopicEcho panel).
# ROS2 and Gazebo Transport are separate networks, as covered earlier —
# without this, the converted position only exists on the ROS2 side.
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_pb2 import Pose as GzPose

# Must match <spherical_coordinates> in quad_world.sdf exactly
REF_LAT = 33.6844
REF_LON = 73.0479
REF_ELEVATION = 0.0

EARTH_RADIUS = 6378137.0  # meters (WGS84 equatorial radius)

GZ_TOPIC = '/quadcopter/gps_local_position'


class GpsOdomConverter(Node):
    def __init__(self):
        super().__init__('gps_odom_converter')

        self.sub = self.create_subscription(
            NavSatFix, '/quadcopter/navsat', self.navsat_callback, 10)
        self.pub = self.create_publisher(
            Odometry, '/quadcopter/gps_odometry', 10)

        # Gazebo Transport publisher — separate from the ROS2 publisher
        # above. This is what a Gazebo GUI panel (TopicEcho) can see.
        self.gz_node = GzNode()
        self.gz_pub = self.gz_node.advertise(GZ_TOPIC, GzPose)

        self.get_logger().info(
            f'Converting GPS fixes to local odometry (reference: {REF_LAT}, {REF_LON}). '
            f'Publishing to ROS2 /quadcopter/gps_odometry AND Gazebo topic {GZ_TOPIC}.'
        )

    def navsat_callback(self, msg: NavSatFix):
        # Equirectangular approximation — accurate for small areas (meters
        # to a few km), which is exactly our use case here.
        x = math.radians(msg.longitude - REF_LON) * EARTH_RADIUS * math.cos(math.radians(REF_LAT))
        y = math.radians(msg.latitude - REF_LAT) * EARTH_RADIUS
        z = msg.altitude - REF_ELEVATION

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'quadcopter/gps_odom'
        odom.child_frame_id = 'quadcopter/base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z
        # Orientation not derived from GPS alone — left as identity.
        # (A real system would fuse this with IMU/magnetometer for heading.)
        odom.pose.pose.orientation.w = 1.0

        self.pub.publish(odom)

        # Also publish to Gazebo Transport, so it's visible in the GUI
        gz_msg = GzPose()
        gz_msg.position.x = x
        gz_msg.position.y = y
        gz_msg.position.z = z
        gz_msg.orientation.w = 1.0
        self.gz_pub.publish(gz_msg)


def main():
    rclpy.init()
    node = GpsOdomConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
