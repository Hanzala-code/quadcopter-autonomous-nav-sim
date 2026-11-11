#!/usr/bin/env python3
"""
Smooth 3D Keyboard Teleop for Quadcopter (ROS 2)
Publishes geometry_msgs/Twist on /quadcopter/cmd_vel

Features:
  - Smooth acceleration/deceleration ramping for realistic flight feel
  - Spacebar for emergency instant brake
  - Speed presets (1: Slow/Precision, 2: Normal, 3: Fast)
  - Press-and-hold timeout decay with clean raw-terminal management

Controls:
  w/s : forward / backward   (linear.x)
  a/d : left / right         (linear.y)
  q/e : up / down            (linear.z)
  z/c : yaw left / yaw right (angular.z)
  SPACE : Emergency Instant Stop
  1/2/3 : Speed multiplier (50%, 100%, 150%)
  x / Ctrl+C : Quit
"""

import sys
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Base Speed Settings
BASE_LINEAR_SPEED = 0.5    # m/s
BASE_VERTICAL_SPEED = 0.4  # m/s
BASE_YAW_SPEED = 0.6       # rad/s

# Acceleration Limits (units/sec^2) - Lower values = smoother/heavier drone feel
LIN_ACCEL = 1.2    # Linear acceleration limit (m/s^2)
YAW_ACCEL = 1.8    # Yaw acceleration limit (rad/s^2)

HOLD_TIMEOUT = 0.25    # Seconds before unpressed key target decays to zero
PUBLISH_RATE_HZ = 30.0 # Higher rate (30 Hz) yields smoother command interpolation

INSTRUCTIONS = """
Quadcopter Smooth Teleop (ROS 2)
-----------------------------------------------------------------------
   w              q : up           1 : Precision Speed (50%)
 a s d             e : down         2 : Normal Speed (100%)
              z : yaw left         3 : Fast Speed (150%)
              c : yaw right

w/s : forward/back     a/d : left/right     q/e : up/down     z/c : yaw
SPACE : Emergency Brake (Instant Stop)      x   : Quit
-----------------------------------------------------------------------
"""

KEY_MAP = {
    'w': ('target_vx', 1.0, 'linear'),
    's': ('target_vx', -1.0, 'linear'),
    'a': ('target_vy', 1.0, 'linear'),
    'd': ('target_vy', -1.0, 'linear'),
    'q': ('target_vz', 1.0, 'vertical'),
    'e': ('target_vz', -1.0, 'vertical'),
    'z': ('target_yaw', 1.0, 'yaw'),
    'c': ('target_yaw', -1.0, 'yaw'),
}


class SmoothQuadTeleop(Node):
    def __init__(self):
        super().__init__('smooth_quad_teleop')
        self.pub = self.create_publisher(Twist, '/quadcopter/cmd_vel', 10)

        # Speed Multiplier
        self.speed_scale = 1.0

        # Target velocities requested by key events
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0
        self.target_yaw = 0.0

        # Actual smoothed output velocities
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self.current_yaw = 0.0

        # Keep track of key activity
        now = time.monotonic()
        self.last_seen = {
            'target_vx': now,
            'target_vy': now,
            'target_vz': now,
            'target_yaw': now
        }

        # Save old terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)

    def get_key_nonblocking(self):
        """Read pending characters from non-blocking stdin."""
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if rlist:
            return sys.stdin.read(1)
        return ''

    def process_key(self, key):
        if not key:
            return True

        if key in ('x', '\x03'):  # 'x' or Ctrl+C
            return False

        # Instant Emergency Stop
        if key == ' ':
            self.target_vx = self.target_vy = self.target_vz = self.target_yaw = 0.0
            self.current_vx = self.current_vy = self.current_vz = self.current_yaw = 0.0
            return True

        # Speed Presets
        if key == '1':
            self.speed_scale = 0.5
            self.get_logger().info('Speed Mode: Precision (50%)')
            return True
        elif key == '2':
            self.speed_scale = 1.0
            self.get_logger().info('Speed Mode: Normal (100%)')
            return True
        elif key == '3':
            self.speed_scale = 1.5
            self.get_logger().info('Speed Mode: Fast (150%)')
            return True

        # Directional Movement Mapping
        if key in KEY_MAP:
            axis_attr, sign, axis_type = KEY_MAP[key]

            if axis_type == 'linear':
                max_speed = BASE_LINEAR_SPEED * self.speed_scale
            elif axis_type == 'vertical':
                max_speed = BASE_VERTICAL_SPEED * self.speed_scale
            else:
                max_speed = BASE_YAW_SPEED * self.speed_scale

            setattr(self, axis_attr, sign * max_speed)
            self.last_seen[axis_attr] = time.monotonic()

        return True

    def decay_stale_targets(self):
        """Zero out target speeds if keys are released past timeout."""
        now = time.monotonic()
        for axis in ('target_vx', 'target_vy', 'target_vz', 'target_yaw'):
            if now - self.last_seen[axis] > HOLD_TIMEOUT:
                setattr(self, axis, 0.0)

    def ramp_value(self, current, target, accel, dt):
        """Slew-rate limiter to smoothly approach target velocity."""
        max_delta = accel * dt
        diff = target - current
        if abs(diff) <= max_delta:
            return target
        return current + (max_delta if diff > 0 else -max_delta)

    def update_and_publish(self, dt):
        # Smoothly ramp current velocities toward target velocities
        self.current_vx = self.ramp_value(self.current_vx, self.target_vx, LIN_ACCEL, dt)
        self.current_vy = self.ramp_value(self.current_vy, self.target_vy, LIN_ACCEL, dt)
        self.current_vz = self.ramp_value(self.current_vz, self.target_vz, LIN_ACCEL, dt)
        self.current_yaw = self.ramp_value(self.current_yaw, self.target_yaw, YAW_ACCEL, dt)

        # Publish Twist message
        msg = Twist()
        msg.linear.x = self.current_vx
        msg.linear.y = self.current_vy
        msg.linear.z = self.current_vz
        msg.angular.z = self.current_yaw
        self.pub.publish(msg)

    def run(self):
        print(INSTRUCTIONS)
        dt = 1.0 / PUBLISH_RATE_HZ

        # Enter raw terminal mode once for the entire runtime loop
        tty.setraw(sys.stdin.fileno())
        try:
            last_time = time.monotonic()
            running = True

            while running and rclpy.ok():
                now = time.monotonic()
                loop_dt = now - last_time
                last_time = now

                # Drain stdin buffer
                while True:
                    key = self.get_key_nonblocking()
                    if not key:
                        break
                    if not self.process_key(key):
                        running = False
                        break

                self.decay_stale_targets()
                self.update_and_publish(loop_dt if loop_dt > 0 else dt)

                # Process background ROS 2 callbacks (non-blocking)
                rclpy.spin_once(self, timeout_sec=0)

                # Maintain loop rate
                elapsed = time.monotonic() - now
                time.sleep(max(0.0, dt - elapsed))

        finally:
            # Restore terminal state & publish zero velocity stop command
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            stop_msg = Twist()
            self.pub.publish(stop_msg)


def main():
    rclpy.init()
    node = SmoothQuadTeleop()
    try:
        node.run()
    except Exception as e:
        print(f"\nTeleop shutdown error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
