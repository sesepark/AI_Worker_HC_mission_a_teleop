#!/usr/bin/env python3

import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


def get_key():
    """Read a single keypress from terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class Sg2KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('sg2_keyboard_teleop')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('lateral_speed', 0.10)
        self.declare_parameter('angular_speed', 0.25)
        self.declare_parameter('speed_step', 0.02)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.lateral_speed = float(self.get_parameter('lateral_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.speed_step = float(self.get_parameter('speed_step').value)

        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, 5)
        self._print_help()

    def _print_help(self):
        self.get_logger().info(
            '\nSG2 Mobile Teleop\n'
            '---------------------------\n'
            'W/S : Forward / Backward\n'
            'A/D : Strafe Left / Right\n'
            'Q/E : Turn Left / Right\n'
            '+/- : Increase / Decrease speed\n'
            'Space: Stop\n'
            'H: Help\n'
            'Ctrl+C: Quit\n'
            f'cmd_vel_topic: {self.cmd_vel_topic}\n'
            f'linear={self.linear_speed:.2f} m/s, '
            f'lateral={self.lateral_speed:.2f} m/s, '
            f'angular={self.angular_speed:.2f} rad/s\n'
        )

    def _change_speed(self, direction):
        self.linear_speed = max(0.02, self.linear_speed + direction * self.speed_step)
        self.lateral_speed = max(0.02, self.lateral_speed + direction * self.speed_step)
        self.angular_speed = max(0.05, self.angular_speed + direction * self.speed_step * 2.0)
        self.get_logger().info(
            f'speed: linear={self.linear_speed:.2f} m/s, '
            f'lateral={self.lateral_speed:.2f} m/s, '
            f'angular={self.angular_speed:.2f} rad/s'
        )

    def run(self):
        while rclpy.ok():
            key = get_key()
            twist = Twist()

            if key == 'w':
                twist.linear.x = self.linear_speed
            elif key == 's':
                twist.linear.x = -self.linear_speed
            elif key == 'a':
                twist.linear.y = self.lateral_speed
            elif key == 'd':
                twist.linear.y = -self.lateral_speed
            elif key == 'q':
                twist.angular.z = self.angular_speed
            elif key == 'e':
                twist.angular.z = -self.angular_speed
            elif key == '+':
                self._change_speed(1.0)
                continue
            elif key == '-':
                self._change_speed(-1.0)
                continue
            elif key in ('h', 'H'):
                self._print_help()
                continue
            elif key == ' ':
                pass
            elif key == '\x03':  # Ctrl+C
                break

            self.publisher.publish(twist)

        self.publisher.publish(Twist())
        self.get_logger().info('SG2 teleop stopped.')


def main():
    rclpy.init()
    node = Sg2KeyboardTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
