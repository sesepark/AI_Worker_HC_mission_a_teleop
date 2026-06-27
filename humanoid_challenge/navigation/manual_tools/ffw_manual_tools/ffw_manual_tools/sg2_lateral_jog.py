#!/usr/bin/env python3
# Copyright 2026 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class Sg2LateralJog(Node):

    def __init__(self):
        super().__init__('sg2_lateral_jog')

        self.declare_parameter('direction', 'right')
        self.declare_parameter('distance', 0.50)
        self.declare_parameter('speed', 0.12)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('use_odom_stop', True)
        self.declare_parameter('max_duration_sec', 12.0)
        self.declare_parameter('wait_for_subscriber_sec', 2.0)
        self.declare_parameter('wrong_direction_tolerance', 0.05)

        self.direction = str(self.get_parameter('direction').value).lower()
        self.distance = abs(float(self.get_parameter('distance').value))
        self.distance = self.distance * 0.971 - 0.031
        self.speed = abs(float(self.get_parameter('speed').value))
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.use_odom_stop = bool(self.get_parameter('use_odom_stop').value)
        self.max_duration_sec = float(self.get_parameter('max_duration_sec').value)
        self.wait_for_subscriber_sec = float(
            self.get_parameter('wait_for_subscriber_sec').value)
        self.wrong_direction_tolerance = float(
            self.get_parameter('wrong_direction_tolerance').value)

        if self.direction not in ('left', 'right'):
            raise ValueError('direction must be "left" or "right"')
        if self.speed <= 0.0:
            raise ValueError('speed must be positive')

        self.sign = 1.0 if self.direction == 'left' else -1.0
        self.open_loop_duration = self.distance / self.speed
        self.stop_cycles_remaining = int(max(5, self.rate_hz * 0.5))

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, qos)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, qos)

        self.current_pose = None
        self.start_pose = None
        self.start_time = None
        self.done = False
        self.stopping = False
        self.stop_reason = ''

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            'SG2 lateral jog ready: '
            f'direction={self.direction}, distance={self.distance:.2f} m, '
            f'speed={self.speed:.2f} m/s, use_odom_stop={self.use_odom_stop}, '
            f'cmd_vel_topic={self.cmd_vel_topic}')

    def wait_for_cmd_subscriber(self):
        deadline = time.monotonic() + self.wait_for_subscriber_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.cmd_pub.get_subscription_count() > 0:
                self.get_logger().info(
                    f'Found {self.cmd_pub.get_subscription_count()} subscriber(s) '
                    f'on {self.cmd_vel_topic}.'
                )
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warning(
            f'No subscribers discovered on {self.cmd_vel_topic}. '
            'Command will still be published.'
        )

    def _odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w)
        self.current_pose = (position.x, position.y, yaw)

    def _timer_callback(self):
        if self.done:
            return

        if self.current_pose is None:
            self.get_logger().warn(
                f'Waiting for odom on {self.odom_topic}...', throttle_duration_sec=1.0)
            return

        if self.start_pose is None:
            self.start_pose = self.current_pose
            self.start_time = time.monotonic()
            self.get_logger().info(
                f'Start lateral jog: {self.direction} {self.distance:.2f} m')

        elapsed = time.monotonic() - self.start_time
        forward_delta, left_delta = self._compute_robot_frame_delta()
        signed_lateral_progress = self.sign * left_delta

        if self.stopping:
            self.cmd_pub.publish(Twist())
            self.stop_cycles_remaining -= 1
            if self.stop_cycles_remaining <= 0:
                self.done = True
                self.get_logger().info(
                    'Lateral jog done: '
                    f'reason={self.stop_reason}, elapsed={elapsed:.2f}s, '
                    f'odom_left_delta={left_delta:+.3f}m, '
                    f'odom_forward_delta={forward_delta:+.3f}m')
            return

        should_stop = False
        if self.use_odom_stop and signed_lateral_progress >= self.distance:
            self.stop_reason = 'odom_distance_reached'
            should_stop = True
        elif (self.use_odom_stop and
                signed_lateral_progress <= -self.wrong_direction_tolerance):
            self.stop_reason = 'wrong_direction_detected'
            should_stop = True
        elif not self.use_odom_stop and elapsed >= self.open_loop_duration:
            self.stop_reason = 'open_loop_duration_reached'
            should_stop = True
        elif elapsed >= self.max_duration_sec:
            self.stop_reason = 'max_duration_reached'
            should_stop = True

        if should_stop:
            self.stopping = True
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()
        cmd.linear.y = self.sign * self.speed
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'jog: elapsed={elapsed:.2f}s, '
            f'odom_left_delta={left_delta:+.3f}m / target='
            f'{self.sign * self.distance:+.3f}m',
            throttle_duration_sec=0.5)

    def _compute_robot_frame_delta(self):
        start_x, start_y, start_yaw = self.start_pose
        current_x, current_y, _ = self.current_pose
        dx = current_x - start_x
        dy = current_y - start_y

        forward_delta = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
        left_delta = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
        return forward_delta, left_delta

    def stop(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = Sg2LateralJog()
    try:
        node.wait_for_cmd_subscriber()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
