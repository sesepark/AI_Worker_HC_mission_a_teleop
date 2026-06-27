#!/usr/bin/env python3

import os
import subprocess
import sys
import time

import rclpy
from rclpy.clock import Clock
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import String


class MissionBSystemNav(Node):
    """System-facing Mission B navigation coordinator.

    Commands from the system team:
      A_TO_B     -> move from A to the B stop line and align with LiDAR
      APPROACH_B -> move a short fixed distance closer to the table and save B
      B_TO_A     -> return from B to A
      STOP       -> stop the current navigation subprocess
    """

    def __init__(self):
        super().__init__('sg2_mission_b_system_nav')

        self.declare_parameter('system_action_topic', '/mission_b/system/action')
        self.declare_parameter('nav_event_topic', '/mission_b/nav/event')
        self.declare_parameter('system_nav_state_topic', '/mission_b/system_nav/state')
        self.declare_parameter('nav_state_topic', '/mission_b/nav/state')
        self.declare_parameter('failure_reason_topic', '/mission_b/nav/failure_reason')
        self.declare_parameter('return_allowed_topic', '/mission_b/nav/return_allowed')
        self.declare_parameter(
            'b_approach_allowed_topic', '/mission_b/nav/b_approach_allowed')
        self.declare_parameter(
            'reached_b_stop_line_topic', '/mission_b/nav/reached_b_stop_line')
        self.declare_parameter(
            'reached_b_place_pose_topic', '/mission_b/nav/reached_b_place_pose')
        self.declare_parameter('arrived_a_topic', '/mission_b/nav/arrived_a')
        self.declare_parameter('route_shutdown_timeout_sec', 3.0)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('backward_distance', 0.70)
        self.declare_parameter('right_distance', 3.80)
        self.declare_parameter('forward_distance', 0.30)
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('lateral_speed', 0.20)
        self.declare_parameter('forward_speed', 0.12)
        self.declare_parameter('b_approach_forward_distance', 0.06)
        self.declare_parameter('b_approach_speed', 0.12)
        self.declare_parameter('pause_after_a_mark_sec', 1.0)
        self.declare_parameter('pause_at_b_sec', 0.0)
        self.declare_parameter('keep_alive_after_done_sec', 2.0)
        self.declare_parameter('enable_lidar_alignment', True)
        self.declare_parameter('desired_front_distance', 0.70)
        self.declare_parameter('return_mode', 'reverse_segments')
        self.declare_parameter('hold_heading_during_segments', True)
        self.declare_parameter('enable_return_final_trim', True)
        self.declare_parameter('return_final_trim_tolerance', 0.03)
        self.declare_parameter('return_final_trim_max_distance', 0.35)
        self.declare_parameter('enable_return_a_lidar_alignment', True)
        self.declare_parameter('return_a_desired_front_distance', 0.30)
        self.declare_parameter('return_a_align_timeout_sec', 30.0)
        self.declare_parameter('min_command_speed', 0.12)
        self.declare_parameter('min_align_vx', 0.12)
        self.declare_parameter('min_center_vy', 0.12)
        self.declare_parameter('min_yaw_wz', 0.12)
        self.declare_parameter('return_a_pre_align_enabled', True)
        self.declare_parameter('return_a_pre_align_backoff_distance', 0.45)
        self.declare_parameter('return_a_skip_final_trim_when_lidar', True)
        self.declare_parameter('return_a_alignment_mode', 'legs')
        self.declare_parameter('return_a_fallback_to_front_band', True)
        self.declare_parameter('return_a_use_front_band_alignment', True)
        self.declare_parameter('return_a_front_band_width', 0.35)
        self.declare_parameter('return_a_roi_x_min', 0.15)
        self.declare_parameter('return_a_roi_x_max', 1.60)
        self.declare_parameter('return_a_roi_y_abs', 2.00)
        self.declare_parameter('return_a_leg_use_front_band', False)
        self.declare_parameter('return_a_leg_cluster_gap', 0.14)
        self.declare_parameter('return_a_leg_min_cluster_points', 1)
        self.declare_parameter('return_a_leg_max_cluster_width_y', 0.22)
        self.declare_parameter('return_a_leg_max_cluster_depth_x', 0.35)
        self.declare_parameter('return_a_leg_min_spacing', 1.00)
        self.declare_parameter('return_a_leg_expected_count', 3)
        self.declare_parameter('return_a_leg_use_yaw', False)
        self.declare_parameter('return_a_debug_markers', True)
        self.declare_parameter(
            'return_a_debug_marker_topic', '/mission_b/debug/conveyor_legs')
        self.declare_parameter('return_a_debug_marker_lifetime_sec', 0.8)

        self.system_action_topic = self.get_parameter('system_action_topic').value
        self.nav_event_topic = self.get_parameter('nav_event_topic').value
        self.system_nav_state_topic = self.get_parameter(
            'system_nav_state_topic').value
        self.nav_state_topic = self.get_parameter('nav_state_topic').value
        self.failure_reason_topic = self.get_parameter('failure_reason_topic').value
        self.return_allowed_topic = self.get_parameter('return_allowed_topic').value
        self.b_approach_allowed_topic = self.get_parameter(
            'b_approach_allowed_topic').value
        self.reached_b_stop_line_topic = self.get_parameter(
            'reached_b_stop_line_topic').value
        self.reached_b_place_pose_topic = self.get_parameter(
            'reached_b_place_pose_topic').value
        self.arrived_a_topic = self.get_parameter('arrived_a_topic').value
        self.route_shutdown_timeout_sec = float(
            self.get_parameter('route_shutdown_timeout_sec').value)

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.event_pub = self.create_publisher(
            String, self.nav_event_topic, latched_qos)
        self.state_pub = self.create_publisher(
            String, self.system_nav_state_topic, latched_qos)
        self.b_approach_allowed_pub = self.create_publisher(
            Bool, self.b_approach_allowed_topic, 10)
        self.return_allowed_pub = self.create_publisher(
            Bool, self.return_allowed_topic, 10)

        self.create_subscription(
            String, self.system_action_topic, self._action_callback, 10)
        self.create_subscription(
            String, self.nav_state_topic, self._nav_state_callback, 10)
        self.create_subscription(
            String, self.failure_reason_topic, self._failure_callback, 10)
        self.create_subscription(
            Bool,
            self.reached_b_stop_line_topic,
            self._reached_b_stop_line_callback,
            10)
        self.create_subscription(
            Bool,
            self.reached_b_place_pose_topic,
            self._reached_b_place_pose_callback,
            10)
        self.create_subscription(
            Bool, self.arrived_a_topic, self._arrived_a_callback, 10)

        self.state = 'IDLE'
        self.nav_state = ''
        self.failure_reason = ''
        self.route_process = None
        self.state_started_at = time.monotonic()

        self.timer = self.create_timer(
            0.1,
            self._timer_callback,
            clock=Clock(clock_type=ClockType.STEADY_TIME))

        self._publish_allowed(False, False)
        self._publish_state()
        self._publish_event('READY')
        self.get_logger().info(
            'Mission B safe-A system navigation ready. '
            f'action_topic={self.system_action_topic}, '
            f'event_topic={self.nav_event_topic}')

    def _action_callback(self, msg):
        action = self._normalize_action(msg.data)
        self.get_logger().info(f'Received system action: {action}')

        if action == 'A_TO_B':
            if self.state not in ('IDLE', 'STOPPED', 'FAILED'):
                self._reject(action, f'busy_state={self.state}')
                return
            self._start_a_to_b()
            return

        if action == 'APPROACH_B':
            if self.state != 'WAITING_APPROACH_B_ACTION':
                self._reject(action, f'expected_WAITING_APPROACH_B_ACTION got {self.state}')
                return
            self._transition('APPROACHING_B_PLACE_POSE')
            self._publish_allowed(approach=True, ret=False)
            self._publish_event('APPROACH_B_ACCEPTED')
            return

        if action == 'B_TO_A':
            if self.state != 'WAITING_B_TO_A_ACTION':
                self._reject(action, f'expected_WAITING_B_TO_A_ACTION got {self.state}')
                return
            self._transition('MOVING_B_TO_A')
            self._publish_allowed(approach=False, ret=True)
            self._publish_event('B_TO_A_ACCEPTED')
            return

        if action == 'STOP':
            self._stop_route_process()
            self._publish_allowed(False, False)
            self._transition('STOPPED')
            self._publish_event('STOPPED')
            return

        self._reject(action, 'unknown_action')

    def _timer_callback(self):
        self._publish_state()

        if self.state == 'APPROACHING_B_PLACE_POSE':
            self._publish_allowed(approach=True, ret=False)
        elif self.state == 'MOVING_B_TO_A':
            self._publish_allowed(approach=False, ret=True)

        if self.state in (
            'MOVING_A_TO_B_STOP_LINE',
            'WAITING_APPROACH_B_ACTION',
            'APPROACHING_B_PLACE_POSE',
            'WAITING_B_TO_A_ACTION',
            'MOVING_B_TO_A',
        ):
            self._fail_if_route_exited_early()

    def _start_a_to_b(self):
        self._stop_route_process()
        self.failure_reason = ''
        self.nav_state = ''
        self._publish_allowed(False, False)
        command = self._make_route_command()
        self.route_process = subprocess.Popen(command)
        self._transition('MOVING_A_TO_B_STOP_LINE')
        self._publish_event('A_TO_B_ACCEPTED')
        self.get_logger().info('Started A_TO_B route subprocess.')

    def _reached_b_stop_line_callback(self, msg):
        if not msg.data or self.state != 'MOVING_A_TO_B_STOP_LINE':
            return
        self._transition('WAITING_APPROACH_B_ACTION')
        self._publish_event('REACHED_B_STOP_LINE')

    def _reached_b_place_pose_callback(self, msg):
        if not msg.data or self.state != 'APPROACHING_B_PLACE_POSE':
            return
        self._publish_allowed(approach=False, ret=False)
        self._transition('WAITING_B_TO_A_ACTION')
        self._publish_event('REACHED_B_PLACE_POSE')

    def _arrived_a_callback(self, msg):
        if not msg.data or self.state != 'MOVING_B_TO_A':
            return
        self._publish_allowed(False, False)
        self._stop_route_process()
        self._transition('IDLE')
        self._publish_event('REACHED_A')

    def _nav_state_callback(self, msg):
        self.nav_state = msg.data

    def _failure_callback(self, msg):
        if msg.data:
            self._fail(f'navigation_failed: {msg.data}')

    def _transition(self, state):
        if self.state != state:
            self.get_logger().info(f'System nav state: {self.state} -> {state}')
        self.state = state
        self.state_started_at = time.monotonic()
        self._publish_state()

    def _publish_state(self):
        msg = String()
        msg.data = (
            f'{self.state};nav={self.nav_state};failure={self.failure_reason}')
        self.state_pub.publish(msg)

    def _publish_event(self, event):
        msg = String()
        msg.data = event
        self.event_pub.publish(msg)
        self.get_logger().info(f'Published nav event: {event}')

    def _publish_allowed(self, approach, ret):
        approach_msg = Bool()
        approach_msg.data = approach
        self.b_approach_allowed_pub.publish(approach_msg)

        return_msg = Bool()
        return_msg.data = ret
        self.return_allowed_pub.publish(return_msg)

    def _reject(self, action, reason):
        self.get_logger().warning(f'Rejected action {action}: {reason}')
        self._publish_event(f'REJECTED:{action}:{reason}')

    def _fail(self, reason):
        if self.state == 'FAILED':
            return
        self.failure_reason = reason
        self._stop_route_process()
        self._publish_allowed(False, False)
        self._transition('FAILED')
        self._publish_event(f'FAILED:{reason}')
        self.get_logger().error(f'Mission B system navigation failed: {reason}')

    def _fail_if_route_exited_early(self):
        if self.route_process is None:
            self._fail('route_process_not_running')
            return
        return_code = self.route_process.poll()
        if return_code is not None:
            self._fail(f'route_process_exited_early: code={return_code}')

    def _stop_route_process(self):
        if self.route_process is None:
            return
        if self.route_process.poll() is None:
            self.route_process.terminate()
            deadline = time.monotonic() + self.route_shutdown_timeout_sec
            while self.route_process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if self.route_process.poll() is None:
                self.route_process.kill()
        self.route_process = None

    def _make_route_command(self):
        params = {
            'cmd_vel_topic': self.get_parameter('cmd_vel_topic').value,
            'backward_distance': self.get_parameter('backward_distance').value,
            'right_distance': self.get_parameter('right_distance').value,
            'forward_distance': self.get_parameter('forward_distance').value,
            'linear_speed': self.get_parameter('linear_speed').value,
            'lateral_speed': self.get_parameter('lateral_speed').value,
            'forward_speed': self.get_parameter('forward_speed').value,
            'b_approach_forward_distance': self.get_parameter(
                'b_approach_forward_distance').value,
            'b_approach_speed': self.get_parameter('b_approach_speed').value,
            'pause_after_a_mark_sec': self.get_parameter('pause_after_a_mark_sec').value,
            'pause_at_b_sec': self.get_parameter('pause_at_b_sec').value,
            'keep_alive_after_done_sec': self.get_parameter('keep_alive_after_done_sec').value,
            'enable_lidar_alignment': self.get_parameter('enable_lidar_alignment').value,
            'desired_front_distance': self.get_parameter('desired_front_distance').value,
            'return_to_start': True,
            'return_mode': self.get_parameter('return_mode').value,
            'hold_heading_during_segments': self.get_parameter('hold_heading_during_segments').value,
            'enable_return_final_trim': self.get_parameter('enable_return_final_trim').value,
            'return_final_trim_tolerance': self.get_parameter('return_final_trim_tolerance').value,
            'return_final_trim_max_distance': self.get_parameter('return_final_trim_max_distance').value,
            'enable_return_a_lidar_alignment': self.get_parameter('enable_return_a_lidar_alignment').value,
            'return_a_desired_front_distance': self.get_parameter('return_a_desired_front_distance').value,
            'return_a_align_timeout_sec': self.get_parameter('return_a_align_timeout_sec').value,
            'min_command_speed': self.get_parameter('min_command_speed').value,
            'min_align_vx': self.get_parameter('min_align_vx').value,
            'min_center_vy': self.get_parameter('min_center_vy').value,
            'min_yaw_wz': self.get_parameter('min_yaw_wz').value,
            'use_sim_time': self.get_parameter('use_sim_time').value,
            'wait_for_b_approach_command': True,
            'b_approach_allowed_topic': self.b_approach_allowed_topic,
            'reached_b_stop_line_topic': self.reached_b_stop_line_topic,
            'reached_b_place_pose_topic': self.reached_b_place_pose_topic,
            'wait_for_return_allowed': True,
            'return_allowed_topic': self.return_allowed_topic,
            'nav_state_topic': self.nav_state_topic,
            'arrived_a_topic': self.arrived_a_topic,
            'arrived_b_topic': '/mission_b/nav/arrived_b',
            'failure_reason_topic': self.failure_reason_topic,
            'return_a_pre_align_enabled': self.get_parameter(
                'return_a_pre_align_enabled').value,
            'return_a_pre_align_backoff_distance': self.get_parameter(
                'return_a_pre_align_backoff_distance').value,
            'return_a_skip_final_trim_when_lidar': self.get_parameter(
                'return_a_skip_final_trim_when_lidar').value,
            'return_a_alignment_mode': self.get_parameter(
                'return_a_alignment_mode').value,
            'return_a_fallback_to_front_band': self.get_parameter(
                'return_a_fallback_to_front_band').value,
            'return_a_use_front_band_alignment': self.get_parameter(
                'return_a_use_front_band_alignment').value,
            'return_a_front_band_width': self.get_parameter(
                'return_a_front_band_width').value,
            'return_a_roi_x_min': self.get_parameter('return_a_roi_x_min').value,
            'return_a_roi_x_max': self.get_parameter('return_a_roi_x_max').value,
            'return_a_roi_y_abs': self.get_parameter('return_a_roi_y_abs').value,
            'return_a_leg_use_front_band': self.get_parameter(
                'return_a_leg_use_front_band').value,
            'return_a_leg_cluster_gap': self.get_parameter(
                'return_a_leg_cluster_gap').value,
            'return_a_leg_min_cluster_points': self.get_parameter(
                'return_a_leg_min_cluster_points').value,
            'return_a_leg_max_cluster_width_y': self.get_parameter(
                'return_a_leg_max_cluster_width_y').value,
            'return_a_leg_max_cluster_depth_x': self.get_parameter(
                'return_a_leg_max_cluster_depth_x').value,
            'return_a_leg_min_spacing': self.get_parameter(
                'return_a_leg_min_spacing').value,
            'return_a_leg_expected_count': self.get_parameter(
                'return_a_leg_expected_count').value,
            'return_a_leg_use_yaw': self.get_parameter(
                'return_a_leg_use_yaw').value,
            'return_a_debug_markers': self.get_parameter(
                'return_a_debug_markers').value,
            'return_a_debug_marker_topic': self.get_parameter(
                'return_a_debug_marker_topic').value,
            'return_a_debug_marker_lifetime_sec': self.get_parameter(
                'return_a_debug_marker_lifetime_sec').value,
        }

        route_script = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'sg2_mission_b_route.py')
        command = [
            sys.executable,
            route_script,
            '--ros-args',
        ]
        for key, value in params.items():
            command.extend(['-p', f'{key}:={self._format_param(value)}'])
        return command

    @staticmethod
    def _normalize_action(value):
        action = value.strip().upper().replace('-', '_').replace(' ', '_')
        aliases = {
            'ACTION_A_TO_B': 'A_TO_B',
            'START_A_TO_B': 'A_TO_B',
            'MOVE_A_TO_B': 'A_TO_B',
            'MOVE_FORWARD': 'APPROACH_B',
            'FORWARD_TO_TABLE': 'APPROACH_B',
            'FINAL_APPROACH_B': 'APPROACH_B',
            'ACTION_B_TO_A': 'B_TO_A',
            'RETURN_TO_A': 'B_TO_A',
            'MOVE_B_TO_A': 'B_TO_A',
        }
        return aliases.get(action, action)

    @staticmethod
    def _format_param(value):
        if isinstance(value, bool):
            return 'true' if value else 'false'
        return str(value)

    def stop(self):
        self._stop_route_process()
        self._publish_allowed(False, False)


def main():
    rclpy.init()
    node = MissionBSystemNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
