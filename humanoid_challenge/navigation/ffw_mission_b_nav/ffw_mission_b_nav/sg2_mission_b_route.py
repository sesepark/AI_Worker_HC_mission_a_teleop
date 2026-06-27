#!/usr/bin/env python3

import math
import statistics
import time
from itertools import combinations

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import ColorRGBA
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def clamp_with_min_abs(value, max_abs, min_abs):
    if abs(value) < 1.0e-9:
        return 0.0

    max_abs = abs(max_abs)
    min_abs = min(abs(min_abs), max_abs)
    limited = clamp(value, -max_abs, max_abs)

    if abs(limited) < min_abs:
        return math.copysign(min_abs, limited)
    return limited


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def percentile(values, ratio):
    if not values:
        return 0.0
    sorted_values = sorted(values)
    index = ratio * (len(sorted_values) - 1)
    low_index = int(math.floor(index))
    high_index = int(math.ceil(index))
    if low_index == high_index:
        return sorted_values[low_index]
    fraction = index - low_index
    return (
        sorted_values[low_index] * (1.0 - fraction) +
        sorted_values[high_index] * fraction
    )


class Sg2MissionBRoute(Node):
    """Mission B primitive route: back, strafe right, optional forward, LiDAR align."""

    def __init__(self):
        super().__init__('sg2_mission_b_route')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('nav_state_topic', '/mission_b/nav/state')
        self.declare_parameter('arrived_a_topic', '/mission_b/nav/arrived_a')
        self.declare_parameter('arrived_b_topic', '/mission_b/nav/arrived_b')
        self.declare_parameter('failure_reason_topic', '/mission_b/nav/failure_reason')
        self.declare_parameter('return_allowed_topic', '/mission_b/nav/return_allowed')
        self.declare_parameter(
            'b_approach_allowed_topic', '/mission_b/nav/b_approach_allowed')
        self.declare_parameter(
            'reached_b_stop_line_topic', '/mission_b/nav/reached_b_stop_line')
        self.declare_parameter(
            'reached_b_place_pose_topic', '/mission_b/nav/reached_b_place_pose')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('wait_for_subscriber_sec', 2.0)
        self.declare_parameter('publish_waypoint_poses', True)
        self.declare_parameter('waypoint_frame', 'odom')
        self.declare_parameter('keep_alive_after_done_sec', 30.0)
        self.declare_parameter('pause_after_a_mark_sec', 1.0)
        self.declare_parameter('return_to_start', False)
        self.declare_parameter('pause_at_b_sec', 0.0)
        self.declare_parameter('return_mode', 'reverse_segments')
        self.declare_parameter('return_match_start_yaw', True)
        self.declare_parameter('wait_for_b_approach_command', False)
        self.declare_parameter('b_approach_allowed_timeout_sec', 0.0)
        self.declare_parameter('wait_for_return_allowed', False)
        self.declare_parameter('return_allowed_timeout_sec', 0.0)

        self.declare_parameter('backward_distance', 0.70)
        self.declare_parameter('right_distance', 3.80)
        self.declare_parameter('forward_distance', 0.0)
        self.declare_parameter('min_command_speed', 0.12)
        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('lateral_speed', 0.20)
        self.declare_parameter('forward_speed', 0.12)
        self.declare_parameter('b_approach_forward_distance', 0.06)
        self.declare_parameter('b_approach_speed', 0.12)
        self.declare_parameter('max_segment_duration_sec', 45.0)
        self.declare_parameter('wrong_direction_tolerance', 0.05)
        self.declare_parameter('hold_heading_during_segments', True)
        self.declare_parameter('heading_hold_tolerance', 0.015)
        self.declare_parameter('heading_hold_kz', 1.20)
        self.declare_parameter('heading_hold_max_wz', 0.18)

        self.declare_parameter('enable_lidar_alignment', False)
        self.declare_parameter('desired_front_distance', 0.70)
        self.declare_parameter('align_timeout_sec', 45.0)
        self.declare_parameter('front_tolerance', 0.04)
        self.declare_parameter('lateral_tolerance', 0.03)
        self.declare_parameter('yaw_tolerance', 0.025)
        self.declare_parameter('yaw_priority_threshold', 0.08)
        self.declare_parameter('stable_cycles_required', 12)

        self.declare_parameter('roi_x_min', 0.20)
        self.declare_parameter('roi_x_max', 1.80)
        self.declare_parameter('roi_y_abs', 1.20)
        self.declare_parameter('front_band_width', 0.35)
        self.declare_parameter('min_points', 8)
        self.declare_parameter('min_visible_width', 0.30)

        self.declare_parameter('kx', 0.55)
        self.declare_parameter('ky', 0.55)
        self.declare_parameter('kz', 1.00)
        self.declare_parameter('max_vx', 0.12)
        self.declare_parameter('max_vy', 0.12)
        self.declare_parameter('max_wz', 0.12)
        self.declare_parameter('min_align_vx', 0.12)
        self.declare_parameter('min_center_vy', 0.12)
        self.declare_parameter('min_yaw_wz', 0.12)

        self.declare_parameter('return_timeout_sec', 120.0)
        self.declare_parameter('return_position_tolerance', 0.05)
        self.declare_parameter('return_yaw_tolerance', 0.05)
        self.declare_parameter('return_kx', 0.60)
        self.declare_parameter('return_ky', 0.60)
        self.declare_parameter('return_kz', 1.00)
        self.declare_parameter('return_max_vx', 0.12)
        self.declare_parameter('return_max_vy', 0.12)
        self.declare_parameter('return_max_wz', 0.12)
        self.declare_parameter('return_min_vx', 0.12)
        self.declare_parameter('return_min_vy', 0.12)
        self.declare_parameter('return_min_wz', 0.12)
        self.declare_parameter('enable_return_final_trim', True)
        self.declare_parameter('return_final_trim_tolerance', 0.03)
        self.declare_parameter('return_final_trim_max_distance', 0.35)
        self.declare_parameter('enable_return_a_lidar_alignment', False)
        self.declare_parameter('return_a_desired_front_distance', 0.30)
        self.declare_parameter('return_a_align_timeout_sec', 30.0)
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

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.nav_state_topic = self.get_parameter('nav_state_topic').value
        self.arrived_a_topic = self.get_parameter('arrived_a_topic').value
        self.arrived_b_topic = self.get_parameter('arrived_b_topic').value
        self.failure_reason_topic = self.get_parameter('failure_reason_topic').value
        self.return_allowed_topic = self.get_parameter('return_allowed_topic').value
        self.b_approach_allowed_topic = self.get_parameter(
            'b_approach_allowed_topic').value
        self.reached_b_stop_line_topic = self.get_parameter(
            'reached_b_stop_line_topic').value
        self.reached_b_place_pose_topic = self.get_parameter(
            'reached_b_place_pose_topic').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.wait_for_subscriber_sec = float(
            self.get_parameter('wait_for_subscriber_sec').value)
        self.publish_waypoint_poses = bool(
            self.get_parameter('publish_waypoint_poses').value)
        self.waypoint_frame = self.get_parameter('waypoint_frame').value
        self.keep_alive_after_done_sec = max(
            0.0, float(self.get_parameter('keep_alive_after_done_sec').value))
        self.pause_after_a_mark_sec = max(
            0.0, float(self.get_parameter('pause_after_a_mark_sec').value))
        self.return_to_start = bool(self.get_parameter('return_to_start').value)
        self.pause_at_b_sec = max(
            0.0, float(self.get_parameter('pause_at_b_sec').value))
        self.return_mode = self.get_parameter('return_mode').value
        self.return_match_start_yaw = bool(
            self.get_parameter('return_match_start_yaw').value)
        self.wait_for_b_approach_command = bool(
            self.get_parameter('wait_for_b_approach_command').value)
        self.b_approach_allowed_timeout_sec = max(
            0.0, float(self.get_parameter('b_approach_allowed_timeout_sec').value))
        self.wait_for_return_allowed = bool(
            self.get_parameter('wait_for_return_allowed').value)
        self.return_allowed_timeout_sec = max(
            0.0, float(self.get_parameter('return_allowed_timeout_sec').value))

        self.backward_distance = abs(float(
            self.get_parameter('backward_distance').value))
        self.right_distance = abs(float(
            self.get_parameter('right_distance').value))
        self.forward_distance = abs(float(
            self.get_parameter('forward_distance').value))
        self.min_command_speed = abs(float(
            self.get_parameter('min_command_speed').value))
        self.linear_speed = max(
            abs(float(self.get_parameter('linear_speed').value)),
            self.min_command_speed)
        self.lateral_speed = max(
            abs(float(self.get_parameter('lateral_speed').value)),
            self.min_command_speed)
        self.forward_speed = max(
            abs(float(self.get_parameter('forward_speed').value)),
            self.min_command_speed)
        self.b_approach_forward_distance = abs(float(
            self.get_parameter('b_approach_forward_distance').value))
        self.b_approach_speed = max(
            abs(float(self.get_parameter('b_approach_speed').value)),
            self.min_command_speed)
        self.max_segment_duration_sec = float(
            self.get_parameter('max_segment_duration_sec').value)
        self.wrong_direction_tolerance = float(
            self.get_parameter('wrong_direction_tolerance').value)
        self.hold_heading_during_segments = bool(
            self.get_parameter('hold_heading_during_segments').value)
        self.heading_hold_tolerance = abs(float(
            self.get_parameter('heading_hold_tolerance').value))
        self.heading_hold_kz = float(
            self.get_parameter('heading_hold_kz').value)
        self.heading_hold_max_wz = abs(float(
            self.get_parameter('heading_hold_max_wz').value))

        self.enable_lidar_alignment = bool(
            self.get_parameter('enable_lidar_alignment').value)
        self.desired_front_distance = float(
            self.get_parameter('desired_front_distance').value)
        self.align_timeout_sec = float(self.get_parameter('align_timeout_sec').value)
        self.front_tolerance = float(self.get_parameter('front_tolerance').value)
        self.lateral_tolerance = float(self.get_parameter('lateral_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.yaw_priority_threshold = float(
            self.get_parameter('yaw_priority_threshold').value)
        self.stable_cycles_required = int(
            self.get_parameter('stable_cycles_required').value)

        self.roi_x_min = float(self.get_parameter('roi_x_min').value)
        self.roi_x_max = float(self.get_parameter('roi_x_max').value)
        self.roi_y_abs = float(self.get_parameter('roi_y_abs').value)
        self.front_band_width = float(self.get_parameter('front_band_width').value)
        self.min_points = int(self.get_parameter('min_points').value)
        self.min_visible_width = float(self.get_parameter('min_visible_width').value)

        self.kx = float(self.get_parameter('kx').value)
        self.ky = float(self.get_parameter('ky').value)
        self.kz = float(self.get_parameter('kz').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.min_align_vx = float(self.get_parameter('min_align_vx').value)
        self.min_center_vy = float(self.get_parameter('min_center_vy').value)
        self.min_yaw_wz = float(self.get_parameter('min_yaw_wz').value)

        self.return_timeout_sec = float(
            self.get_parameter('return_timeout_sec').value)
        self.return_position_tolerance = float(
            self.get_parameter('return_position_tolerance').value)
        self.return_yaw_tolerance = float(
            self.get_parameter('return_yaw_tolerance').value)
        self.return_kx = float(self.get_parameter('return_kx').value)
        self.return_ky = float(self.get_parameter('return_ky').value)
        self.return_kz = float(self.get_parameter('return_kz').value)
        self.return_max_vx = float(self.get_parameter('return_max_vx').value)
        self.return_max_vy = float(self.get_parameter('return_max_vy').value)
        self.return_max_wz = float(self.get_parameter('return_max_wz').value)
        self.return_min_vx = float(self.get_parameter('return_min_vx').value)
        self.return_min_vy = float(self.get_parameter('return_min_vy').value)
        self.return_min_wz = float(self.get_parameter('return_min_wz').value)
        self.enable_return_final_trim = bool(
            self.get_parameter('enable_return_final_trim').value)
        self.return_final_trim_tolerance = abs(float(
            self.get_parameter('return_final_trim_tolerance').value))
        self.return_final_trim_max_distance = abs(float(
            self.get_parameter('return_final_trim_max_distance').value))
        self.enable_return_a_lidar_alignment = bool(
            self.get_parameter('enable_return_a_lidar_alignment').value)
        self.return_a_desired_front_distance = float(
            self.get_parameter('return_a_desired_front_distance').value)
        self.return_a_align_timeout_sec = float(
            self.get_parameter('return_a_align_timeout_sec').value)
        self.return_a_pre_align_enabled = bool(
            self.get_parameter('return_a_pre_align_enabled').value)
        self.return_a_pre_align_backoff_distance = abs(float(
            self.get_parameter('return_a_pre_align_backoff_distance').value))
        self.return_a_skip_final_trim_when_lidar = bool(
            self.get_parameter('return_a_skip_final_trim_when_lidar').value)
        self.return_a_alignment_mode = str(
            self.get_parameter('return_a_alignment_mode').value).lower()
        self.return_a_fallback_to_front_band = bool(
            self.get_parameter('return_a_fallback_to_front_band').value)
        self.return_a_use_front_band_alignment = bool(
            self.get_parameter('return_a_use_front_band_alignment').value)
        self.return_a_front_band_width = abs(float(
            self.get_parameter('return_a_front_band_width').value))
        self.return_a_roi_x_min = float(
            self.get_parameter('return_a_roi_x_min').value)
        self.return_a_roi_x_max = float(
            self.get_parameter('return_a_roi_x_max').value)
        self.return_a_roi_y_abs = abs(float(
            self.get_parameter('return_a_roi_y_abs').value))
        self.return_a_leg_use_front_band = bool(
            self.get_parameter('return_a_leg_use_front_band').value)
        self.return_a_leg_cluster_gap = abs(float(
            self.get_parameter('return_a_leg_cluster_gap').value))
        self.return_a_leg_min_cluster_points = int(
            self.get_parameter('return_a_leg_min_cluster_points').value)
        self.return_a_leg_max_cluster_width_y = abs(float(
            self.get_parameter('return_a_leg_max_cluster_width_y').value))
        self.return_a_leg_max_cluster_depth_x = abs(float(
            self.get_parameter('return_a_leg_max_cluster_depth_x').value))
        self.return_a_leg_min_spacing = abs(float(
            self.get_parameter('return_a_leg_min_spacing').value))
        self.return_a_leg_expected_count = int(
            self.get_parameter('return_a_leg_expected_count').value)
        self.return_a_leg_use_yaw = bool(
            self.get_parameter('return_a_leg_use_yaw').value)
        self.return_a_debug_markers = bool(
            self.get_parameter('return_a_debug_markers').value)
        self.return_a_debug_marker_topic = str(
            self.get_parameter('return_a_debug_marker_topic').value)
        self.return_a_debug_marker_lifetime_sec = float(
            self.get_parameter('return_a_debug_marker_lifetime_sec').value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        waypoint_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, qos)
        self.nav_state_pub = self.create_publisher(
            String, self.nav_state_topic, event_qos)
        self.arrived_a_pub = self.create_publisher(
            Bool, self.arrived_a_topic, event_qos)
        self.arrived_b_pub = self.create_publisher(
            Bool, self.arrived_b_topic, event_qos)
        self.reached_b_stop_line_pub = self.create_publisher(
            Bool, self.reached_b_stop_line_topic, event_qos)
        self.reached_b_place_pose_pub = self.create_publisher(
            Bool, self.reached_b_place_pose_topic, event_qos)
        self.failure_reason_pub = self.create_publisher(
            String, self.failure_reason_topic, event_qos)
        self.a_pose_pub = self.create_publisher(
            PoseStamped, '/mission_b/primitive/a_pose', waypoint_qos)
        self.b_pose_pub = self.create_publisher(
            PoseStamped, '/mission_b/primitive/b_pose', waypoint_qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/mission_b/primitive/waypoints', waypoint_qos)
        self.return_a_debug_marker_pub = self.create_publisher(
            MarkerArray, self.return_a_debug_marker_topic, 1)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, qos)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, qos)
        self.return_allowed_sub = self.create_subscription(
            Bool, self.return_allowed_topic, self._return_allowed_callback, qos)
        self.b_approach_allowed_sub = self.create_subscription(
            Bool,
            self.b_approach_allowed_topic,
            self._b_approach_allowed_callback,
            qos)

        self.current_pose = None
        self.latest_scan = None
        self.done = False
        self.done_time = None
        self.state = 'waiting'
        self.segment_index = 0
        self.segment_start_pose = None
        self.segment_start_time = None
        self.route_start_pose = None
        self.b_arrival_pose = None
        self.stop_cycles_remaining = 0
        self.align_start_time = None
        self.alignment_target = 'B'
        self.alignment_desired_front_distance = self.desired_front_distance
        self.alignment_timeout_sec = self.align_timeout_sec
        self.stable_cycles = 0
        self.pause_until = None
        self.pause_after_a_until = None
        self.b_approach_allowed_received = False
        self.b_stop_line_reached_time = None
        self.wait_for_return_allowed_start_time = None
        self.return_allowed_received = False
        self.return_start_time = None
        self.return_segments = []
        self.return_segments_prepared = False
        self.return_segment_index = 0
        self.return_trim_segments = []
        self.return_trim_segment_index = 0
        self.return_trim_segments_prepared = False

        self.segments = []
        if self.backward_distance > 0.0:
            self.segments.append({
                'name': 'backward',
                'distance': self.backward_distance,
                'speed': self.linear_speed,
            })
        if self.right_distance > 0.0:
            self.segments.append({
                'name': 'right',
                'distance': self.right_distance,
                'speed': self.lateral_speed,
            })
        if self.forward_distance > 0.0:
            self.segments.append({
                'name': 'forward',
                'distance': self.forward_distance,
                'speed': self.forward_speed,
            })

        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self._timer_callback)
        self.waypoint_timer = self.create_timer(1.0, self._publish_waypoint_poses)

        self.get_logger().info(
            'Mission B primitive route ready: '
            f'backward={self.backward_distance:.2f}m, '
            f'right={self.right_distance:.2f}m, '
            f'forward={self.forward_distance:.2f}m, '
            f'min_command_speed={self.min_command_speed:.2f}m/s, '
            f'linear_speed={self.linear_speed:.2f}m/s, '
            f'lateral_speed={self.lateral_speed:.2f}m/s, '
            f'forward_speed={self.forward_speed:.2f}m/s, '
            f'b_approach={self.b_approach_forward_distance:.2f}m, '
            f'hold_heading={self.hold_heading_during_segments}, '
            f'pause_after_a={self.pause_after_a_mark_sec:.1f}s, '
            f'return_mode={self.return_mode}, '
            f'wait_for_b_approach={self.wait_for_b_approach_command}, '
            f'wait_for_return_allowed={self.wait_for_return_allowed}, '
            f'lidar_alignment={self.enable_lidar_alignment}, '
            f'return_a_lidar_alignment={self.enable_return_a_lidar_alignment}, '
            f'cmd_vel_topic={self.cmd_vel_topic}')
        self._publish_arrived_a(False)
        self._publish_arrived_b(False)
        self._publish_reached_b_stop_line(False)
        self._publish_reached_b_place_pose(False)
        self._publish_failure_reason('')
        self._publish_nav_state('READY')
        self.get_logger().info(
            'Safe A return enabled: '
            f'pre_align={self.return_a_pre_align_enabled}, '
            f'backoff={self.return_a_pre_align_backoff_distance:.2f}m, '
            f'a_alignment_mode={self.return_a_alignment_mode}, '
            f'fallback_front_band={self.return_a_fallback_to_front_band}, '
            f'roi_y_abs={self.return_a_roi_y_abs:.2f}m, '
            f'leg_cluster_gap={self.return_a_leg_cluster_gap:.2f}m, '
            f'debug_markers={self.return_a_debug_markers} '
            f'({self.return_a_debug_marker_topic})')

    def wait_for_cmd_subscriber(self):
        deadline = time.monotonic() + self.wait_for_subscriber_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.cmd_pub.get_subscription_count() > 0:
                self.get_logger().info(
                    f'Found {self.cmd_pub.get_subscription_count()} subscriber(s) '
                    f'on {self.cmd_vel_topic}.')
                return
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().warning(
            f'No subscribers discovered on {self.cmd_vel_topic}. '
            'Command will still be published.')

    def _odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w)
        self.current_pose = (position.x, position.y, yaw)

    def _scan_callback(self, msg):
        self.latest_scan = msg

    def _return_allowed_callback(self, msg):
        if msg.data:
            self.return_allowed_received = True

    def _b_approach_allowed_callback(self, msg):
        if msg.data:
            self.b_approach_allowed_received = True

    def _timer_callback(self):
        if self.done:
            return

        if self.current_pose is None:
            self.get_logger().warn(
                f'Waiting for odom on {self.odom_topic}...',
                throttle_duration_sec=1.0)
            return

        if self.state == 'waiting':
            self._start_next_step()
        elif self.state == 'pause_after_a_mark':
            self._pause_after_a_mark_step()
        elif self.state == 'segment':
            self._segment_step()
        elif self.state == 'settling':
            self._settling_step()
        elif self.state == 'wait_for_b_approach_command_at_b':
            self._wait_for_b_approach_command_at_b_step()
        elif self.state == 'b_approach_segment':
            self._b_approach_segment_step()
        elif self.state == 'b_approach_settling':
            self._b_approach_settling_step()
        elif self.state == 'aligning':
            self._alignment_step()
        elif self.state == 'pause_at_b':
            self._pause_at_b_step()
        elif self.state == 'wait_for_return_allowed_at_b':
            self._wait_for_return_allowed_at_b_step()
        elif self.state == 'return_yaw_to_start':
            self._return_yaw_to_start_step()
        elif self.state == 'return_segment':
            self._return_segment_step()
        elif self.state == 'return_settling':
            self._return_settling_step()
        elif self.state == 'return_trim_segment':
            self._return_trim_segment_step()
        elif self.state == 'return_trim_settling':
            self._return_trim_settling_step()
        elif self.state == 'return_final_yaw_to_start':
            self._return_final_yaw_to_start_step()
        elif self.state == 'returning_to_a':
            self._return_to_a_step()

    def _start_next_step(self):
        if self.route_start_pose is None:
            self.route_start_pose = self.current_pose
            self._publish_waypoint_poses()
            self._publish_nav_state('MARKED_A')
            self.cmd_pub.publish(Twist())
            if self.pause_after_a_mark_sec > 0.0:
                self.pause_after_a_until = (
                    time.monotonic() + self.pause_after_a_mark_sec)
                self.state = 'pause_after_a_mark'
                self.get_logger().info(
                    'Marked A pose. Waiting '
                    f'{self.pause_after_a_mark_sec:.1f}s before moving: '
                    f'x={self.route_start_pose[0]:+.3f}, '
                    f'y={self.route_start_pose[1]:+.3f}, '
                    f'yaw={self.route_start_pose[2]:+.3f}')
                return
            self.get_logger().info(
                'Marked A pose. Starting route on next step: '
                f'x={self.route_start_pose[0]:+.3f}, '
                f'y={self.route_start_pose[1]:+.3f}, '
                f'yaw={self.route_start_pose[2]:+.3f}')
            return

        if self.segment_index < len(self.segments):
            segment = self.segments[self.segment_index]
            self.segment_start_pose = self.current_pose
            self.segment_start_time = time.monotonic()
            self.state = 'segment'
            self._publish_nav_state('MOVING_TO_B')
            self.get_logger().info(
                f'Start segment {self.segment_index + 1}/{len(self.segments)}: '
                f'{segment["name"]} {segment["distance"]:.2f}m '
                f'at {segment["speed"]:.2f}m/s')
            return

        if self.wait_for_b_approach_command:
            if self.enable_lidar_alignment:
                self._start_lidar_alignment(
                    target='B_STOP_LINE',
                    desired_front_distance=self.desired_front_distance,
                    timeout_sec=self.align_timeout_sec)
            else:
                self._arrive_at_b_stop_line()
            return

        if self.enable_lidar_alignment:
            self._start_lidar_alignment(
                target='B',
                desired_front_distance=self.desired_front_distance,
                timeout_sec=self.align_timeout_sec)
            return

        self._arrive_at_b('route_complete_without_lidar_alignment')

    def _pause_after_a_mark_step(self):
        self.cmd_pub.publish(Twist())
        self._publish_waypoint_poses()
        if time.monotonic() >= self.pause_after_a_until:
            self.pause_after_a_until = None
            self.state = 'waiting'

    def _publish_waypoint_poses(self):
        if not self.publish_waypoint_poses or self.route_start_pose is None:
            return

        start_x, start_y, start_yaw = self.route_start_pose

        self.a_pose_pub.publish(
            self._make_pose_stamped(start_x, start_y, start_yaw))
        if self.b_arrival_pose is not None:
            b_x, b_y, b_yaw = self.b_arrival_pose
            self.b_pose_pub.publish(
                self._make_pose_stamped(b_x, b_y, b_yaw))

        self.marker_pub.publish(self._make_waypoint_markers())

    def _make_pose_stamped(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id = self.waypoint_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation = yaw_to_quaternion(yaw)
        return msg

    def _make_waypoint_markers(self):
        a_x, a_y, a_yaw = self.route_start_pose
        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.extend([
            self._make_sphere_marker(0, 'A', a_x, a_y, now, 0.0, 0.4, 1.0),
            self._make_text_marker(1, 'A', a_x, a_y, now),
            self._make_arrow_marker(2, a_x, a_y, a_yaw, now, 0.0, 0.4, 1.0),
        ])
        if self.b_arrival_pose is not None:
            b_x, b_y, b_yaw = self.b_arrival_pose
            markers.markers.extend([
                self._make_sphere_marker(3, 'B', b_x, b_y, now, 0.0, 0.9, 0.2),
                self._make_text_marker(4, 'B', b_x, b_y, now),
                self._make_arrow_marker(5, b_x, b_y, b_yaw, now, 0.0, 0.9, 0.2),
            ])
        else:
            markers.markers.extend([
                self._make_delete_marker(3, 'mission_b_B_point', now),
                self._make_delete_marker(4, 'mission_b_B_label', now),
                self._make_delete_marker(5, 'mission_b_heading', now),
            ])
        return markers

    def _make_base_marker(self, marker_id, marker_type, x, y, stamp):
        marker = Marker()
        marker.header.frame_id = self.waypoint_frame
        marker.header.stamp = stamp
        marker.ns = 'mission_b_primitive_waypoints'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1.0
        marker.color.a = 1.0
        return marker

    def _make_sphere_marker(self, marker_id, label, x, y, stamp, red, green, blue):
        marker = self._make_base_marker(marker_id, Marker.SPHERE, x, y, stamp)
        marker.ns = f'mission_b_{label}_point'
        marker.pose.position.z = 0.08
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.08
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        return marker

    def _make_text_marker(self, marker_id, text, x, y, stamp):
        marker = self._make_base_marker(marker_id, Marker.TEXT_VIEW_FACING, x, y, stamp)
        marker.ns = f'mission_b_{text}_label'
        marker.pose.position.z = 0.35
        marker.scale.z = 0.28
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.text = text
        return marker

    def _make_arrow_marker(self, marker_id, x, y, yaw, stamp, red, green, blue):
        marker = self._make_base_marker(marker_id, Marker.ARROW, x, y, stamp)
        marker.ns = 'mission_b_heading'
        marker.pose.position.z = 0.12
        marker.pose.orientation = yaw_to_quaternion(yaw)
        marker.scale.x = 0.45
        marker.scale.y = 0.06
        marker.scale.z = 0.06
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        return marker

    def _make_delete_marker(self, marker_id, namespace, stamp):
        marker = Marker()
        marker.header.frame_id = self.waypoint_frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _publish_nav_state(self, state):
        msg = String()
        msg.data = state
        self.nav_state_pub.publish(msg)

    def _publish_arrived_a(self, arrived):
        msg = Bool()
        msg.data = arrived
        self.arrived_a_pub.publish(msg)

    def _publish_arrived_b(self, arrived):
        msg = Bool()
        msg.data = arrived
        self.arrived_b_pub.publish(msg)

    def _publish_reached_b_stop_line(self, reached):
        msg = Bool()
        msg.data = reached
        self.reached_b_stop_line_pub.publish(msg)

    def _publish_reached_b_place_pose(self, reached):
        msg = Bool()
        msg.data = reached
        self.reached_b_place_pose_pub.publish(msg)

    def _publish_failure_reason(self, reason):
        msg = String()
        msg.data = reason
        self.failure_reason_pub.publish(msg)

    def _segment_step(self):
        segment = self.segments[self.segment_index]
        elapsed = time.monotonic() - self.segment_start_time
        forward_delta, left_delta = self._compute_delta_from_segment_start()
        progress = self._segment_progress(segment['name'], forward_delta, left_delta)

        if progress >= segment['distance']:
            self.get_logger().info(
                f'Segment done: {segment["name"]}, elapsed={elapsed:.2f}s, '
                f'progress={progress:.3f}m, '
                f'forward_delta={forward_delta:+.3f}m, '
                f'left_delta={left_delta:+.3f}m')
            self._begin_settling()
            return

        if progress <= -self.wrong_direction_tolerance:
            self._fail(
                f'wrong_direction_detected in {segment["name"]}: '
                f'progress={progress:+.3f}m')
            return

        if elapsed >= self.max_segment_duration_sec:
            self._fail(
                f'segment_timeout in {segment["name"]}: '
                f'elapsed={elapsed:.1f}s, progress={progress:.3f}m')
            return

        cmd = self._make_segment_command(segment['name'], segment['speed'])
        if cmd is None:
            self._fail(f'unknown segment name: {segment["name"]}')
            return

        yaw_error = self._apply_segment_heading_hold(cmd)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'{segment["name"]}: progress={progress:.3f}m / '
            f'{segment["distance"]:.3f}m, yaw_err={yaw_error:+.3f}rad, '
            f'cmd_wz={cmd.angular.z:+.2f}',
            throttle_duration_sec=0.5)

    def _segment_progress(self, name, forward_delta, left_delta):
        if name == 'backward':
            return -forward_delta
        if name == 'forward':
            return forward_delta
        if name == 'right':
            return -left_delta
        if name == 'left':
            return left_delta
        return 0.0

    def _make_segment_command(self, name, speed):
        cmd = Twist()
        if name == 'backward':
            cmd.linear.x = -speed
        elif name == 'forward':
            cmd.linear.x = speed
        elif name == 'right':
            cmd.linear.y = -speed
        elif name == 'left':
            cmd.linear.y = speed
        else:
            return None
        return cmd

    def _apply_segment_heading_hold(self, cmd):
        if (
            not self.hold_heading_during_segments or
            self.segment_start_pose is None or
            self.current_pose is None
        ):
            return 0.0

        _, _, target_yaw = self._heading_hold_target_pose()
        _, _, current_yaw = self.current_pose
        yaw_error = normalize_angle(target_yaw - current_yaw)
        if abs(yaw_error) > self.heading_hold_tolerance:
            cmd.angular.z = clamp(
                self.heading_hold_kz * yaw_error,
                -self.heading_hold_max_wz,
                self.heading_hold_max_wz)
        return yaw_error

    def _heading_hold_target_pose(self):
        if self.route_start_pose is not None:
            if self.state == 'segment':
                return self.route_start_pose
            if self.state == 'return_segment' and self.return_match_start_yaw:
                return self.route_start_pose
            if self.state == 'return_trim_segment' and self.return_match_start_yaw:
                return self.route_start_pose
        return self.segment_start_pose

    def _begin_settling(self):
        self.cmd_pub.publish(Twist())
        self.stop_cycles_remaining = int(max(5, self.rate_hz * 0.5))
        self.state = 'settling'

    def _settling_step(self):
        self.cmd_pub.publish(Twist())
        self.stop_cycles_remaining -= 1
        if self.stop_cycles_remaining <= 0:
            self.segment_index += 1
            self.state = 'waiting'

    def _alignment_step(self):
        elapsed = time.monotonic() - self.align_start_time
        if elapsed > self.alignment_timeout_sec:
            self._fail(
                f'lidar_alignment_timeout target={self.alignment_target} '
                f'after {elapsed:.1f}s')
            return

        detection = self._detect_table_from_scan()
        if detection is None:
            self.cmd_pub.publish(Twist())
            self.stable_cycles = 0
            self.get_logger().warn(
                f'No stable LiDAR cluster in front ROI for {self.alignment_target}.',
                throttle_duration_sec=1.0)
            return

        front_distance, center_y, yaw_error, visible_width, point_count = detection
        front_error = front_distance - self.alignment_desired_front_distance
        lateral_error = center_y

        cmd = Twist()
        phase = 'verify'

        if abs(lateral_error) > self.lateral_tolerance:
            phase = 'center'
            cmd.linear.y = clamp_with_min_abs(
                self.ky * lateral_error, self.max_vy, self.min_center_vy)
        elif abs(front_error) > self.front_tolerance:
            phase = 'distance'
            cmd.linear.x = clamp_with_min_abs(
                self.kx * front_error, self.max_vx, self.min_align_vx)
        elif abs(yaw_error) > self.yaw_tolerance:
            phase = 'yaw'
            cmd.angular.z = clamp_with_min_abs(
                -self.kz * yaw_error, self.max_wz, self.min_yaw_wz)

        aligned = (
            abs(front_error) <= self.front_tolerance and
            abs(lateral_error) <= self.lateral_tolerance and
            abs(yaw_error) <= self.yaw_tolerance
        )

        self.stable_cycles = self.stable_cycles + 1 if aligned else 0
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            'LiDAR align: '
            f'target={self.alignment_target}, '
            f'phase={phase}, '
            f'front={front_distance:.3f}m err={front_error:+.3f}, '
            f'center_y={center_y:+.3f}m, yaw_err={yaw_error:+.3f}rad, '
            f'width={visible_width:.3f}m, points={point_count}, '
            f'cmd=({cmd.linear.x:+.2f}, {cmd.linear.y:+.2f}, '
            f'{cmd.angular.z:+.2f}), stable={self.stable_cycles}/'
            f'{self.stable_cycles_required}',
            throttle_duration_sec=0.5)

        if self.stable_cycles >= self.stable_cycles_required:
            if self.alignment_target == 'B_STOP_LINE':
                self._arrive_at_b_stop_line()
            elif self.alignment_target == 'A':
                self._finish('returned_to_a_lidar_alignment_complete')
            else:
                self._arrive_at_b('lidar_alignment_complete')

    def _arrive_at_b_stop_line(self):
        self.cmd_pub.publish(Twist())
        self.b_approach_allowed_received = False
        self.b_stop_line_reached_time = time.monotonic()
        self.state = 'wait_for_b_approach_command_at_b'
        self._publish_reached_b_stop_line(True)
        self._publish_nav_state('REACHED_B_STOP_LINE')
        self.get_logger().info(
            'Reached B stop line. Waiting for B approach allowance on '
            f'{self.b_approach_allowed_topic}.')

    def _wait_for_b_approach_command_at_b_step(self):
        self.cmd_pub.publish(Twist())
        self._publish_waypoint_poses()
        self._publish_reached_b_stop_line(True)
        self._publish_nav_state('WAITING_FOR_B_APPROACH')

        if self.b_approach_allowed_received:
            self.get_logger().info(
                'B approach allowance received. Starting final approach to B.')
            self._start_b_approach_segment()
            return

        if self.b_approach_allowed_timeout_sec <= 0.0:
            return

        elapsed = time.monotonic() - self.b_stop_line_reached_time
        if elapsed >= self.b_approach_allowed_timeout_sec:
            self._fail(
                f'b_approach_allowed_timeout after {elapsed:.1f}s on '
                f'{self.b_approach_allowed_topic}')

    def _start_b_approach_segment(self):
        if self.b_approach_forward_distance <= self.return_position_tolerance:
            self._arrive_at_b('b_approach_distance_zero')
            return

        self.segment_start_pose = self.current_pose
        self.segment_start_time = time.monotonic()
        self.state = 'b_approach_segment'
        self._publish_nav_state('APPROACHING_B_PLACE_POSE')
        self.get_logger().info(
            'Start B final approach segment: '
            f'forward {self.b_approach_forward_distance:.2f}m '
            f'at {self.b_approach_speed:.2f}m/s')

    def _b_approach_segment_step(self):
        elapsed = time.monotonic() - self.segment_start_time
        forward_delta, left_delta = self._compute_delta_from_segment_start()

        if forward_delta >= self.b_approach_forward_distance:
            self.get_logger().info(
                'B final approach segment done: '
                f'elapsed={elapsed:.2f}s, '
                f'forward_delta={forward_delta:+.3f}m, '
                f'left_delta={left_delta:+.3f}m')
            self.cmd_pub.publish(Twist())
            self.stop_cycles_remaining = int(max(5, self.rate_hz * 0.5))
            self.state = 'b_approach_settling'
            return

        if forward_delta <= -self.wrong_direction_tolerance:
            self._fail(
                'wrong_direction_detected in B final approach: '
                f'forward_delta={forward_delta:+.3f}m')
            return

        if elapsed >= self.max_segment_duration_sec:
            self._fail(
                'b_approach_segment_timeout: '
                f'elapsed={elapsed:.1f}s, progress={forward_delta:.3f}m')
            return

        cmd = Twist()
        cmd.linear.x = self.b_approach_speed
        yaw_error = self._apply_segment_heading_hold(cmd)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            'B final approach: '
            f'progress={forward_delta:.3f}m / '
            f'{self.b_approach_forward_distance:.3f}m, '
            f'yaw_err={yaw_error:+.3f}rad, cmd_wz={cmd.angular.z:+.2f}',
            throttle_duration_sec=0.5)

    def _b_approach_settling_step(self):
        self.cmd_pub.publish(Twist())
        self.stop_cycles_remaining -= 1
        if self.stop_cycles_remaining <= 0:
            self._arrive_at_b('b_approach_forward_complete')

    def _start_lidar_alignment(self, target, desired_front_distance, timeout_sec):
        self.align_start_time = time.monotonic()
        self.alignment_target = target
        self.alignment_desired_front_distance = desired_front_distance
        self.alignment_timeout_sec = timeout_sec
        self.stable_cycles = 0
        self.state = 'aligning'
        if target == 'A':
            self._publish_nav_state('ALIGNING_AT_A')
        else:
            self._publish_nav_state('ALIGNING_AT_B')
        self.get_logger().info(
            'Start LiDAR final alignment: '
            f'target={target}, '
            f'desired_front_distance={desired_front_distance:.2f}m, '
            f'timeout={timeout_sec:.1f}s')

    def _arrive_at_b(self, reason):
        self.cmd_pub.publish(Twist())
        if self.b_arrival_pose is None:
            self.b_arrival_pose = self.current_pose
            self.get_logger().info(
                'Mission B arrived at B: '
                f'reason={reason}, '
                f'x={self.b_arrival_pose[0]:+.3f}, '
                f'y={self.b_arrival_pose[1]:+.3f}, '
                f'yaw={self.b_arrival_pose[2]:+.3f}')
        self._publish_waypoint_poses()
        self._publish_arrived_b(True)
        self._publish_reached_b_place_pose(True)
        self._publish_nav_state('ARRIVED_B')

        if self.return_to_start:
            if self.wait_for_return_allowed:
                self.return_allowed_received = False
                self.wait_for_return_allowed_start_time = time.monotonic()
                self.state = 'wait_for_return_allowed_at_b'
                self._publish_nav_state('WAITING_FOR_RETURN_ALLOWED')
                self.get_logger().info(
                    f'Waiting for return allowance on {self.return_allowed_topic}.')
                return
            if self.pause_at_b_sec > 0.0:
                self.pause_until = time.monotonic() + self.pause_at_b_sec
                self.state = 'pause_at_b'
                self.get_logger().info(
                    f'Paused at B for {self.pause_at_b_sec:.1f}s before return.')
            else:
                self._start_return_to_a()
            return

        self._finish(reason)

    def _wait_for_return_allowed_at_b_step(self):
        self.cmd_pub.publish(Twist())
        self._publish_waypoint_poses()
        self._publish_nav_state('WAITING_FOR_RETURN_ALLOWED')

        if self.return_allowed_received:
            self.get_logger().info('Return allowance received. Starting return to A.')
            if self.pause_at_b_sec > 0.0:
                self.pause_until = time.monotonic() + self.pause_at_b_sec
                self.state = 'pause_at_b'
                self.get_logger().info(
                    f'Paused at B for {self.pause_at_b_sec:.1f}s before return.')
            else:
                self._start_return_to_a()
            return

        if self.return_allowed_timeout_sec <= 0.0:
            return

        elapsed = time.monotonic() - self.wait_for_return_allowed_start_time
        if elapsed >= self.return_allowed_timeout_sec:
            self._fail(
                f'return_allowed_timeout after {elapsed:.1f}s on '
                f'{self.return_allowed_topic}')

    def _pause_at_b_step(self):
        self.cmd_pub.publish(Twist())
        self._publish_waypoint_poses()
        if time.monotonic() >= self.pause_until:
            self._start_return_to_a()

    def _start_return_to_a(self):
        if self.route_start_pose is None:
            self._fail('return_to_start_requested_without_a_pose')
            return

        self.return_start_time = time.monotonic()
        self.stable_cycles = 0
        self.return_segment_index = 0
        self.return_segments = []
        self.return_segments_prepared = False
        self.return_trim_segment_index = 0
        self.return_trim_segments = []
        self.return_trim_segments_prepared = False

        if self.return_mode == 'odom_target':
            self.state = 'returning_to_a'
            self._publish_nav_state('RETURNING_TO_A')
            self.get_logger().info(
                'Start return to A using saved A pose target: '
                f'x={self.route_start_pose[0]:+.3f}, '
                f'y={self.route_start_pose[1]:+.3f}, '
                f'yaw={self.route_start_pose[2]:+.3f}')
            return

        if self.return_mode != 'reverse_segments':
            self._fail(f'unsupported_return_mode: {self.return_mode}')
            return

        if self.return_match_start_yaw:
            self.state = 'return_yaw_to_start'
        else:
            self._prepare_reverse_return_segments()
            self._start_next_return_segment()

        self._publish_nav_state('RETURNING_TO_A')
        self.get_logger().info(
            'Start return to A using computed reverse straight segments: '
            f'match_start_yaw={self.return_match_start_yaw}')

    def _use_return_a_pre_align(self):
        return (
            self.return_a_pre_align_enabled and
            self.enable_return_a_lidar_alignment and
            self.return_a_pre_align_backoff_distance > 0.0
        )

    def _return_a_target_pose(self):
        start_x, start_y, start_yaw = self.route_start_pose
        if not self._use_return_a_pre_align():
            return start_x, start_y, start_yaw

        target_x = (
            start_x -
            math.cos(start_yaw) * self.return_a_pre_align_backoff_distance
        )
        target_y = (
            start_y -
            math.sin(start_yaw) * self.return_a_pre_align_backoff_distance
        )
        return target_x, target_y, start_yaw

    def _prepare_reverse_return_segments(self):
        if self.return_segments_prepared:
            return

        target_x, target_y, _ = self._return_a_target_pose()
        current_x, current_y, current_yaw = self.current_pose
        dx = target_x - current_x
        dy = target_y - current_y
        forward_error = math.cos(current_yaw) * dx + math.sin(current_yaw) * dy
        left_error = -math.sin(current_yaw) * dx + math.cos(current_yaw) * dy
        backoff_distance = self.forward_distance
        distance_tolerance = self.return_position_tolerance * 0.5

        if backoff_distance > distance_tolerance:
            self.return_segments.append({
                'name': 'backward',
                'distance': backoff_distance,
                'speed': self.forward_speed,
            })

        if abs(left_error) > distance_tolerance:
            self.return_segments.append({
                'name': 'left' if left_error > 0.0 else 'right',
                'distance': abs(left_error),
                'speed': self.lateral_speed,
            })

        final_forward_error = forward_error + backoff_distance
        if abs(final_forward_error) > distance_tolerance:
            self.return_segments.append({
                'name': 'forward' if final_forward_error > 0.0 else 'backward',
                'distance': abs(final_forward_error),
                'speed': self.linear_speed,
            })

        self.return_segments_prepared = True
        self.get_logger().info(
            'Computed safe reverse return segments: '
            f'target_x={target_x:+.3f}, target_y={target_y:+.3f}, '
            f'pre_align={self._use_return_a_pre_align()}, '
            f'forward_error={forward_error:+.3f}m, '
            f'left_error={left_error:+.3f}m, '
            f'backoff={backoff_distance:.3f}m, '
            f'segments={[(s["name"], round(s["distance"], 3)) for s in self.return_segments]}')

    def _start_return_final_trim_or_yaw(self):
        if (
            self._use_return_a_pre_align() and
            self.return_a_skip_final_trim_when_lidar
        ):
            self.get_logger().info(
                'Skipping exact A-pose final trim. '
                'Starting yaw/LiDAR alignment from safe pre-align pose.')
            if self.return_match_start_yaw:
                self.state = 'return_final_yaw_to_start'
                self.get_logger().info('Start final yaw alignment before A LiDAR docking.')
            else:
                self._finish_return_to_a('returned_to_a_pre_align_pose')
            return

        if self.enable_return_final_trim:
            self._prepare_return_final_trim_segments()
            if self.return_trim_segment_index < len(self.return_trim_segments):
                self._start_next_return_trim_segment()
                return

        if self.return_match_start_yaw:
            self.state = 'return_final_yaw_to_start'
            self.get_logger().info('Start final yaw alignment at A.')
        else:
            self._finish_return_to_a('returned_to_a_reverse_segments')

    def _detect_table_from_scan(self):
        if self.alignment_target == 'A':
            if self.return_a_alignment_mode == 'legs':
                detection = self._detect_a_conveyor_legs_from_scan()
                if detection is not None:
                    return detection
                if self.return_a_fallback_to_front_band:
                    self.get_logger().warn(
                        'A conveyor leg alignment failed; falling back to '
                        'front-band alignment.',
                        throttle_duration_sec=1.0)
                    return self._detect_a_conveyor_from_scan()
                return None

            if (
                self.return_a_alignment_mode == 'front_band' or
                self.return_a_use_front_band_alignment
            ):
                return self._detect_a_conveyor_from_scan()
        return self._detect_b_table_from_scan()

    def _detect_b_table_from_scan(self):
        scan = self.latest_scan
        if scan is None:
            return None

        points = self._scan_to_roi_points(scan)
        if len(points) < self.min_points:
            return None

        x_values = [point[0] for point in points]
        front_x = percentile(x_values, 0.10)
        front_points = [
            point for point in points
            if point[0] <= front_x + self.front_band_width
        ]

        if len(front_points) < self.min_points:
            front_points = points

        front_x_values = [point[0] for point in front_points]
        front_distance = percentile(front_x_values, 0.20)

        y_values = [point[1] for point in front_points]
        y_low = percentile(y_values, 0.10)
        y_high = percentile(y_values, 0.90)
        visible_width = y_high - y_low

        if visible_width >= self.min_visible_width:
            center_y = (y_low + y_high) * 0.5
        else:
            center_y = statistics.median(y_values)

        yaw_error = self._estimate_front_edge_yaw(front_points)
        return front_distance, center_y, yaw_error, visible_width, len(front_points)

    def _detect_a_conveyor_legs_from_scan(self):
        scan = self.latest_scan
        if scan is None:
            return None

        points = self._scan_to_return_a_roi_points(scan)
        if len(points) < self.return_a_leg_expected_count:
            return None

        active_points = self._select_return_a_front_band(points)
        if not self.return_a_leg_use_front_band:
            active_points = points

        clusters = self._cluster_return_a_points(active_points)
        candidates = self._filter_return_a_leg_candidates(clusters)
        selected = self._select_return_a_leg_triplet(candidates)
        self._publish_return_a_leg_markers(
            scan.header, points, active_points, candidates, selected)
        if len(selected) < self.return_a_leg_expected_count:
            self.get_logger().warn(
                'A conveyor leg alignment waiting for leg candidates: '
                f'candidates={len(candidates)}/'
                f'{self.return_a_leg_expected_count}, '
                f'roi_points={len(points)}, active_points={len(active_points)}',
                throttle_duration_sec=1.0)
            return None

        front_distance = statistics.mean([candidate['x'] for candidate in selected])
        center_y = statistics.mean([candidate['y'] for candidate in selected])
        point_count = sum(candidate['count'] for candidate in selected)
        y_values = [candidate['y'] for candidate in selected]
        visible_width = max(y_values) - min(y_values) if len(y_values) > 1 else 0.0

        yaw_error = 0.0
        if self.return_a_leg_use_yaw:
            yaw_points = [(candidate['x'], candidate['y']) for candidate in selected]
            yaw_error = self._estimate_front_edge_yaw(yaw_points)

        self.get_logger().info(
            'A conveyor leg alignment detection: '
            f'front={front_distance:.3f}m, center_y={center_y:+.3f}m, '
            f'width={visible_width:.3f}m, selected={len(selected)}, '
            f'points={point_count}',
            throttle_duration_sec=0.5)

        return (
            front_distance,
            center_y,
            yaw_error,
            visible_width,
            point_count,
        )

    def _select_return_a_front_band(self, points):
        if not points:
            return []
        x_values = [point[0] for point in points]
        front_x = percentile(x_values, 0.15)
        front_limit = front_x + self.return_a_front_band_width
        return [point for point in points if point[0] <= front_limit]

    def _cluster_return_a_points(self, points):
        remaining = list(points)
        clusters = []
        while remaining:
            seed = remaining.pop(0)
            cluster = [seed]
            queue = [seed]

            while queue:
                current = queue.pop(0)
                next_remaining = []
                for point in remaining:
                    if self._distance(current, point) <= self.return_a_leg_cluster_gap:
                        cluster.append(point)
                        queue.append(point)
                    else:
                        next_remaining.append(point)
                remaining = next_remaining

            clusters.append(cluster)
        return clusters

    def _filter_return_a_leg_candidates(self, clusters):
        candidates = []
        for cluster in clusters:
            if len(cluster) < self.return_a_leg_min_cluster_points:
                continue

            x_values = [point[0] for point in cluster]
            y_values = [point[1] for point in cluster]
            width_y = max(y_values) - min(y_values)
            depth_x = max(x_values) - min(x_values)
            if width_y > self.return_a_leg_max_cluster_width_y:
                continue
            if depth_x > self.return_a_leg_max_cluster_depth_x:
                continue

            candidates.append({
                'x': statistics.median(x_values),
                'y': statistics.median(y_values),
                'width_y': width_y,
                'depth_x': depth_x,
                'count': len(cluster),
            })

        return sorted(candidates, key=lambda item: (item['y'], item['x']))

    def _select_return_a_leg_triplet(self, candidates):
        expected_count = self.return_a_leg_expected_count
        if len(candidates) <= expected_count:
            return candidates

        best = []
        best_score = None
        for group in combinations(candidates, expected_count):
            pair_distances = [
                self._distance((first['x'], first['y']), (second['x'], second['y']))
                for first, second in combinations(group, 2)
            ]
            if min(pair_distances) < self.return_a_leg_min_spacing:
                continue

            line_error = self._line_error(group)
            center_y = abs(statistics.mean([item['y'] for item in group]))
            span = max(pair_distances)
            total_points = sum(item['count'] for item in group)
            score = line_error + center_y * 0.02 - span * 0.01 - total_points * 0.001
            if best_score is None or score < best_score:
                best_score = score
                best = list(group)

        if best:
            return sorted(best, key=lambda item: (item['y'], item['x']))
        return candidates[:expected_count]

    @staticmethod
    def _distance(first, second):
        return math.hypot(first[0] - second[0], first[1] - second[1])

    @staticmethod
    def _line_error(group):
        if len(group) < 3:
            return 0.0

        x_values = [item['x'] for item in group]
        y_values = [item['y'] for item in group]
        mean_x = statistics.mean(x_values)
        mean_y = statistics.mean(y_values)
        var_x = statistics.mean([(x - mean_x) ** 2 for x in x_values])
        var_y = statistics.mean([(y - mean_y) ** 2 for y in y_values])
        cov_xy = statistics.mean([
            (x - mean_x) * (y - mean_y)
            for x, y in zip(x_values, y_values)
        ])
        trace = var_x + var_y
        determinant = var_x * var_y - cov_xy * cov_xy
        discriminant = max(0.0, trace * trace - 4.0 * determinant)
        minor_eigenvalue = (trace - math.sqrt(discriminant)) * 0.5
        return math.sqrt(max(0.0, minor_eigenvalue))

    def _detect_a_conveyor_from_scan(self):
        scan = self.latest_scan
        if scan is None:
            return None

        points = self._scan_to_return_a_roi_points(scan)
        if len(points) < self.min_points:
            return None

        x_values = [point[0] for point in points]
        front_x = percentile(x_values, 0.15)
        front_points = [
            point for point in points
            if point[0] <= front_x + self.return_a_front_band_width
        ]
        if len(front_points) < self.min_points:
            front_points = points

        front_x_values = [point[0] for point in front_points]
        y_values = [point[1] for point in front_points]

        front_distance = percentile(front_x_values, 0.20)
        y_low = percentile(y_values, 0.10)
        y_high = percentile(y_values, 0.90)
        visible_width = y_high - y_low

        if visible_width >= self.min_visible_width:
            center_y = (y_low + y_high) * 0.5
        else:
            center_y = statistics.median(y_values)

        yaw_error = self._estimate_front_edge_yaw(front_points)

        return (
            front_distance,
            center_y,
            yaw_error,
            visible_width,
            len(front_points),
        )

    def _scan_to_return_a_roi_points(self, scan):
        points = []
        angle = scan.angle_min
        for distance in scan.ranges:
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                if (
                    self.return_a_roi_x_min <= x <= self.return_a_roi_x_max and
                    abs(y) <= self.return_a_roi_y_abs
                ):
                    points.append((x, y))
            angle += scan.angle_increment
        return points

    def _publish_return_a_leg_markers(
        self, header, points, active_points, candidates, selected
    ):
        if not self.return_a_debug_markers:
            return

        markers = MarkerArray()
        marker_id = 1
        markers.markers.append(self._make_return_a_roi_marker(header, marker_id))
        marker_id += 1

        if active_points:
            markers.markers.append(self._make_return_a_point_marker(
                header,
                marker_id,
                'safe_a_active_scan_points',
                active_points,
                ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.65),
                0.025,
            ))
            marker_id += 1

        for index, candidate in enumerate(candidates):
            is_selected = candidate in selected
            color = (
                ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.95)
                if is_selected else
                ColorRGBA(r=1.0, g=0.7, b=0.0, a=0.65)
            )
            scale = 0.12 if is_selected else 0.08
            markers.markers.append(self._make_return_a_sphere_marker(
                header,
                marker_id,
                'safe_a_leg_candidates',
                candidate['x'],
                candidate['y'],
                color,
                scale,
            ))
            marker_id += 1
            markers.markers.append(self._make_return_a_text_marker(
                header,
                marker_id,
                'safe_a_leg_labels',
                candidate['x'],
                candidate['y'],
                f'{index + 1}: {candidate["count"]}',
            ))
            marker_id += 1

        if len(selected) >= self.return_a_leg_expected_count:
            center_x = statistics.mean([candidate['x'] for candidate in selected])
            center_y = statistics.mean([candidate['y'] for candidate in selected])
            markers.markers.append(self._make_return_a_cube_marker(
                header,
                marker_id,
                'safe_a_leg_center',
                center_x,
                center_y,
                ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.95),
            ))
            marker_id += 1
            markers.markers.append(self._make_return_a_text_marker(
                header,
                marker_id,
                'safe_a_leg_center_label',
                center_x,
                center_y,
                'safe-A center',
            ))

        self.return_a_debug_marker_pub.publish(markers)

    def _make_return_a_roi_marker(self, header, marker_id):
        marker = self._make_return_a_base_marker(
            header, marker_id, 'safe_a_roi', Marker.LINE_STRIP)
        marker.color = ColorRGBA(r=0.1, g=0.8, b=1.0, a=0.75)
        marker.scale.x = 0.025
        corners = [
            (self.return_a_roi_x_min, -self.return_a_roi_y_abs),
            (self.return_a_roi_x_max, -self.return_a_roi_y_abs),
            (self.return_a_roi_x_max, self.return_a_roi_y_abs),
            (self.return_a_roi_x_min, self.return_a_roi_y_abs),
            (self.return_a_roi_x_min, -self.return_a_roi_y_abs),
        ]
        marker.points = [self._return_a_point(x, y, 0.02) for x, y in corners]
        return marker

    def _make_return_a_point_marker(self, header, marker_id, ns, points, color, scale):
        marker = self._make_return_a_base_marker(header, marker_id, ns, Marker.POINTS)
        marker.color = color
        marker.scale.x = scale
        marker.scale.y = scale
        marker.points = [self._return_a_point(x, y, 0.03) for x, y in points]
        return marker

    def _make_return_a_sphere_marker(self, header, marker_id, ns, x, y, color, scale):
        marker = self._make_return_a_base_marker(header, marker_id, ns, Marker.SPHERE)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.08
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color
        return marker

    def _make_return_a_cube_marker(self, header, marker_id, ns, x, y, color):
        marker = self._make_return_a_base_marker(header, marker_id, ns, Marker.CUBE)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.12
        marker.scale.x = 0.10
        marker.scale.y = 0.10
        marker.scale.z = 0.24
        marker.color = color
        return marker

    def _make_return_a_text_marker(self, header, marker_id, ns, x, y, text):
        marker = self._make_return_a_base_marker(
            header, marker_id, ns, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.25
        marker.scale.z = 0.12
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.95)
        marker.text = text
        return marker

    def _make_return_a_base_marker(self, header, marker_id, ns, marker_type):
        marker = Marker()
        marker.header = header
        marker.ns = ns
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        lifetime_sec = max(0.0, self.return_a_debug_marker_lifetime_sec)
        marker.lifetime = Duration(
            sec=int(lifetime_sec),
            nanosec=int((lifetime_sec % 1.0) * 1e9),
        )
        return marker

    @staticmethod
    def _return_a_point(x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    def _return_yaw_to_start_step(self):
        elapsed = time.monotonic() - self.return_start_time
        if elapsed > self.return_timeout_sec:
            self._fail(f'return_yaw_to_start_timeout after {elapsed:.1f}s')
            return

        _, _, target_yaw = self.route_start_pose
        _, _, current_yaw = self.current_pose
        yaw_error = normalize_angle(target_yaw - current_yaw)

        if abs(yaw_error) <= self.return_yaw_tolerance:
            self.cmd_pub.publish(Twist())
            self._prepare_reverse_return_segments()
            self._start_next_return_segment()
            return

        cmd = Twist()
        cmd.angular.z = clamp_with_min_abs(
            self.return_kz * yaw_error,
            self.return_max_wz,
            self.return_min_wz)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            'Return yaw align: '
            f'yaw_err={yaw_error:+.3f}rad, '
            f'cmd_wz={cmd.angular.z:+.2f}',
            throttle_duration_sec=0.5)

    def _start_next_return_segment(self):
        if self.return_segment_index >= len(self.return_segments):
            self._start_return_final_trim_or_yaw()
            return

        segment = self.return_segments[self.return_segment_index]
        self.segment_start_pose = self.current_pose
        self.segment_start_time = time.monotonic()
        self.state = 'return_segment'
        self.get_logger().info(
            f'Start return segment {self.return_segment_index + 1}/'
            f'{len(self.return_segments)}: {segment["name"]} '
            f'{segment["distance"]:.2f}m at {segment["speed"]:.2f}m/s')

    def _return_segment_step(self):
        segment = self.return_segments[self.return_segment_index]
        elapsed = time.monotonic() - self.segment_start_time
        forward_delta, left_delta = self._compute_delta_from_segment_start()
        progress = self._segment_progress(segment['name'], forward_delta, left_delta)

        if progress >= segment['distance']:
            self.get_logger().info(
                f'Return segment done: {segment["name"]}, '
                f'elapsed={elapsed:.2f}s, progress={progress:.3f}m, '
                f'forward_delta={forward_delta:+.3f}m, '
                f'left_delta={left_delta:+.3f}m')
            self._begin_return_settling()
            return

        if progress <= -self.wrong_direction_tolerance:
            self._fail(
                f'wrong_direction_detected in return {segment["name"]}: '
                f'progress={progress:+.3f}m')
            return

        if time.monotonic() - self.return_start_time >= self.return_timeout_sec:
            self._fail(f'return_to_a_timeout after {self.return_timeout_sec:.1f}s')
            return

        cmd = self._make_segment_command(segment['name'], segment['speed'])
        if cmd is None:
            self._fail(f'unknown return segment name: {segment["name"]}')
            return

        yaw_error = self._apply_segment_heading_hold(cmd)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'return {segment["name"]}: progress={progress:.3f}m / '
            f'{segment["distance"]:.3f}m, yaw_err={yaw_error:+.3f}rad, '
            f'cmd_wz={cmd.angular.z:+.2f}',
            throttle_duration_sec=0.5)

    def _begin_return_settling(self):
        self.cmd_pub.publish(Twist())
        self.stop_cycles_remaining = int(max(5, self.rate_hz * 0.5))
        self.state = 'return_settling'

    def _return_settling_step(self):
        self.cmd_pub.publish(Twist())
        self.stop_cycles_remaining -= 1
        if self.stop_cycles_remaining <= 0:
            self.return_segment_index += 1
            self._start_next_return_segment()


    def _prepare_return_final_trim_segments(self):
        if self.return_trim_segments_prepared:
            return

        forward_error, left_error = self._compute_error_to_start_pose()
        tolerance = self.return_final_trim_tolerance

        if abs(left_error) > tolerance:
            lateral_distance = min(
                abs(left_error), self.return_final_trim_max_distance)
            self.return_trim_segments.append({
                'name': 'left' if left_error > 0.0 else 'right',
                'distance': lateral_distance,
                'speed': self.lateral_speed,
            })

        if abs(forward_error) > tolerance:
            forward_distance = min(
                abs(forward_error), self.return_final_trim_max_distance)
            self.return_trim_segments.append({
                'name': 'forward' if forward_error > 0.0 else 'backward',
                'distance': forward_distance,
                'speed': self.linear_speed,
            })

        self.return_trim_segments_prepared = True
        self.get_logger().info(
            'Computed final A trim segments: '
            f'forward_error={forward_error:+.3f}m, '
            f'left_error={left_error:+.3f}m, '
            f'segments={[(s["name"], round(s["distance"], 3)) for s in self.return_trim_segments]}')

    def _compute_error_to_start_pose(self):
        target_x, target_y, target_yaw = self.route_start_pose
        current_x, current_y, _ = self.current_pose
        dx = target_x - current_x
        dy = target_y - current_y
        forward_error = math.cos(target_yaw) * dx + math.sin(target_yaw) * dy
        left_error = -math.sin(target_yaw) * dx + math.cos(target_yaw) * dy
        return forward_error, left_error

    def _start_next_return_trim_segment(self):
        if self.return_trim_segment_index >= len(self.return_trim_segments):
            if self.return_match_start_yaw:
                self.state = 'return_final_yaw_to_start'
                self.get_logger().info('Start final yaw alignment at A.')
            else:
                self._finish_return_to_a('returned_to_a_reverse_segments')
            return

        segment = self.return_trim_segments[self.return_trim_segment_index]
        self.segment_start_pose = self.current_pose
        self.segment_start_time = time.monotonic()
        self.state = 'return_trim_segment'
        self.get_logger().info(
            f'Start final A trim segment {self.return_trim_segment_index + 1}/'
            f'{len(self.return_trim_segments)}: {segment["name"]} '
            f'{segment["distance"]:.2f}m at {segment["speed"]:.2f}m/s')

    def _return_trim_segment_step(self):
        segment = self.return_trim_segments[self.return_trim_segment_index]
        elapsed = time.monotonic() - self.segment_start_time
        forward_delta, left_delta = self._compute_delta_from_segment_start()
        progress = self._segment_progress(segment['name'], forward_delta, left_delta)

        if progress >= segment['distance']:
            self.get_logger().info(
                f'Final A trim segment done: {segment["name"]}, '
                f'elapsed={elapsed:.2f}s, progress={progress:.3f}m')
            self._begin_return_trim_settling()
            return

        if progress <= -self.wrong_direction_tolerance:
            self._fail(
                f'wrong_direction_detected in final A trim {segment["name"]}: '
                f'progress={progress:+.3f}m')
            return

        if time.monotonic() - self.return_start_time >= self.return_timeout_sec:
            self._fail(f'return_to_a_timeout after {self.return_timeout_sec:.1f}s')
            return

        cmd = self._make_segment_command(segment['name'], segment['speed'])
        if cmd is None:
            self._fail(f'unknown final trim segment name: {segment["name"]}')
            return

        yaw_error = self._apply_segment_heading_hold(cmd)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'final trim {segment["name"]}: progress={progress:.3f}m / '
            f'{segment["distance"]:.3f}m, yaw_err={yaw_error:+.3f}rad, '
            f'cmd_wz={cmd.angular.z:+.2f}',
            throttle_duration_sec=0.5)

    def _begin_return_trim_settling(self):
        self.cmd_pub.publish(Twist())
        self.stop_cycles_remaining = int(max(5, self.rate_hz * 0.5))
        self.state = 'return_trim_settling'

    def _return_trim_settling_step(self):
        self.cmd_pub.publish(Twist())
        self.stop_cycles_remaining -= 1
        if self.stop_cycles_remaining <= 0:
            self.return_trim_segment_index += 1
            self._start_next_return_trim_segment()

    def _return_final_yaw_to_start_step(self):
        elapsed = time.monotonic() - self.return_start_time
        if elapsed > self.return_timeout_sec:
            self._fail(f'return_final_yaw_to_start_timeout after {elapsed:.1f}s')
            return

        _, _, target_yaw = self.route_start_pose
        _, _, current_yaw = self.current_pose
        yaw_error = normalize_angle(target_yaw - current_yaw)

        if abs(yaw_error) <= self.return_yaw_tolerance:
            self.cmd_pub.publish(Twist())
            self._finish_return_to_a('returned_to_a_reverse_segments')
            return

        cmd = Twist()
        cmd.angular.z = clamp_with_min_abs(
            self.return_kz * yaw_error,
            self.return_max_wz,
            self.return_min_wz)
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            'Return final yaw align: '
            f'yaw_err={yaw_error:+.3f}rad, '
            f'cmd_wz={cmd.angular.z:+.2f}',
            throttle_duration_sec=0.5)

    def _return_to_a_step(self):
        self._publish_waypoint_poses()
        elapsed = time.monotonic() - self.return_start_time
        if elapsed > self.return_timeout_sec:
            self._fail(f'return_to_a_timeout after {elapsed:.1f}s')
            return

        target_x, target_y, target_yaw = self.route_start_pose
        current_x, current_y, current_yaw = self.current_pose
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.hypot(dx, dy)
        yaw_error = normalize_angle(target_yaw - current_yaw)

        cmd = Twist()
        if distance > self.return_position_tolerance:
            forward_error = math.cos(current_yaw) * dx + math.sin(current_yaw) * dy
            left_error = -math.sin(current_yaw) * dx + math.cos(current_yaw) * dy
            if abs(forward_error) > self.return_position_tolerance * 0.5:
                cmd.linear.x = clamp_with_min_abs(
                    self.return_kx * forward_error,
                    self.return_max_vx,
                    self.return_min_vx)
            if abs(left_error) > self.return_position_tolerance * 0.5:
                cmd.linear.y = clamp_with_min_abs(
                    self.return_ky * left_error,
                    self.return_max_vy,
                    self.return_min_vy)
            phase = 'translate'
        elif abs(yaw_error) > self.return_yaw_tolerance:
            cmd.angular.z = clamp_with_min_abs(
                self.return_kz * yaw_error,
                self.return_max_wz,
                self.return_min_wz)
            phase = 'yaw'
        else:
            self._finish_return_to_a('returned_to_a')
            return

        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            'Return to A: '
            f'phase={phase}, distance={distance:.3f}m, '
            f'yaw_err={yaw_error:+.3f}rad, '
            f'cmd=({cmd.linear.x:+.2f}, {cmd.linear.y:+.2f}, '
            f'{cmd.angular.z:+.2f})',
            throttle_duration_sec=0.5)

    def _finish_return_to_a(self, reason):
        if self.enable_return_a_lidar_alignment:
            self._start_lidar_alignment(
                target='A',
                desired_front_distance=self.return_a_desired_front_distance,
                timeout_sec=self.return_a_align_timeout_sec)
            return
        self._finish(reason)


    def _scan_to_roi_points(self, scan):
        points = []
        angle = scan.angle_min
        for distance in scan.ranges:
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                if (
                    self.roi_x_min <= x <= self.roi_x_max and
                    abs(y) <= self.roi_y_abs
                ):
                    points.append((x, y))
            angle += scan.angle_increment
        return points

    def _estimate_front_edge_yaw(self, points):
        if len(points) < 2:
            return 0.0

        mean_y = sum(point[1] for point in points) / len(points)
        mean_x = sum(point[0] for point in points) / len(points)
        y_variance = sum((point[1] - mean_y) ** 2 for point in points)
        if y_variance < 1e-9:
            return 0.0

        # The table front edge should be roughly parallel to the robot Y axis,
        # so an aligned edge has x ~= constant, not angle ~= 0.  Estimate only
        # the small tilt from that ideal edge by fitting x = slope * y + b.
        xy_covariance = sum(
            (point[1] - mean_y) * (point[0] - mean_x) for point in points)
        slope = xy_covariance / y_variance
        return math.atan(slope)

    def _compute_delta_from_segment_start(self):
        start_x, start_y, start_yaw = self.segment_start_pose
        current_x, current_y, _ = self.current_pose
        dx = current_x - start_x
        dy = current_y - start_y

        forward_delta = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
        left_delta = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
        return forward_delta, left_delta

    def _finish(self, reason):
        self.cmd_pub.publish(Twist())
        self.done = True
        self.done_time = time.monotonic()
        self.state = 'done'
        if reason.startswith('returned_to_a'):
            self._publish_arrived_a(True)
            self._publish_nav_state('ARRIVED_A')
        else:
            self._publish_nav_state('DONE')
        self.get_logger().info(
            f'Mission B primitive route done: {reason}. '
            f'Keeping waypoint markers alive for '
            f'{self.keep_alive_after_done_sec:.1f}s.')

    def _fail(self, reason):
        self.cmd_pub.publish(Twist())
        self.done = True
        self.done_time = time.monotonic()
        self.state = 'failed'
        self._publish_failure_reason(reason)
        self._publish_nav_state('FAILED')
        self.get_logger().error(
            f'Mission B primitive route failed: {reason}. '
            f'Keeping waypoint markers alive for '
            f'{self.keep_alive_after_done_sec:.1f}s.')

    def stop(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = Sg2MissionBRoute()
    try:
        node.wait_for_cmd_subscriber()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.done:
                continue
            if node.done_time is None:
                break
            elapsed_since_done = time.monotonic() - node.done_time
            if elapsed_since_done >= node.keep_alive_after_done_sec:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
