import json
import math
import os

import cv2
from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


DXL_CURRENT_JOINTS = [
    ('right', 'J1', 'dxl1'),
    ('right', 'J2', 'dxl2'),
    ('right', 'J3', 'dxl3'),
    ('left', 'J1', 'dxl31'),
    ('left', 'J2', 'dxl32'),
    ('left', 'J3', 'dxl33'),
]
DXL_CURRENT_NAMES = {name for _, _, name in DXL_CURRENT_JOINTS}
DXL_CURRENT_INTERFACE = 'Present Current'


class AlignmentStatus(Node):

    def __init__(self):
        super().__init__('teleop_alignment_status')

        self.declare_parameter('right_goal_topic', '/r_goal_pose')
        self.declare_parameter('left_goal_topic', '/l_goal_pose')
        self.declare_parameter('right_current_topic', '/r_gripper_pose')
        self.declare_parameter('left_current_topic', '/l_gripper_pose')
        self.declare_parameter('status_topic', '/teleop/alignment_status')
        self.declare_parameter('ok_topic', '/teleop/alignment_ok')
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('pos_threshold_m', 0.30)
        self.declare_parameter('ori_threshold_deg', 120.0)
        self.declare_parameter('stale_timeout_s', 1.0)
        self.declare_parameter('record_practice_events', True)
        self.declare_parameter('practice_event_log_path', '~/teleop_practice_events.jsonl')
        self.declare_parameter('practice_event_input_topic', '/teleop/practice_event/mark')
        self.declare_parameter('practice_event_output_topic', '/teleop/practice_event')
        self.declare_parameter('tact_trigger_topic', '/leader/joystick_controller/tact_trigger')
        self.declare_parameter(
            'joystick_mode_topic', '/leader/joystick_controller_right/joystick_mode')
        self.declare_parameter(
            'head_target_topic', '/leader/joystick_controller_left/joint_trajectory')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter(
            'right_center_distance_topic', '/teleop/wrist_right/center_distance_m')
        self.declare_parameter(
            'left_center_distance_topic', '/teleop/wrist_left/center_distance_m')
        self.declare_parameter(
            'right_depth_metrics_topic', '/teleop/wrist_right/depth_metrics')
        self.declare_parameter(
            'left_depth_metrics_topic', '/teleop/wrist_left/depth_metrics')
        self.declare_parameter('table_reference_enabled', False)
        self.declare_parameter('table_x_m', 0.0)
        self.declare_parameter('table_y_m', 0.0)
        self.declare_parameter('table_yaw_deg', 0.0)
        self.declare_parameter('status_panel_topic', '/teleop/operator_status/compressed')
        self.declare_parameter('status_panel_width', 1280)
        self.declare_parameter('status_panel_height', 720)
        self.declare_parameter('status_panel_jpeg_quality', 95)
        self.declare_parameter('dxl_current_topic', '/dynamic_joint_states')
        self.declare_parameter('dxl_current_calibration_s', 3.0)
        self.declare_parameter('dxl_current_baseline_floor', 50.0)
        self.declare_parameter('dxl_current_caution_ratio', 1.6)
        self.declare_parameter('dxl_current_warn_ratio', 2.2)
        self.declare_parameter('dxl_current_stale_timeout_s', 1.0)

        self.pos_threshold_m = float(self.get_parameter('pos_threshold_m').value)
        self.ori_threshold_deg = float(self.get_parameter('ori_threshold_deg').value)
        self.stale_timeout_s = float(self.get_parameter('stale_timeout_s').value)
        publish_hz = max(float(self.get_parameter('publish_hz').value), 0.1)
        self.record_practice_events = self._as_bool(
            self.get_parameter('record_practice_events').value)
        self.practice_event_log_path = os.path.expanduser(
            str(self.get_parameter('practice_event_log_path').value))
        self.table_reference_enabled = self._as_bool(
            self.get_parameter('table_reference_enabled').value)
        self.table_x_m = float(self.get_parameter('table_x_m').value)
        self.table_y_m = float(self.get_parameter('table_y_m').value)
        self.table_yaw_rad = math.radians(float(self.get_parameter('table_yaw_deg').value))
        self.status_panel_topic = str(self.get_parameter('status_panel_topic').value).strip()
        self.status_panel_width = max(int(self.get_parameter('status_panel_width').value), 320)
        self.status_panel_height = max(int(self.get_parameter('status_panel_height').value), 180)
        self.status_panel_jpeg_quality = int(min(
            max(int(self.get_parameter('status_panel_jpeg_quality').value), 1), 100))
        self.dxl_current_topic = str(self.get_parameter('dxl_current_topic').value).strip()
        self.dxl_current_calibration_s = max(
            float(self.get_parameter('dxl_current_calibration_s').value), 0.0)
        self.dxl_current_baseline_floor = max(
            float(self.get_parameter('dxl_current_baseline_floor').value), 1.0)
        self.dxl_current_caution_ratio = max(
            float(self.get_parameter('dxl_current_caution_ratio').value), 1.0)
        self.dxl_current_warn_ratio = max(
            float(self.get_parameter('dxl_current_warn_ratio').value),
            self.dxl_current_caution_ratio)
        self.dxl_current_stale_timeout_s = max(
            float(self.get_parameter('dxl_current_stale_timeout_s').value), 0.2)

        self.latest = {}
        self.latest_alignment_status = None
        self.latest_joint_state = None
        self.latest_head_target = None
        self.latest_odom = None
        self.latest_cmd_vel = None
        self.latest_mode = 'arm_control'
        self.latest_center_distance = {}
        self.latest_depth_metrics = {}
        self.latest_dxl_current = {}
        self.latest_dxl_current_time = None
        self.dxl_current_calibration_start = None
        self.dxl_current_baseline = {}
        self.dxl_current_samples = {name: [] for name in DXL_CURRENT_NAMES}
        self.last_event_warn_time = None
        self._subscribe_pose('right_goal', self.get_parameter('right_goal_topic').value)
        self._subscribe_pose('left_goal', self.get_parameter('left_goal_topic').value)
        self._subscribe_pose('right_current', self.get_parameter('right_current_topic').value)
        self._subscribe_pose('left_current', self.get_parameter('left_current_topic').value)

        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 1)
        self.ok_pub = self.create_publisher(
            Bool, self.get_parameter('ok_topic').value, 1)
        self.practice_event_pub = self.create_publisher(
            String, self.get_parameter('practice_event_output_topic').value, 10)
        self.status_panel_pub = None
        if self.status_panel_topic:
            self.status_panel_pub = self.create_publisher(
                CompressedImage, self.status_panel_topic, 1)
        self._init_practice_event_inputs()
        self.timer = self.create_timer(1.0 / publish_hz, self._publish_status)

    def _subscribe_pose(self, key, topic):
        self.create_subscription(
            PoseStamped, topic, lambda msg, key=key: self._pose_callback(key, msg), 1)

    def _init_practice_event_inputs(self):
        self._subscribe_if_topic(
            String, self.get_parameter('joystick_mode_topic').value,
            self._joystick_mode_callback)
        self._subscribe_if_topic(
            JointTrajectory, self.get_parameter('head_target_topic').value,
            self._head_target_callback)
        self._subscribe_if_topic(
            JointState, self.get_parameter('joint_state_topic').value,
            self._joint_state_callback)
        self._subscribe_if_topic(
            Odometry, self.get_parameter('odom_topic').value,
            self._odom_callback)
        self._subscribe_if_topic(
            Twist, self.get_parameter('cmd_vel_topic').value,
            self._cmd_vel_callback)
        self._subscribe_if_topic(
            Float32, self.get_parameter('right_center_distance_topic').value,
            lambda msg: self._center_distance_callback('right', msg),
            qos_profile_sensor_data)
        self._subscribe_if_topic(
            Float32, self.get_parameter('left_center_distance_topic').value,
            lambda msg: self._center_distance_callback('left', msg),
            qos_profile_sensor_data)
        self._subscribe_if_topic(
            String, self.get_parameter('right_depth_metrics_topic').value,
            lambda msg: self._depth_metrics_callback('right', msg))
        self._subscribe_if_topic(
            String, self.get_parameter('left_depth_metrics_topic').value,
            lambda msg: self._depth_metrics_callback('left', msg))
        self._subscribe_if_topic(
            DynamicJointState, self.dxl_current_topic,
            self._dxl_current_callback, qos_profile_sensor_data)

        if not self.record_practice_events:
            return
        self._subscribe_if_topic(
            String, self.get_parameter('practice_event_input_topic').value,
            self._manual_practice_event_callback)
        self._subscribe_if_topic(
            String, self.get_parameter('tact_trigger_topic').value,
            self._tact_trigger_callback)

        self.get_logger().info(
            f'teleop practice event recording enabled: {self.practice_event_log_path}')

    def _subscribe_if_topic(self, msg_type, topic, callback, qos=10):
        topic = str(topic).strip()
        if topic:
            self.create_subscription(msg_type, topic, callback, qos)

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _pose_callback(self, key, msg):
        self.latest[key] = {
            'msg': msg,
            'received_time': self.get_clock().now(),
        }

    def _manual_practice_event_callback(self, msg):
        event = msg.data.strip() or 'mark'
        self._record_practice_event(event, 'manual')

    def _tact_trigger_callback(self, msg):
        trigger = msg.data.strip()
        event_by_trigger = {
            'left': 'retry',
            'right': 'next',
            'left_long_time': 'good_pose',
            'right_long_time': 'success',
        }
        self._record_practice_event(
            event_by_trigger.get(trigger, trigger or 'tact'), f'tact:{trigger}')

    def _joystick_mode_callback(self, msg):
        if msg.data.strip():
            self.latest_mode = msg.data.strip()

    def _joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def _head_target_callback(self, msg):
        target = {}
        if msg.points:
            positions = msg.points[-1].positions
            for index, name in enumerate(msg.joint_names):
                if index < len(positions):
                    target[name] = float(positions[index])
        self.latest_head_target = {
            'stamp_sec': self._stamp_sec(msg.header.stamp),
            'target': target,
        }

    def _odom_callback(self, msg):
        self.latest_odom = msg

    def _cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg

    def _center_distance_callback(self, side, msg):
        self.latest_center_distance[side] = float(msg.data)

    def _depth_metrics_callback(self, side, msg):
        try:
            self.latest_depth_metrics[side] = json.loads(msg.data)
        except json.JSONDecodeError:
            self._warn_event_throttled(f'failed to parse {side} depth metrics')

    def _dxl_current_callback(self, msg):
        now = self.get_clock().now()
        if self.dxl_current_calibration_start is None:
            self.dxl_current_calibration_start = now

        latest = {}
        for index, joint_name in enumerate(msg.joint_names):
            if joint_name not in DXL_CURRENT_NAMES or index >= len(msg.interface_values):
                continue
            interface_values = msg.interface_values[index]
            entry = {
                'current_raw': None,
                'error_code': None,
                'hardware_error_status': None,
            }
            for interface_name, value in zip(
                interface_values.interface_names,
                interface_values.values
            ):
                cleaned_value = self._safe_float(value)
                if cleaned_value is None:
                    continue
                if interface_name == DXL_CURRENT_INTERFACE:
                    entry['current_raw'] = cleaned_value
                elif interface_name == 'Error Code':
                    entry['error_code'] = cleaned_value
                elif interface_name == 'Hardware Error Status':
                    entry['hardware_error_status'] = cleaned_value
            latest[joint_name] = entry

        if not latest:
            return

        self.latest_dxl_current = latest
        self.latest_dxl_current_time = now

        if self._dxl_current_calibrating(now):
            for joint_name, entry in latest.items():
                current = entry.get('current_raw')
                if current is not None:
                    self.dxl_current_samples[joint_name].append(abs(float(current)))
        elif len(self.dxl_current_baseline) < len(DXL_CURRENT_NAMES):
            self._finalize_dxl_current_baseline()

    def _publish_status(self):
        right = self._arm_status('right')
        left = self._arm_status('left')
        all_ok = bool(right['ok'] and left['ok'])

        payload = {
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'mode': self.latest_mode,
            'thresholds': {
                'pos_m': self.pos_threshold_m,
                'ori_deg': self.ori_threshold_deg,
            },
            'right': right,
            'left': left,
            'all_ok': all_ok,
            'center_distance_m': self._center_distance_snapshot(),
            'depth_metrics': self._depth_metrics_snapshot(),
            'head': self._head_snapshot(),
            'table_relative': self._table_relative_snapshot(),
            'dxl_current': self._dxl_current_snapshot(),
        }
        self.latest_alignment_status = payload

        status_msg = String()
        status_msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(status_msg)

        ok_msg = Bool()
        ok_msg.data = all_ok
        self.ok_pub.publish(ok_msg)
        self._publish_status_panel(payload)

    def _record_practice_event(self, event, source):
        if not self.record_practice_events:
            return
        payload = {
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'event': event,
            'source': source,
            'mode': self.latest_mode,
            'joint_state': self._joint_state_snapshot(),
            'odom': self._odom_snapshot(),
            'cmd_vel': self._cmd_vel_snapshot(),
            'center_distance_m': self._center_distance_snapshot(),
            'depth_metrics': self._depth_metrics_snapshot(),
            'head': self._head_snapshot(),
            'table_relative': self._table_relative_snapshot(),
            'dxl_current': self._dxl_current_snapshot(),
            'alignment': self.latest_alignment_status,
        }
        data = json.dumps(payload, sort_keys=True)

        msg = String()
        msg.data = data
        self.practice_event_pub.publish(msg)

        if not self.practice_event_log_path:
            return
        try:
            parent = os.path.dirname(self.practice_event_log_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(self.practice_event_log_path, 'a', encoding='utf-8') as stream:
                stream.write(data + '\n')
        except OSError as exc:
            self._warn_event_throttled(f'failed to write practice event log: {exc}')

    def _warn_event_throttled(self, message):
        now = self.get_clock().now()
        if (self.last_event_warn_time is None or
                (now - self.last_event_warn_time).nanoseconds > 5_000_000_000):
            self.get_logger().warn(message)
            self.last_event_warn_time = now

    def _publish_status_panel(self, payload):
        if (
            self.status_panel_pub is None or
            self.count_subscribers(self.status_panel_topic) <= 0
        ):
            return

        width = self.status_panel_width
        height = self.status_panel_height
        scale = min(width / 640.0, height / 360.0)
        def p(x, y):
            return int(round(x * scale)), int(round(y * scale))
        def line(value):
            return max(int(round(value * scale)), 1)
        image = np.full((height, width, 3), (34, 36, 40), dtype=np.uint8)
        mode = str(payload.get('mode') or 'unknown').strip()
        mode_key = mode.lower()
        if mode_key == 'swerve':
            mode_label = 'BASE MOVE'
            header_color = (32, 113, 239)
        else:
            mode_label = 'ZED/HEAD + LIFT'
            header_color = (67, 161, 92)

        cv2.rectangle(image, (0, 0), (width, int(round(58 * scale))), header_color, -1)
        self._put_panel_text(
            image, f'MODE: {mode.upper()}', p(16, 38),
            0.86 * scale, (255, 255, 255), line(2))
        self._put_panel_text(
            image, mode_label, p(370, 38),
            0.70 * scale, (255, 255, 255), line(2))

        head = payload.get('head') or {}
        head_state = 'HOLD' if mode_key == 'swerve' else 'ACTIVE'
        head_text = (
            f'HEAD {head_state}: CUR '
            f'{self._format_joint(head.get("head_joint1"))},'
            f'{self._format_joint(head.get("head_joint2"))}  TGT '
            f'{self._format_joint(head.get("target_head_joint1"))},'
            f'{self._format_joint(head.get("target_head_joint2"))}  ERR '
            f'{self._format_joint(head.get("error_head_joint1"))},'
            f'{self._format_joint(head.get("error_head_joint2"))}'
        )
        self._put_panel_text(
            image, head_text, p(16, 84), 0.50 * scale, (236, 241, 245), line(1))

        depth_metrics = payload.get('depth_metrics') or {}
        distances = payload.get('center_distance_m') or {}
        left_metric = depth_metrics.get('left') or {}
        right_metric = depth_metrics.get('right') or {}
        left_distance = self._format_distance(left_metric.get('target_m', left_metric.get('center_m', distances.get('left'))))
        right_distance = self._format_distance(right_metric.get('target_m', right_metric.get('center_m', distances.get('right'))))
        left_hint = str(left_metric.get('hint') or '--')
        right_hint = str(right_metric.get('hint') or '--')
        self._put_panel_text(
            image, f'L TARGET DEPTH: {left_distance} {left_hint}',
            p(16, 118), 0.50 * scale, (236, 241, 245), line(1))
        self._put_panel_text(
            image, f'R TARGET DEPTH: {right_distance} {right_hint}',
            p(292, 118), 0.50 * scale, (236, 241, 245), line(1))

        left_offset = self._format_offset(left_metric)
        right_offset = self._format_offset(right_metric)
        self._put_panel_text(
            image, f'L OFFSET: {left_offset}',
            p(16, 152), 0.50 * scale, (236, 241, 245), line(1))
        self._put_panel_text(
            image, f'R OFFSET: {right_offset}',
            p(292, 152), 0.50 * scale, (236, 241, 245), line(1))

        left_view = self._format_view(left_metric)
        right_view = self._format_view(right_metric)
        self._put_panel_text(
            image, f'VIEW: L {left_view}  R {right_view}',
            p(16, 186), 0.50 * scale, (203, 211, 218), line(1))

        table = payload.get('table_relative')
        if table:
            table_text = (
                f'TABLE: {self._format_distance(table.get("distance_m"))} '
                f'HEAD ERR: {self._format_angle(table.get("heading_to_table_error_deg"))}'
            )
        else:
            table_text = 'TABLE: --'
        self._put_panel_text(
            image, table_text, p(16, 220), 0.48 * scale, (203, 211, 218), line(1))

        self._draw_dxl_current_panel(
            image, payload.get('dxl_current'), p, line, scale)

        ok, encoded = cv2.imencode(
            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), self.status_panel_jpeg_quality])
        if not ok:
            self._warn_event_throttled('failed to JPEG-encode operator status panel')
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'teleop_operator_status'
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        self.status_panel_pub.publish(msg)

    def _put_panel_text(self, image, text, origin, scale, color, thickness):
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            color, thickness, cv2.LINE_AA)

    def _draw_dxl_current_panel(self, image, snapshot, p, line, scale):
        base_color = (236, 241, 245)
        muted_color = (203, 211, 218)
        state = (snapshot or {}).get('state', 'no_data')
        title = 'DXL LOAD'
        if state == 'calibrating':
            elapsed = float((snapshot or {}).get('calibration_elapsed_s') or 0.0)
            total = max(self.dxl_current_calibration_s, 0.1)
            text = f'{title}: CALIBRATING {elapsed:.1f}/{total:.1f}s'
            self._put_panel_text(image, text, p(16, 256), 0.48 * scale, muted_color, line(1))
            return
        if state == 'no_data':
            self._put_panel_text(
                image, f'{title}: NO DATA', p(16, 256),
                0.48 * scale, muted_color, line(1))
            return
        if state == 'stale':
            stale_age = (snapshot or {}).get('stale_age_s')
            suffix = f' {float(stale_age):.1f}s' if stale_age is not None else ''
            self._put_panel_text(
                image, f'{title}: STALE{suffix}', p(16, 256),
                0.48 * scale, (0, 165, 255), line(1))
            return

        self._put_panel_text(
            image, f'{title} raw', p(16, 256),
            0.48 * scale, base_color, line(1))
        self._draw_dxl_current_row(image, snapshot, 'right', p(16, 290), p, line, scale)
        self._draw_dxl_current_row(image, snapshot, 'left', p(16, 324), p, line, scale)

    def _draw_dxl_current_row(self, image, snapshot, side, origin, p, line, scale):
        side_label = 'R' if side == 'right' else 'L'
        self._put_panel_text(
            image, f'{side_label}:', origin, 0.45 * scale,
            (236, 241, 245), line(1))
        x_base = 54
        joints = (snapshot or {}).get('joints') or {}
        for index, (_, joint_label, joint_name) in enumerate(
            [item for item in DXL_CURRENT_JOINTS if item[0] == side]
        ):
            entry = joints.get(joint_name) or {}
            text = self._format_dxl_current_entry(joint_label, entry)
            color = self._dxl_level_color(entry.get('level'))
            x = x_base + index * 190
            self._put_panel_text(
                image, text, p(x, origin[1] / scale),
                0.43 * scale, color, line(1))
            self._draw_dxl_current_bar(
                image, entry, p(x, origin[1] / scale + 10), scale)

    def _draw_dxl_current_bar(self, image, entry, origin, scale):
        width = int(round(128 * scale))
        height = max(int(round(7 * scale)), 2)
        x, y = origin
        cv2.rectangle(image, (x, y), (x + width, y + height), (72, 76, 82), -1)
        ratio = entry.get('load_ratio')
        if ratio is None:
            return
        fill_ratio = min(max(float(ratio) / max(self.dxl_current_warn_ratio, 0.1), 0.0), 1.0)
        fill_width = int(round(width * fill_ratio))
        if fill_width <= 0:
            return
        cv2.rectangle(
            image, (x, y), (x + fill_width, y + height),
            self._dxl_level_color(entry.get('level')), -1)

    def _format_distance(self, value):
        if value is None:
            return '-- m'
        return f'{float(value):.2f} m'

    def _format_angle(self, value):
        if value is None:
            return '-- deg'
        return f'{float(value):+.1f} deg'

    def _format_offset(self, metrics):
        offset_x = metrics.get('offset_x_px')
        offset_y = metrics.get('offset_y_px')
        axis = metrics.get('axis_angle_deg')
        if offset_x is None or offset_y is None:
            return '--'
        if axis is None:
            return f'{int(offset_x):+d},{int(offset_y):+d}px'
        return f'{int(offset_x):+d},{int(offset_y):+d}px {float(axis):+.0f}deg'

    def _format_view(self, metrics):
        view = metrics.get('view') or {}
        rotate = view.get('rotate_deg')
        if rotate is None:
            return '--'
        flags = []
        if view.get('flip_horizontal'):
            flags.append('FH')
        if view.get('flip_vertical'):
            flags.append('FV')
        suffix = (' ' + '/'.join(flags)) if flags else ''
        return f'{float(rotate):.0f}deg{suffix}'

    def _format_joint(self, value):
        if value is None:
            return '--'
        return f'{float(value):+.2f}'

    def _format_dxl_current_entry(self, joint_label, entry):
        current = self._format_dxl_current_value(entry.get('current_raw'))
        if entry.get('fault'):
            return f'{joint_label} FAULT/{current}'
        ratio = entry.get('load_ratio')
        if ratio is None:
            return f'{joint_label} --/{current}'
        return f'{joint_label} {float(ratio):.1f}x/{current}'

    def _format_dxl_current_value(self, value):
        if value is None:
            return '--'
        value = float(value)
        if abs(value - round(value)) < 0.05:
            return str(int(round(value)))
        return f'{value:.1f}'

    def _dxl_level_color(self, level):
        if level == 'fault':
            return (36, 36, 255)
        if level == 'red':
            return (48, 64, 255)
        if level == 'orange':
            return (0, 165, 255)
        if level == 'green':
            return (84, 214, 127)
        return (203, 211, 218)

    def _format_arm_status(self, status):
        if not status.get('available'):
            return '--'
        if not status.get('fresh'):
            return 'STALE'
        return 'OK' if status.get('ok') else 'CHECK'

    def _joint_state_snapshot(self):
        msg = self.latest_joint_state
        if msg is None:
            return None
        return {
            'stamp_sec': self._stamp_sec(msg.header.stamp),
            'name': list(msg.name),
            'position': self._float_list(msg.position),
            'velocity': self._float_list(msg.velocity),
            'effort': self._float_list(msg.effort),
        }

    def _head_snapshot(self):
        msg = self.latest_joint_state
        if msg is None:
            return None
        current = {
            'head_joint1': self._joint_position(msg, 'head_joint1'),
            'head_joint2': self._joint_position(msg, 'head_joint2'),
        }
        target_msg = self.latest_head_target or {}
        target_map = target_msg.get('target') or {}
        target = {
            'head_joint1': self._clean_float(target_map.get('head_joint1')) if 'head_joint1' in target_map else None,
            'head_joint2': self._clean_float(target_map.get('head_joint2')) if 'head_joint2' in target_map else None,
        }
        error = {}
        for name in ('head_joint1', 'head_joint2'):
            if current[name] is None or target[name] is None:
                error[name] = None
            else:
                error[name] = self._clean_float(current[name] - target[name])
        return {
            'head_joint1': current['head_joint1'],
            'head_joint2': current['head_joint2'],
            'target_head_joint1': target['head_joint1'],
            'target_head_joint2': target['head_joint2'],
            'error_head_joint1': error['head_joint1'],
            'error_head_joint2': error['head_joint2'],
            'target_stamp_sec': target_msg.get('stamp_sec'),
        }

    def _joint_position(self, msg, name):
        try:
            index = msg.name.index(name)
        except ValueError:
            return None
        if index >= len(msg.position):
            return None
        return self._clean_float(msg.position[index])

    def _odom_snapshot(self):
        msg = self.latest_odom
        if msg is None:
            return None
        pose = msg.pose.pose
        twist = msg.twist.twist
        yaw = self._yaw_from_quaternion(pose.orientation)
        return {
            'stamp_sec': self._stamp_sec(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'child_frame_id': msg.child_frame_id,
            'x_m': self._clean_float(pose.position.x),
            'y_m': self._clean_float(pose.position.y),
            'z_m': self._clean_float(pose.position.z),
            'yaw_deg': self._clean_float(math.degrees(yaw)),
            'linear_x_mps': self._clean_float(twist.linear.x),
            'linear_y_mps': self._clean_float(twist.linear.y),
            'angular_z_radps': self._clean_float(twist.angular.z),
        }

    def _cmd_vel_snapshot(self):
        msg = self.latest_cmd_vel
        if msg is None:
            return None
        return {
            'linear_x_mps': self._clean_float(msg.linear.x),
            'linear_y_mps': self._clean_float(msg.linear.y),
            'angular_z_radps': self._clean_float(msg.angular.z),
        }

    def _center_distance_snapshot(self):
        if not self.latest_center_distance:
            return None
        return {
            side: self._clean_float(distance)
            for side, distance in self.latest_center_distance.items()
        }

    def _depth_metrics_snapshot(self):
        if not self.latest_depth_metrics:
            return None
        return self.latest_depth_metrics.copy()

    def _dxl_current_snapshot(self):
        if self.latest_dxl_current_time is None:
            return {
                'state': 'no_data',
                'topic': self.dxl_current_topic,
            }

        now = self.get_clock().now()
        age_s = (now - self.latest_dxl_current_time).nanoseconds / 1e9
        if age_s > self.dxl_current_stale_timeout_s:
            return {
                'state': 'stale',
                'topic': self.dxl_current_topic,
                'stale_age_s': self._clean_float(age_s),
            }

        if self._dxl_current_calibrating(now):
            elapsed_s = 0.0
            if self.dxl_current_calibration_start is not None:
                elapsed_s = (now - self.dxl_current_calibration_start).nanoseconds / 1e9
            return {
                'state': 'calibrating',
                'topic': self.dxl_current_topic,
                'calibration_elapsed_s': self._clean_float(elapsed_s),
                'calibration_target_s': self._clean_float(self.dxl_current_calibration_s),
            }

        if len(self.dxl_current_baseline) < len(DXL_CURRENT_NAMES):
            self._finalize_dxl_current_baseline()

        joints = {}
        worst_level = 'green'
        for side, label, joint_name in DXL_CURRENT_JOINTS:
            entry = self.latest_dxl_current.get(joint_name, {})
            current = entry.get('current_raw')
            error_code = self._nonzero_or_none(entry.get('error_code'))
            hardware_status = self._nonzero_or_none(entry.get('hardware_error_status'))
            fault = bool(error_code is not None or hardware_status is not None)
            baseline = max(
                float(self.dxl_current_baseline.get(joint_name, self.dxl_current_baseline_floor)),
                self.dxl_current_baseline_floor)
            ratio = None if current is None else abs(float(current)) / baseline
            level = self._dxl_current_level(ratio, fault)
            worst_level = self._max_dxl_level(worst_level, level)
            joints[joint_name] = {
                'side': side,
                'label': label,
                'current_raw': self._clean_float(current) if current is not None else None,
                'baseline_raw': self._clean_float(baseline),
                'load_ratio': self._clean_float(ratio) if ratio is not None else None,
                'level': level,
                'fault': fault,
                'error_code': error_code,
                'hardware_error_status': hardware_status,
            }
        return {
            'state': 'ok',
            'topic': self.dxl_current_topic,
            'age_s': self._clean_float(age_s),
            'baseline_floor': self._clean_float(self.dxl_current_baseline_floor),
            'caution_ratio': self._clean_float(self.dxl_current_caution_ratio),
            'warn_ratio': self._clean_float(self.dxl_current_warn_ratio),
            'worst_level': worst_level,
            'joints': joints,
        }

    def _dxl_current_calibrating(self, now):
        if self.dxl_current_calibration_start is None:
            return False
        elapsed_s = (now - self.dxl_current_calibration_start).nanoseconds / 1e9
        return elapsed_s < self.dxl_current_calibration_s

    def _finalize_dxl_current_baseline(self):
        for joint_name in DXL_CURRENT_NAMES:
            samples = self.dxl_current_samples.get(joint_name) or []
            if samples:
                ordered = sorted(samples)
                middle = len(ordered) // 2
                if len(ordered) % 2:
                    baseline = ordered[middle]
                else:
                    baseline = 0.5 * (ordered[middle - 1] + ordered[middle])
            else:
                latest = self.latest_dxl_current.get(joint_name, {})
                current = latest.get('current_raw')
                baseline = (
                    abs(float(current))
                    if current is not None
                    else self.dxl_current_baseline_floor
                )
            self.dxl_current_baseline[joint_name] = max(
                float(baseline), self.dxl_current_baseline_floor)

    def _dxl_current_level(self, ratio, fault):
        if fault:
            return 'fault'
        if ratio is None:
            return 'unknown'
        if ratio >= self.dxl_current_warn_ratio:
            return 'red'
        if ratio >= self.dxl_current_caution_ratio:
            return 'orange'
        return 'green'

    def _max_dxl_level(self, current, candidate):
        priority = {
            'unknown': 0,
            'green': 1,
            'orange': 2,
            'red': 3,
            'fault': 4,
        }
        return candidate if priority.get(candidate, 0) > priority.get(current, 0) else current

    def _nonzero_or_none(self, value):
        if value is None:
            return None
        value = int(round(float(value)))
        return value if value != 0 else None

    def _table_relative_snapshot(self):
        msg = self.latest_odom
        if not self.table_reference_enabled or msg is None:
            return None
        pose = msg.pose.pose
        base_yaw = self._yaw_from_quaternion(pose.orientation)
        dx = self.table_x_m - pose.position.x
        dy = self.table_y_m - pose.position.y
        bearing = math.atan2(dy, dx)
        return {
            'table_x_m': self._clean_float(self.table_x_m),
            'table_y_m': self._clean_float(self.table_y_m),
            'table_yaw_deg': self._clean_float(math.degrees(self.table_yaw_rad)),
            'dx_m': self._clean_float(dx),
            'dy_m': self._clean_float(dy),
            'distance_m': self._clean_float(math.hypot(dx, dy)),
            'bearing_to_table_deg': self._clean_float(math.degrees(bearing)),
            'heading_to_table_error_deg': self._clean_float(
                math.degrees(self._normalize_angle(bearing - base_yaw))),
            'base_yaw_minus_table_yaw_deg': self._clean_float(
                math.degrees(self._normalize_angle(base_yaw - self.table_yaw_rad))),
        }

    def _stamp_sec(self, stamp):
        return self._clean_float(stamp.sec + stamp.nanosec / 1e9)

    def _float_list(self, values):
        return [self._clean_float(value) for value in values]

    def _clean_float(self, value):
        value = float(value)
        if not math.isfinite(value):
            return None
        return value

    def _safe_float(self, value):
        try:
            return self._clean_float(value)
        except (TypeError, ValueError):
            return None

    def _yaw_from_quaternion(self, quat):
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _arm_status(self, arm):
        goal = self.latest.get(f'{arm}_goal')
        current = self.latest.get(f'{arm}_current')
        if goal is None or current is None:
            return {
                'available': False,
                'fresh': False,
                'ok': False,
                'pos_m': None,
                'ori_deg': None,
                'goal_frame': None,
                'current_frame': None,
            }

        now = self.get_clock().now()
        fresh = (
            (now - goal['received_time']).nanoseconds <= self.stale_timeout_s * 1e9 and
            (now - current['received_time']).nanoseconds <= self.stale_timeout_s * 1e9
        )
        goal_msg = goal['msg']
        current_msg = current['msg']
        pos_m = self._position_error(current_msg.pose, goal_msg.pose)
        ori_deg = self._orientation_error_deg(current_msg.pose, goal_msg.pose)
        ok = bool(
            fresh and
            pos_m <= self.pos_threshold_m and
            ori_deg <= self.ori_threshold_deg
        )
        return {
            'available': True,
            'fresh': bool(fresh),
            'ok': ok,
            'pos_m': round(pos_m, 4),
            'ori_deg': round(ori_deg, 2),
            'goal_frame': goal_msg.header.frame_id,
            'current_frame': current_msg.header.frame_id,
        }

    def _position_error(self, pose_a, pose_b):
        dx = pose_a.position.x - pose_b.position.x
        dy = pose_a.position.y - pose_b.position.y
        dz = pose_a.position.z - pose_b.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _orientation_error_deg(self, pose_a, pose_b):
        qa = pose_a.orientation
        qb = pose_b.orientation
        dot = qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w
        norm_a = math.sqrt(qa.x * qa.x + qa.y * qa.y + qa.z * qa.z + qa.w * qa.w)
        norm_b = math.sqrt(qb.x * qb.x + qb.y * qb.y + qb.z * qb.z + qb.w * qb.w)
        if norm_a <= 1e-9 or norm_b <= 1e-9:
            return 180.0
        dot = abs(dot / (norm_a * norm_b))
        dot = min(max(dot, 0.0), 1.0)
        return math.degrees(2.0 * math.acos(dot))


def main(args=None):
    rclpy.init(args=args)
    node = AlignmentStatus()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
