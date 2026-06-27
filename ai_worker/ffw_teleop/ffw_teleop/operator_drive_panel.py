import json
import math
import os
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


PANEL_WIDTH = 560
PANEL_HEIGHT = 980
KEY_RELEASE_DEBOUNCE_MS = 60
MOVE_ACTIONS = ('forward', 'backward', 'left', 'right', 'rot_left', 'rot_right')
HEAD_ACTIONS = ('head_up', 'head_down', 'head_left', 'head_right')
ACTION_LABELS = {
    'forward': 'FORWARD',
    'backward': 'BACK',
    'left': 'LEFT',
    'right': 'RIGHT',
    'rot_left': 'ROT L',
    'rot_right': 'ROT R',
    'head_up': 'HEAD UP',
    'head_down': 'HEAD DOWN',
    'head_left': 'HEAD L',
    'head_right': 'HEAD R',
}
KEY_ACTIONS = {
    'w': 'forward',
    's': 'backward',
    'a': 'left',
    'd': 'right',
    'q': 'rot_left',
    'e': 'rot_right',
}
HEAD_KEY_ACTIONS = {
    'up': 'head_up',
    'down': 'head_down',
    'left': 'head_left',
    'right': 'head_right',
}


class OperatorDrivePanel(Node):

    def __init__(self):
        super().__init__('operator_drive_panel')

        self.declare_parameter('window_title', 'Teleop Drive Control')
        self.declare_parameter('window_width', PANEL_WIDTH)
        self.declare_parameter('window_height', PANEL_HEIGHT)
        self.declare_parameter('window_x', 80)
        self.declare_parameter('window_y', 560)
        self.declare_parameter('show_hz', 30.0)
        self.declare_parameter('keyboard_cmd_vel_topic', '/teleop/keyboard_cmd_vel')
        self.declare_parameter('keyboard_enabled_topic', '/teleop/keyboard_drive/enabled')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_mux_status_topic', '/teleop/cmd_vel_mux/status')
        self.declare_parameter('monitor_head_cmd_topic', '/teleop/monitor_head_cmd')
        self.declare_parameter('head_enabled_topic', '/teleop/head_drive/enabled')
        self.declare_parameter('head_mux_status_topic', '/teleop/head_mux/status')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('head_pan_joint', 'head_joint1')
        self.declare_parameter('head_tilt_joint', 'head_joint2')
        self.declare_parameter('keyboard_linear_x_mps', 0.24)
        self.declare_parameter('keyboard_linear_y_mps', 0.24)
        self.declare_parameter('keyboard_angular_z_radps', 0.40)
        self.declare_parameter('default_drive_speed_scale', 0.5)
        self.declare_parameter('boost_drive_speed_scale', 1.0)
        self.declare_parameter('head_pan_step_deg', 3.0)
        self.declare_parameter('head_tilt_step_deg', 3.0)
        self.declare_parameter('head_min_rad', -1.0)
        self.declare_parameter('head_max_rad', 1.0)
        self.declare_parameter('head_command_duration_s', 0.25)
        self.declare_parameter('click_jog_duration_s', 0.25)
        self.declare_parameter('mouse_hold_timeout_s', 0.75)
        self.declare_parameter('mouse_max_hold_s', 8.0)
        self.declare_parameter('operator_ok_topic', '/teleop/operator_ok')
        self.declare_parameter('ok_overlay_duration_s', 3.0)
        self.declare_parameter('layout_command_topic', '/teleop/operator_layout/command')
        self.declare_parameter('layout_status_topic', '/teleop/operator_layout/status')
        self.declare_parameter('headless_ok', True)

        self.window_title = str(self.get_parameter('window_title').value).strip()
        self.window_width = max(int(self.get_parameter('window_width').value), 320)
        self.window_height = max(int(self.get_parameter('window_height').value), 360)
        self.window_x = int(self.get_parameter('window_x').value)
        self.window_y = int(self.get_parameter('window_y').value)
        show_hz = max(float(self.get_parameter('show_hz').value), 5.0)
        self.tick_period_s = 1.0 / show_hz
        self.tick_period_ms = max(int(round(1000.0 / show_hz)), 1)
        self.keyboard_cmd_vel_topic = str(
            self.get_parameter('keyboard_cmd_vel_topic').value).strip()
        self.keyboard_enabled_topic = str(
            self.get_parameter('keyboard_enabled_topic').value).strip()
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value).strip()
        self.cmd_vel_mux_status_topic = str(
            self.get_parameter('cmd_vel_mux_status_topic').value).strip()
        self.monitor_head_cmd_topic = str(
            self.get_parameter('monitor_head_cmd_topic').value).strip()
        self.head_enabled_topic = str(
            self.get_parameter('head_enabled_topic').value).strip()
        self.head_mux_status_topic = str(
            self.get_parameter('head_mux_status_topic').value).strip()
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value).strip()
        self.head_pan_joint = str(self.get_parameter('head_pan_joint').value).strip()
        self.head_tilt_joint = str(self.get_parameter('head_tilt_joint').value).strip()
        self.keyboard_linear_x_mps = max(
            float(self.get_parameter('keyboard_linear_x_mps').value), 0.0)
        self.keyboard_linear_y_mps = max(
            float(self.get_parameter('keyboard_linear_y_mps').value), 0.0)
        self.keyboard_angular_z_radps = max(
            float(self.get_parameter('keyboard_angular_z_radps').value), 0.0)
        self.default_drive_speed_scale = self._clamp(
            float(self.get_parameter('default_drive_speed_scale').value), 0.05, 1.0)
        self.boost_drive_speed_scale = self._clamp(
            float(self.get_parameter('boost_drive_speed_scale').value), 0.05, 1.0)
        if self.boost_drive_speed_scale < self.default_drive_speed_scale:
            self.boost_drive_speed_scale = self.default_drive_speed_scale
        self.head_pan_step_rad = math.radians(
            max(float(self.get_parameter('head_pan_step_deg').value), 0.1))
        self.head_tilt_step_rad = math.radians(
            max(float(self.get_parameter('head_tilt_step_deg').value), 0.1))
        self.head_min_rad = float(self.get_parameter('head_min_rad').value)
        self.head_max_rad = float(self.get_parameter('head_max_rad').value)
        if self.head_min_rad > self.head_max_rad:
            self.head_min_rad, self.head_max_rad = self.head_max_rad, self.head_min_rad
        self.head_command_duration_s = max(
            float(self.get_parameter('head_command_duration_s').value), 0.05)
        self.click_jog_duration_s = max(
            float(self.get_parameter('click_jog_duration_s').value), 0.05)
        self.mouse_hold_timeout_s = max(
            float(self.get_parameter('mouse_hold_timeout_s').value), 0.5)
        self.mouse_max_hold_s = max(
            float(self.get_parameter('mouse_max_hold_s').value), 0.0)
        self.operator_ok_topic = str(self.get_parameter('operator_ok_topic').value).strip()
        self.ok_overlay_duration_s = max(
            float(self.get_parameter('ok_overlay_duration_s').value), 0.1)
        self.layout_command_topic = str(
            self.get_parameter('layout_command_topic').value).strip()
        self.layout_status_topic = str(
            self.get_parameter('layout_status_topic').value).strip()
        self.headless_ok = self._as_bool(self.get_parameter('headless_ok').value)

        self.drive_enabled = False
        self.head_enabled = False
        self.focus_active = False
        self.pressed_actions = set()
        self.release_jobs = {}
        self.mouse_action = ''
        self.mouse_hold_started_s = 0.0
        self.timed_motion = None
        self.drive_speed_scale = self.default_drive_speed_scale
        self.status_text = 'LOCKED'
        self.head_status_text = 'HEAD LOCKED'
        self.active_text = '--'
        self.last_mux_status = {}
        self.last_mux_status_time_s = 0.0
        self.last_head_mux_status = {}
        self.last_head_mux_status_time_s = 0.0
        self.last_layout_status = 'Layout: ready'
        self.latest_head_position = {}
        self.latest_joint_state_time_s = 0.0
        self.gui_available = False
        self.root = None
        self.tk = None
        self.messagebox = None
        self.headless_timer = None
        self.closing = False

        drive_qos = QoSProfile(depth=1)
        drive_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        drive_qos.durability = DurabilityPolicy.VOLATILE
        enabled_qos = QoSProfile(depth=1)
        enabled_qos.reliability = ReliabilityPolicy.RELIABLE
        enabled_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cmd_pub = self.create_publisher(Twist, self.keyboard_cmd_vel_topic, drive_qos)
        self.enabled_pub = self.create_publisher(Bool, self.keyboard_enabled_topic, enabled_qos)
        self.head_cmd_pub = self.create_publisher(
            JointTrajectory, self.monitor_head_cmd_topic, 10)
        self.head_enabled_pub = self.create_publisher(
            Bool, self.head_enabled_topic, enabled_qos)
        self.ok_pub = self.create_publisher(String, self.operator_ok_topic, 10)
        self.layout_command_pub = None
        if self.layout_command_topic:
            self.layout_command_pub = self.create_publisher(
                String, self.layout_command_topic, 10)
        self.layout_status_sub = None
        if self.layout_status_topic:
            self.layout_status_sub = self.create_subscription(
                String, self.layout_status_topic, self._layout_status_callback, 10)
        self.mux_status_sub = None
        if self.cmd_vel_mux_status_topic:
            self.mux_status_sub = self.create_subscription(
                String, self.cmd_vel_mux_status_topic, self._mux_status_callback, 10)
        self.head_mux_status_sub = None
        if self.head_mux_status_topic:
            self.head_mux_status_sub = self.create_subscription(
                String, self.head_mux_status_topic, self._head_mux_status_callback, 10)
        if self.joint_state_topic:
            self.create_subscription(JointState, self.joint_state_topic, self._joint_state_callback, 10)

        self._init_window()
        self._publish_enabled()
        self._publish_head_enabled()
        if not self.gui_available:
            self.headless_timer = self.create_timer(self.tick_period_s, self._headless_tick)
        self.get_logger().info(
            f'operator drive panel active: window={self.window_title!r}, '
            f'cmd={self.keyboard_cmd_vel_topic}, enabled={self.keyboard_enabled_topic}, '
            f'gui={self.gui_available}')

    def destroy_node(self):
        self.closing = True
        self.drive_enabled = False
        self.head_enabled = False
        self._clear_motion('SHUTDOWN - STOPPED')
        self._publish_enabled()
        self._publish_head_enabled()
        self._destroy_window()
        super().destroy_node()

    def run_gui(self):
        if self.root is None:
            return
        self.root.after(self.tick_period_ms, self._gui_tick)
        self.root.mainloop()

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _mux_connected(self):
        return (
            self.count_subscribers(self.keyboard_cmd_vel_topic) > 0 and
            self.count_subscribers(self.keyboard_enabled_topic) > 0
        )

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _mux_status_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            status = {}
        self.last_mux_status = status if isinstance(status, dict) else {}
        self.last_mux_status_time_s = self._now_s()

    def _head_mux_status_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            status = {}
        self.last_head_mux_status = status if isinstance(status, dict) else {}
        self.last_head_mux_status_time_s = self._now_s()

    def _layout_status_callback(self, msg):
        text = str(msg.data or '').strip()
        if not text:
            return
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            self.last_layout_status = f'Layout: {text}'
            self._update_display()
            return
        action = str(payload.get('action') or '').strip()
        status = str(payload.get('status') or '').strip()
        message = str(payload.get('message') or '').strip()
        parts = [part for part in (action, status, message) if part]
        self.last_layout_status = f'Layout: {" - ".join(parts) or text}'
        self._update_display()

    def _joint_state_callback(self, msg):
        now_s = self._now_s()
        for joint in (self.head_pan_joint, self.head_tilt_joint):
            if joint in msg.name:
                index = msg.name.index(joint)
                if index < len(msg.position):
                    self.latest_head_position[joint] = float(msg.position[index])
        if self.head_pan_joint in self.latest_head_position and self.head_tilt_joint in self.latest_head_position:
            self.latest_joint_state_time_s = now_s

    def _mux_status_fresh(self):
        return (
            self.last_mux_status_time_s > 0.0 and
            (self._now_s() - self.last_mux_status_time_s) <= 1.0
        )

    def _head_mux_status_fresh(self):
        return (
            self.last_head_mux_status_time_s > 0.0 and
            (self._now_s() - self.last_head_mux_status_time_s) <= 1.0
        )

    def _cmd_vel_publisher_count(self):
        return self.count_publishers(self.cmd_vel_topic) if self.cmd_vel_topic else 0

    def _cmd_vel_publishers(self):
        if not self.cmd_vel_topic:
            return []
        publishers = []
        try:
            infos = self.get_publishers_info_by_topic(self.cmd_vel_topic)
        except Exception:
            return []
        for info in infos:
            namespace = str(getattr(info, 'node_namespace', '') or '').strip()
            name = str(getattr(info, 'node_name', '') or '').strip()
            if not name:
                continue
            if namespace and namespace != '/':
                publishers.append(f'{namespace.rstrip("/")}/{name}')
            else:
                publishers.append(f'/{name}')
        return sorted(set(publishers))

    def _cmd_vel_conflict(self):
        return self._cmd_vel_publisher_count() > 1

    def _owner_display(self, mux_connected):
        if self._cmd_vel_conflict():
            return 'CMD_VEL CONFLICT - MULTIPLE PUBLISHERS', '#9b3f3f'
        if not mux_connected:
            return 'OWNER: MUX OFF / NOT CONNECTED', '#6d5228'
        if not self._mux_status_fresh():
            return 'OWNER: WAITING FOR MUX STATUS', '#6d5228'

        owner = str(self.last_mux_status.get('owner', '')).strip().lower()
        if owner == 'monitor':
            return 'OWNER: MONITOR', '#2d7d4c'
        if owner == 'leader':
            return 'OWNER: LEADER', '#385d8a'
        if owner == 'monitor_stale':
            return 'OWNER: MONITOR STALE - STOPPED', '#8a5b2f'
        if owner == 'leader_stale':
            return 'OWNER: LEADER STALE - STOPPED', '#7a3232'
        return 'OWNER: UNKNOWN', '#6d5228'

    def _mux_text(self, mux_connected):
        cmd_vel_publishers = self._cmd_vel_publisher_count()
        cmd_vel_text = f'cmd_vel pubs={cmd_vel_publishers}'
        publisher_names = self._cmd_vel_publishers()
        if cmd_vel_publishers > 1 and publisher_names:
            shown = ', '.join(publisher_names[:3])
            if len(publisher_names) > 3:
                shown += ', ...'
            cmd_vel_text = f'{cmd_vel_text} [{shown}]'
        if not mux_connected:
            return f'OFF - launch start_cmd_vel_mux:=true; {cmd_vel_text}'
        if not self._mux_status_fresh():
            return f'connected, waiting status; {cmd_vel_text}'
        owner = self.last_mux_status.get('owner', '--')
        output_topic = self.last_mux_status.get('output_topic', '--')
        return f'{owner} -> {output_topic}; {cmd_vel_text}'

    def _init_window(self):
        if not os.environ.get('DISPLAY') and self.headless_ok:
            self.get_logger().warn('DISPLAY is not set; drive panel running headless')
            return
        try:
            import tkinter as tk
            from tkinter import messagebox
        except Exception as exc:
            self.get_logger().error(f'tkinter is unavailable; drive panel GUI disabled: {exc}')
            return

        try:
            root = tk.Tk()
        except Exception as exc:
            self.get_logger().error(f'failed to create drive panel window: {exc}')
            return

        self.tk = tk
        self.messagebox = messagebox
        self.root = root
        self.gui_available = True
        root.title(self.window_title)
        root.configure(bg='#202226')
        root.minsize(PANEL_WIDTH, PANEL_HEIGHT)
        if self.window_x >= 0 and self.window_y >= 0:
            root.geometry(
                f'{self.window_width}x{self.window_height}+{self.window_x}+{self.window_y}')
        else:
            root.geometry(f'{self.window_width}x{self.window_height}')
        root.protocol('WM_DELETE_WINDOW', self._on_close)

        self.focus_var = tk.StringVar(value='NOT FOCUSED - CLICK PANEL')
        self.owner_var = tk.StringVar(value='OWNER: MUX OFF / NOT CONNECTED')
        self.drive_var = tk.StringVar(value='DRIVE LOCKED')
        self.status_var = tk.StringVar(value='status: LOCKED')
        self.active_var = tk.StringVar(value='active: --    mux: --')
        self.speed_var = tk.StringVar(value=self._speed_text())
        self.speed_button_var = tk.StringVar(value=self._speed_button_text())
        self.distance_cm_var = tk.StringVar(value='10')
        self.rotate_deg_var = tk.StringVar(value='10')
        self.head_var = tk.StringVar(value='head: --')
        self.head_drive_var = tk.StringVar(value='HEAD LOCKED')
        self.layout_status_var = tk.StringVar(value=self.last_layout_status)

        outer = tk.Frame(root, bg='#202226')
        outer.pack(fill='both', expand=True, padx=16, pady=14)

        self.focus_label = tk.Label(
            outer,
            textvariable=self.focus_var,
            bg='#4b5159',
            fg='#f6f8fa',
            font=('Helvetica', 13, 'bold'),
            height=2,
        )
        self.focus_label.pack(fill='x')

        self.owner_label = tk.Label(
            outer,
            textvariable=self.owner_var,
            bg='#6d5228',
            fg='#ffffff',
            font=('Helvetica', 14, 'bold'),
            height=2,
        )
        self.owner_label.pack(fill='x', pady=(10, 0))

        self.drive_button = tk.Button(
            outer,
            textvariable=self.drive_var,
            command=lambda: self._set_drive_enabled(not self.drive_enabled),
            font=('Helvetica', 15, 'bold'),
            height=2,
            relief='raised',
        )
        self.drive_button.pack(fill='x', pady=(12, 6))

        self.status_label = tk.Label(
            outer,
            textvariable=self.status_var,
            bg='#202226',
            fg='#e6edf3',
            anchor='w',
            font=('Helvetica', 11),
        )
        self.status_label.pack(fill='x', pady=(4, 0))

        self.active_label = tk.Label(
            outer,
            textvariable=self.active_var,
            bg='#202226',
            fg='#b7c2cc',
            anchor='w',
            font=('Helvetica', 10),
        )
        self.active_label.pack(fill='x', pady=(2, 0))

        self.speed_label = tk.Label(
            outer,
            textvariable=self.speed_var,
            bg='#202226',
            fg='#9ca6af',
            anchor='w',
            font=('Helvetica', 10),
        )
        self.speed_label.pack(fill='x', pady=(2, 12))

        self.speed_button = tk.Button(
            outer,
            textvariable=self.speed_button_var,
            command=self._toggle_drive_speed_scale,
            font=('Helvetica', 11, 'bold'),
            height=1,
            bg='#344258',
            fg='#ffffff',
            activebackground='#415a78',
            activeforeground='#ffffff',
        )
        self.speed_button.pack(fill='x', pady=(0, 12))

        grid = tk.Frame(outer, bg='#202226')
        grid.pack(fill='x', expand=False)
        for col in range(3):
            grid.grid_columnconfigure(col, weight=1)
        for row in range(4):
            grid.grid_rowconfigure(row, weight=1)

        self.action_buttons = {}
        self._make_motion_button(grid, 'FORWARD', 'forward', 0, 1)
        self._make_motion_button(grid, 'LEFT', 'left', 1, 0)
        stop_button = tk.Button(
            grid,
            text='STOP',
            command=lambda: self._clear_motion('STOP - STOPPED'),
            font=('Helvetica', 13, 'bold'),
            bg='#7a3232',
            fg='#ffffff',
            activebackground='#9b3f3f',
            activeforeground='#ffffff',
        )
        stop_button.grid(row=1, column=1, sticky='nsew', padx=5, pady=5)
        self._make_motion_button(grid, 'RIGHT', 'right', 1, 2)
        self._make_motion_button(grid, 'BACK', 'backward', 2, 1)
        self._make_motion_button(grid, 'ROT L', 'rot_left', 3, 0)
        self._make_motion_button(grid, 'ROT R', 'rot_right', 3, 2)

        precise_frame = tk.LabelFrame(
            outer,
            text='DISTANCE / ANGLE JOG',
            bg='#202226',
            fg='#e6edf3',
            font=('Helvetica', 11, 'bold'),
            labelanchor='n',
        )
        precise_frame.pack(fill='x', pady=(12, 0))
        for col in range(5):
            precise_frame.grid_columnconfigure(col, weight=1)
        tk.Label(
            precise_frame,
            text='cm',
            bg='#202226',
            fg='#b7c2cc',
            font=('Helvetica', 10),
        ).grid(row=0, column=0, sticky='e', padx=4, pady=4)
        self.distance_entry = tk.Entry(
            precise_frame,
            textvariable=self.distance_cm_var,
            width=7,
            justify='center',
            font=('Helvetica', 11),
        )
        self.distance_entry.grid(row=0, column=1, sticky='ew', padx=4, pady=4)
        tk.Label(
            precise_frame,
            text='deg',
            bg='#202226',
            fg='#b7c2cc',
            font=('Helvetica', 10),
        ).grid(row=0, column=2, sticky='e', padx=4, pady=4)
        self.rotate_entry = tk.Entry(
            precise_frame,
            textvariable=self.rotate_deg_var,
            width=7,
            justify='center',
            font=('Helvetica', 11),
        )
        self.rotate_entry.grid(row=0, column=3, sticky='ew', padx=4, pady=4)
        tk.Button(
            precise_frame,
            text='STOP',
            command=lambda: self._clear_motion('STOP - STOPPED'),
            font=('Helvetica', 10, 'bold'),
            bg='#7a3232',
            fg='#ffffff',
        ).grid(row=0, column=4, sticky='ew', padx=4, pady=4)
        self._make_timed_button(precise_frame, 'FWD', 'forward', 1, 1)
        self._make_timed_button(precise_frame, 'LEFT', 'left', 2, 0)
        self._make_timed_button(precise_frame, 'RIGHT', 'right', 2, 2)
        self._make_timed_button(precise_frame, 'BACK', 'backward', 3, 1)
        self._make_timed_button(precise_frame, 'ROT L', 'rot_left', 2, 3)
        self._make_timed_button(precise_frame, 'ROT R', 'rot_right', 2, 4)

        head_frame = tk.LabelFrame(
            outer,
            text='ZED / HEAD',
            bg='#202226',
            fg='#e6edf3',
            font=('Helvetica', 11, 'bold'),
            labelanchor='n',
        )
        head_frame.pack(fill='x', pady=(12, 0))
        for col in range(3):
            head_frame.grid_columnconfigure(col, weight=1)
        self.head_drive_button = tk.Button(
            head_frame,
            textvariable=self.head_drive_var,
            command=lambda: self._set_head_enabled(not self.head_enabled),
            font=('Helvetica', 11, 'bold'),
            height=1,
            bg='#5b4444',
            fg='#ffffff',
        )
        self.head_drive_button.grid(row=0, column=0, columnspan=3, sticky='ew', padx=5, pady=5)
        tk.Label(
            head_frame,
            textvariable=self.head_var,
            bg='#202226',
            fg='#b7c2cc',
            anchor='w',
            font=('Helvetica', 10),
        ).grid(row=1, column=0, columnspan=3, sticky='ew', padx=5, pady=(0, 5))
        self._make_head_button(head_frame, 'UP', 'head_up', 2, 1)
        self._make_head_button(head_frame, 'LEFT', 'head_left', 3, 0)
        self._make_head_button(head_frame, 'RIGHT', 'head_right', 3, 2)
        self._make_head_button(head_frame, 'DOWN', 'head_down', 4, 1)

        ok_button = tk.Button(
            outer,
            text='OK SIGN',
            command=self._send_ok,
            font=('Helvetica', 12, 'bold'),
            bg='#2d7d4c',
            fg='#ffffff',
            activebackground='#35945a',
            activeforeground='#ffffff',
            height=2,
        )
        ok_button.pack(fill='x', pady=(12, 0))

        layout_frame = tk.LabelFrame(
            outer,
            text='WINDOW LAYOUT',
            bg='#202226',
            fg='#e6edf3',
            font=('Helvetica', 11, 'bold'),
            labelanchor='n',
        )
        layout_frame.pack(fill='x', pady=(12, 0))
        for col in range(3):
            layout_frame.grid_columnconfigure(col, weight=1)
        layout_buttons = [
            ('SAVE', lambda: self._publish_layout_command('save')),
            ('RESTORE', lambda: self._publish_layout_command('restore')),
            ('RESET', self._confirm_reset_layout),
        ]
        for col, (text, command) in enumerate(layout_buttons):
            tk.Button(
                layout_frame,
                text=text,
                command=command,
                font=('Helvetica', 10, 'bold'),
                height=1,
            ).grid(row=0, column=col, sticky='ew', padx=5, pady=5)
        tk.Label(
            layout_frame,
            textvariable=self.layout_status_var,
            bg='#202226',
            fg='#9ca6af',
            anchor='w',
            wraplength=max(self.window_width - 70, 280),
            font=('Helvetica', 10),
        ).grid(row=1, column=0, columnspan=3, sticky='ew', padx=5, pady=(0, 5))

        root.bind('<FocusIn>', self._on_focus_in)
        root.bind('<FocusOut>', self._on_focus_out)
        root.bind('<KeyPress>', self._on_key_press)
        root.bind('<KeyRelease>', self._on_key_release)
        root.bind('<Button-1>', self._focus_panel, add='+')
        for entry in (self.distance_entry, self.rotate_entry):
            entry.bind('<Button-1>', self._focus_entry, add='+')
            entry.bind('<FocusIn>', self._on_entry_focus_in, add='+')
        root.bind_all('<ButtonRelease-1>', self._on_any_mouse_release, add='+')
        root.after(200, self._request_initial_focus)
        self._update_display()

    def _make_motion_button(self, parent, label, action, row, col):
        button = self.tk.Button(
            parent,
            text=label,
            font=('Helvetica', 13, 'bold'),
            bg='#344258',
            fg='#ffffff',
            activebackground='#415a78',
            activeforeground='#ffffff',
        )
        button.grid(row=row, column=col, sticky='nsew', padx=5, pady=5)
        button.bind('<ButtonPress-1>', lambda event, action=action: self._mouse_press(action))
        button.bind('<ButtonRelease-1>', lambda event, action=action: self._mouse_release(action))
        self.action_buttons[action] = button

    def _make_timed_button(self, parent, label, action, row, col):
        button = self.tk.Button(
            parent,
            text=label,
            font=('Helvetica', 10, 'bold'),
            bg='#344258',
            fg='#ffffff',
            activebackground='#415a78',
            activeforeground='#ffffff',
            command=lambda action=action: self._start_timed_motion(action),
        )
        button.grid(row=row, column=col, sticky='ew', padx=4, pady=4)

    def _make_head_button(self, parent, label, action, row, col):
        button = self.tk.Button(
            parent,
            text=label,
            font=('Helvetica', 11, 'bold'),
            bg='#344258',
            fg='#ffffff',
            activebackground='#415a78',
            activeforeground='#ffffff',
            command=lambda action=action: self._head_jog(action),
        )
        button.grid(row=row, column=col, sticky='nsew', padx=5, pady=5)

    def _request_initial_focus(self):
        if self.root is None or self.closing:
            return
        try:
            self.root.focus_force()
        except Exception:
            self.root.focus_set()
        self._poll_focus()

    def _focus_panel(self, event=None):
        if event is not None and self._event_from_entry(event):
            return
        if self.root is not None:
            self.root.focus_set()

    def _focus_entry(self, event):
        widget = getattr(event, 'widget', None)
        if widget is not None:
            widget.focus_set()

    def _on_entry_focus_in(self, event=None):
        del event
        self.focus_active = True
        self._update_display()

    def _on_focus_in(self, event=None):
        del event
        self.focus_active = True
        self._update_display()

    def _on_focus_out(self, event=None):
        del event
        if self.root is not None:
            self.root.after(20, self._poll_focus)

    def _poll_focus(self):
        if self.root is None or self.closing:
            return
        focused = self.root.focus_get() is not None
        if self.focus_active and not focused:
            self.focus_active = False
            self._clear_motion('FOCUS LOST - STOPPED')
        elif focused:
            self.focus_active = True
        self._update_display()

    def _on_key_press(self, event):
        key = str(event.keysym).lower()
        if self._event_from_entry(event):
            return
        if key == 'k':
            self._set_drive_enabled(not self.drive_enabled)
            return
        if key == 'h':
            self._set_head_enabled(not self.head_enabled)
            return
        if key == 'space':
            self._clear_motion('SPACE - STOPPED')
            return
        if key == 'o':
            self._send_ok()
            return

        head_action = HEAD_KEY_ACTIONS.get(key)
        if head_action is not None:
            self._head_jog(head_action)
            return

        action = KEY_ACTIONS.get(key)
        if action is None:
            return
        self._cancel_release_job(action)
        if not self._ready_for_manual_drive():
            return
        self.pressed_actions.add(action)
        self.status_text = f'KEY {ACTION_LABELS[action]}'
        self._publish_current_command()
        self._update_display()

    def _event_from_entry(self, event):
        widget = getattr(event, 'widget', None)
        if widget is None or self.tk is None:
            return False
        try:
            return isinstance(widget, self.tk.Entry)
        except TypeError:
            return False

    def _on_key_release(self, event):
        if self._event_from_entry(event):
            return
        action = KEY_ACTIONS.get(str(event.keysym).lower())
        if action is None or self.root is None:
            return
        self._cancel_release_job(action)
        self.release_jobs[action] = self.root.after(
            KEY_RELEASE_DEBOUNCE_MS,
            lambda action=action: self._release_key_action(action),
        )

    def _release_key_action(self, action):
        self.release_jobs.pop(action, None)
        if action in self.pressed_actions:
            self.pressed_actions.discard(action)
            self.status_text = 'DRIVE ON - STOPPED' if not self._active_actions() else 'DRIVE ON'
            self._publish_current_command()
            self._update_display()

    def _cancel_release_job(self, action):
        job = self.release_jobs.pop(action, None)
        if job is not None and self.root is not None:
            try:
                self.root.after_cancel(job)
            except Exception:
                pass

    def _cancel_all_release_jobs(self):
        for action in list(self.release_jobs):
            self._cancel_release_job(action)

    def _mouse_press(self, action):
        if action not in MOVE_ACTIONS:
            return
        self._focus_panel()
        self.focus_active = True
        if not self._ready_for_manual_drive(require_focus=False):
            return
        self.mouse_action = action
        self.mouse_hold_started_s = time.time()
        self.status_text = f'MOUSE {ACTION_LABELS[action]}'
        self._publish_current_command()
        self._update_display()

    def _mouse_release(self, action):
        if self.mouse_action == action:
            self.mouse_action = ''
            self.mouse_hold_started_s = 0.0
            self.status_text = 'DRIVE ON - STOPPED' if not self._active_actions() else 'DRIVE ON'
            self._publish_current_command()
            self._update_display()

    def _on_any_mouse_release(self, event=None):
        del event
        if self.mouse_action:
            self.mouse_action = ''
            self.mouse_hold_started_s = 0.0
            self.status_text = 'DRIVE ON - STOPPED'
            self._publish_current_command()
            self._update_display()

    def _ready_for_manual_drive(self, require_focus=True):
        if not self.drive_enabled:
            self.status_text = 'LOCKED - CLICK DRIVE ON FIRST'
            self._publish_zero()
            self._update_display()
            return False
        if require_focus and not self.focus_active:
            self.status_text = 'NOT FOCUSED - CLICK PANEL'
            self._clear_motion('NOT FOCUSED - STOPPED')
            return False
        if not self._mux_connected():
            self.status_text = 'DRIVE ON - WAITING FOR MUX'
            self._update_display()
        return True

    def _set_drive_enabled(self, enabled):
        self.drive_enabled = bool(enabled)
        if self.drive_enabled and not self._mux_connected():
            status_text = 'DRIVE ON - WAITING FOR MUX'
        else:
            status_text = 'DRIVE ON' if self.drive_enabled else 'LOCKED'
        self._clear_motion(status_text)
        self._publish_enabled()
        self._update_display()

    def _set_head_enabled(self, enabled):
        self.head_enabled = bool(enabled)
        self.head_status_text = 'HEAD ON' if self.head_enabled else 'HEAD LOCKED'
        self._publish_head_enabled()
        self._update_display()

    def _clear_motion(self, status_text=None):
        self._cancel_all_release_jobs()
        self.pressed_actions.clear()
        self.mouse_action = ''
        self.mouse_hold_started_s = 0.0
        self.timed_motion = None
        if status_text:
            self.status_text = status_text
        elif self.drive_enabled:
            self.status_text = 'DRIVE ON - STOPPED'
        self._publish_zero()
        self.active_text = '--'

    def _headless_tick(self):
        if self.drive_enabled:
            self.drive_enabled = False
            self._clear_motion('HEADLESS - DRIVE DISABLED')
            self._publish_enabled()
        if self.head_enabled:
            self.head_enabled = False
            self._publish_head_enabled()

    def _gui_tick(self):
        if self.closing or self.root is None:
            return
        try:
            rclpy.spin_once(self, timeout_sec=0.0)
        except Exception as exc:
            self.get_logger().warn(f'ROS spin_once failed in drive panel: {exc}')
        self._poll_focus()
        self._publish_drive_tick()
        self._update_display()
        if not self.closing and self.root is not None:
            self.root.after(self.tick_period_ms, self._gui_tick)

    def _publish_drive_tick(self):
        if not self.drive_enabled:
            return
        if self.mouse_action and self.mouse_max_hold_s > 0.0:
            hold_age_s = time.time() - self.mouse_hold_started_s
            if hold_age_s > self.mouse_max_hold_s:
                self.mouse_action = ''
                self.mouse_hold_started_s = 0.0
                self.status_text = 'MOUSE HOLD TIMEOUT - STOPPED'
        if self.timed_motion:
            end_s = float(self.timed_motion.get('end_s') or 0.0)
            if time.time() >= end_s:
                label = str(self.timed_motion.get('label') or 'TIMED')
                self.timed_motion = None
                self.status_text = f'{label} DONE - STOPPED'
        self._publish_current_command()

    def _publish_current_command(self):
        twist, active_text = self._make_twist()
        self.active_text = active_text
        if self.drive_enabled:
            self.cmd_pub.publish(twist)

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _publish_enabled(self):
        msg = Bool()
        msg.data = bool(self.drive_enabled)
        self.enabled_pub.publish(msg)

    def _publish_head_enabled(self):
        msg = Bool()
        msg.data = bool(self.head_enabled)
        self.head_enabled_pub.publish(msg)

    def _active_actions(self):
        actions = set(self.pressed_actions)
        if self.mouse_action:
            actions.add(self.mouse_action)
        return actions

    def _make_twist(self):
        if self.timed_motion:
            twist = self.timed_motion.get('twist') or Twist()
            label = str(self.timed_motion.get('label') or 'TIMED')
            return twist, label
        actions = self._active_actions()
        twist = Twist()
        twist.linear.x = self._axis_value(actions, 'forward', 'backward',
                                          self._effective_linear_x_mps())
        twist.linear.y = self._axis_value(actions, 'left', 'right',
                                          self._effective_linear_y_mps())
        twist.angular.z = self._axis_value(actions, 'rot_left', 'rot_right',
                                           self._effective_angular_z_radps())
        active = [ACTION_LABELS[action] for action in MOVE_ACTIONS if action in actions]
        return twist, '+'.join(active) if active else '--'

    def _axis_value(self, actions, positive_action, negative_action, magnitude):
        positive = positive_action in actions
        negative = negative_action in actions
        if positive and not negative:
            return magnitude
        if negative and not positive:
            return -magnitude
        return 0.0

    def _start_timed_motion(self, action):
        self._focus_panel()
        if not self._ready_for_manual_drive(require_focus=False):
            return
        twist = Twist()
        label = ACTION_LABELS.get(action, action.upper())
        if action in ('forward', 'backward', 'left', 'right'):
            distance_m = max(self._parse_float(self.distance_cm_var.get(), 0.0), 0.0) / 100.0
            if distance_m <= 0.0:
                self.status_text = 'DISTANCE MUST BE > 0'
                self._update_display()
                return
            if action == 'forward':
                twist.linear.x = self._effective_linear_x_mps()
            elif action == 'backward':
                twist.linear.x = -self._effective_linear_x_mps()
            elif action == 'left':
                twist.linear.y = self._effective_linear_y_mps()
            elif action == 'right':
                twist.linear.y = -self._effective_linear_y_mps()
            speed = abs(twist.linear.x or twist.linear.y)
            duration_s = distance_m / max(speed, 1e-6)
            label = f'{label} {distance_m * 100.0:.1f}cm'
        elif action in ('rot_left', 'rot_right'):
            angle_rad = math.radians(max(self._parse_float(self.rotate_deg_var.get(), 0.0), 0.0))
            if angle_rad <= 0.0:
                self.status_text = 'ANGLE MUST BE > 0'
                self._update_display()
                return
            twist.angular.z = (
                self._effective_angular_z_radps()
                if action == 'rot_left'
                else -self._effective_angular_z_radps()
            )
            duration_s = angle_rad / max(abs(twist.angular.z), 1e-6)
            label = f'{label} {math.degrees(angle_rad):.1f}deg'
        else:
            return

        self._cancel_all_release_jobs()
        self.pressed_actions.clear()
        self.mouse_action = ''
        self.mouse_hold_started_s = 0.0
        self.timed_motion = {
            'twist': twist,
            'end_s': time.time() + duration_s,
            'label': label,
        }
        self.status_text = f'TIMED {label} {duration_s:.2f}s'
        self._publish_current_command()
        self._update_display()

    def _parse_float(self, value, default):
        try:
            return float(str(value).strip())
        except (TypeError, ValueError):
            return default

    def _head_jog(self, action):
        self._focus_panel()
        if not self.head_enabled:
            self.head_status_text = 'HEAD LOCKED - CLICK HEAD ON FIRST'
            self._update_display()
            return
        if not self._head_mux_connected():
            self.head_status_text = 'HEAD ON - WAITING FOR MUX'
            self._update_display()
            return
        pan = self.latest_head_position.get(self.head_pan_joint)
        tilt = self.latest_head_position.get(self.head_tilt_joint)
        if pan is None or tilt is None:
            self.head_status_text = 'HEAD WAITING FOR JOINT STATE'
            self._update_display()
            return

        if action == 'head_up':
            tilt -= self.head_tilt_step_rad
        elif action == 'head_down':
            tilt += self.head_tilt_step_rad
        elif action == 'head_left':
            pan -= self.head_pan_step_rad
        elif action == 'head_right':
            pan += self.head_pan_step_rad
        else:
            return

        pan = self._clamp(pan, self.head_min_rad, self.head_max_rad)
        tilt = self._clamp(tilt, self.head_min_rad, self.head_max_rad)
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [self.head_pan_joint, self.head_tilt_joint]
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.velocities = [0.0, 0.0]
        point.accelerations = [0.0, 0.0]
        point.time_from_start.sec = int(self.head_command_duration_s)
        point.time_from_start.nanosec = int(
            (self.head_command_duration_s % 1.0) * 1_000_000_000)
        msg.points.append(point)
        self.head_cmd_pub.publish(msg)
        self.latest_head_position[self.head_pan_joint] = pan
        self.latest_head_position[self.head_tilt_joint] = tilt
        self.head_status_text = (
            f'{ACTION_LABELS.get(action, "HEAD")} '
            f'pan={pan:+.2f} tilt={tilt:+.2f}'
        )
        self._update_display()

    def _head_mux_connected(self):
        return (
            self.count_subscribers(self.monitor_head_cmd_topic) > 0 and
            self.count_subscribers(self.head_enabled_topic) > 0
        )

    def _clamp(self, value, low, high):
        return min(max(value, low), high)

    def _effective_linear_x_mps(self):
        return self.keyboard_linear_x_mps * self.drive_speed_scale

    def _effective_linear_y_mps(self):
        return self.keyboard_linear_y_mps * self.drive_speed_scale

    def _effective_angular_z_radps(self):
        return self.keyboard_angular_z_radps * self.drive_speed_scale

    def _speed_percent(self, scale=None):
        if scale is None:
            scale = self.drive_speed_scale
        return int(round(float(scale) * 100.0))

    def _speed_button_text(self):
        if abs(self.drive_speed_scale - self.boost_drive_speed_scale) < 1e-3:
            target = self._speed_percent(self.default_drive_speed_scale)
        else:
            target = self._speed_percent(self.boost_drive_speed_scale)
        return f'SPEED {self._speed_percent()}%  |  TOGGLE {target}%'

    def _toggle_drive_speed_scale(self):
        if abs(self.drive_speed_scale - self.boost_drive_speed_scale) < 1e-3:
            self.drive_speed_scale = self.default_drive_speed_scale
        else:
            self.drive_speed_scale = self.boost_drive_speed_scale
        if self.timed_motion:
            self.timed_motion = None
            self.status_text = f'SPEED {self._speed_percent()}% - TIMED STOPPED'
        else:
            self.status_text = f'SPEED {self._speed_percent()}%'
        self._publish_current_command()
        self._update_display()

    def _speed_text(self):
        return (
            f'speed: {self._speed_percent()}% '
            f'x/y {self._effective_linear_x_mps():.2f}/{self._effective_linear_y_mps():.2f} m/s, '
            f'rot {self._effective_angular_z_radps():.2f} rad/s '
            f'(max {self.keyboard_linear_x_mps:.2f}/{self.keyboard_linear_y_mps:.2f}, '
            f'{self.keyboard_angular_z_radps:.2f})'
        )

    def _update_display(self):
        if not self.gui_available or self.root is None:
            return
        mux_connected = self._mux_connected()
        owner_text, owner_bg = self._owner_display(mux_connected)
        self.owner_var.set(owner_text)
        self.owner_label.configure(bg=owner_bg)
        if self.focus_active:
            self.focus_var.set('FOCUSED - WASD ACTIVE')
            self.focus_label.configure(bg='#2d7d4c')
        else:
            self.focus_var.set('NOT FOCUSED - CLICK PANEL')
            self.focus_label.configure(bg='#7a5a2d')

        if self.drive_enabled and not mux_connected:
            drive_text = 'DRIVE ON - WAITING FOR MUX'
            drive_bg = '#6d5228'
        elif self.drive_enabled:
            drive_text = 'DRIVE ON - MONITOR CONTROL'
            drive_bg = '#2d7d4c'
        elif mux_connected:
            drive_text = 'DRIVE LOCKED - LEADER CONTROL'
            drive_bg = '#5b4444'
        else:
            drive_text = 'DRIVE LOCKED / MUX OFF'
            drive_bg = '#6d5228'
        self.drive_var.set(drive_text)
        self.drive_button.configure(bg=drive_bg, fg='#ffffff', activebackground=drive_bg)
        self.status_var.set(f'status: {self.status_text}')
        self.active_var.set(f'active: {self.active_text}    mux: {self._mux_text(mux_connected)}')
        self.speed_var.set(self._speed_text())
        self.speed_button_var.set(self._speed_button_text())
        self.layout_status_var.set(self.last_layout_status)
        self._update_head_display()

        button_bg = '#344258' if self.drive_enabled else '#33363d'
        active_bg = '#4f6f91'
        for action, button in self.action_buttons.items():
            bg = active_bg if action in self._active_actions() else button_bg
            button.configure(bg=bg, activebackground=active_bg)

    def _update_head_display(self):
        head_mux_connected = self._head_mux_connected()
        if self.head_enabled and head_mux_connected:
            head_text = 'HEAD ON - MONITOR ZED'
            head_bg = '#2d7d4c'
        elif self.head_enabled:
            head_text = 'HEAD ON - WAITING FOR MUX'
            head_bg = '#6d5228'
        elif head_mux_connected:
            head_text = 'HEAD LOCKED - LEADER HEAD'
            head_bg = '#5b4444'
        else:
            head_text = 'HEAD LOCKED / MUX OFF'
            head_bg = '#6d5228'
        self.head_drive_var.set(head_text)
        self.head_drive_button.configure(bg=head_bg, fg='#ffffff', activebackground=head_bg)

        pan = self.latest_head_position.get(self.head_pan_joint)
        tilt = self.latest_head_position.get(self.head_tilt_joint)
        if pan is None or tilt is None:
            pos_text = 'pos --,--'
        else:
            age_s = self._now_s() - self.latest_joint_state_time_s
            pos_text = f'pos pan={pan:+.2f} tilt={tilt:+.2f} age={age_s:.1f}s'
        owner = '--'
        if self._head_mux_status_fresh():
            owner = self.last_head_mux_status.get('owner', '--')
        self.head_var.set(f'{self.head_status_text} | owner={owner} | {pos_text}')

    def _publish_layout_command(self, command):
        command = str(command or '').strip().lower()
        if not command:
            return
        if self.layout_command_pub is None:
            self.last_layout_status = 'Layout: command topic disabled'
            self.get_logger().warn('layout command topic is disabled')
            self._update_display()
            return
        msg = String()
        msg.data = command
        self.layout_command_pub.publish(msg)
        self.last_layout_status = f'Layout: {command} requested'
        self.get_logger().info(f'layout command requested: {command}')
        self._update_display()

    def _confirm_reset_layout(self):
        if self.messagebox is not None:
            confirmed = self.messagebox.askyesno(
                'Reset Layout',
                'Delete the saved operator window layout?',
            )
            if not confirmed:
                return
        self._publish_layout_command('reset')

    def _send_ok(self):
        payload = {
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'event': 'ok',
            'source': 'drive_panel',
            'duration_s': self.ok_overlay_duration_s,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.ok_pub.publish(msg)
        self.status_text = 'OK SIGN SENT'
        self._update_display()

    def _on_close(self):
        self.closing = True
        self.drive_enabled = False
        self.head_enabled = False
        self._clear_motion('CLOSED - STOPPED')
        self._publish_enabled()
        self._publish_head_enabled()
        self._destroy_window()

    def _destroy_window(self):
        if self.root is None:
            return
        root = self.root
        self.root = None
        self.gui_available = False
        try:
            root.destroy()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = OperatorDrivePanel()
    try:
        if node.gui_available:
            node.run_gui()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
