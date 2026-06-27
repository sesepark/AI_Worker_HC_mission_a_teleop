import copy
import json
import os
import time

import cv2
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from std_msgs.msg import String


DEFAULT_MISSIONS = {
    'A': {
        'title': 'Mission A - Part Sorting',
        'summary': 'Identify requested parts and place the selected parts into the tray.',
        'specific_features': [
            'Monitor command parser placeholder',
            'Part detection overlay placeholder',
            'Tray placement assist placeholder',
        ],
    },
    'B': {
        'title': 'Mission B - Box Transport',
        'summary': 'Grasp boxes, move to the target table, and place them at the marked area.',
        'specific_features': [
            'Dual-arm box grasp assist placeholder',
            'Stop line and target table assist placeholder',
            'Transport cycle counter placeholder',
        ],
    },
    'C': {
        'title': 'Mission C - Sequential Assembly',
        'summary': 'Follow the monitor order and insert the requested nuts onto the peg set.',
        'specific_features': [
            'Assembly order parser placeholder',
            'Peg diameter and position assist placeholder',
            'Completion button assist placeholder',
        ],
    },
    'D': {
        'title': 'Mission D - Wheel Mounting',
        'summary': 'Mount the wheel, insert the bolt, and complete tightening with the drill.',
        'specific_features': [
            'Wheel and hub alignment assist placeholder',
            'Bolt and drill grasp assist placeholder',
            'Fastening alignment assist placeholder',
        ],
    },
}

MISSION_COLORS_BGR = {
    'A': (64, 128, 236),
    'B': (54, 166, 92),
    'C': (192, 132, 52),
    'D': (92, 92, 220),
}

MISSION_COLORS_TK = {
    'A': '#ec8040',
    'B': '#5ca636',
    'C': '#3484c0',
    'D': '#dc5c5c',
}


class MissionModeManager(Node):

    def __init__(self):
        super().__init__('mission_mode_manager')

        self.declare_parameter('profiles_config', '')
        self.declare_parameter('select_topic', '/teleop/mission/select')
        self.declare_parameter('mission_id_topic', '/teleop/mission/id')
        self.declare_parameter('mission_state_topic', '/teleop/mission/state')
        self.declare_parameter('mission_panel_topic', '/teleop/mission_panel/compressed')
        self.declare_parameter('publish_hz', 2.0)
        self.declare_parameter('panel_width', 1280)
        self.declare_parameter('panel_height', 720)
        self.declare_parameter('panel_jpeg_quality', 95)
        self.declare_parameter('initial_mission', '')
        self.declare_parameter('gui_enabled', True)
        self.declare_parameter('layout_command_topic', '/teleop/operator_layout/command')
        self.declare_parameter('layout_status_topic', '/teleop/operator_layout/status')
        self.declare_parameter('keyboard_drive_ui_enabled', False)
        self.declare_parameter('keyboard_cmd_vel_topic', '/teleop/keyboard_cmd_vel')
        self.declare_parameter('keyboard_enabled_topic', '/teleop/keyboard_drive/enabled')
        self.declare_parameter('keyboard_linear_x_mps', 0.1666667)
        self.declare_parameter('keyboard_linear_y_mps', 0.1666667)
        self.declare_parameter('keyboard_angular_z_radps', 0.25)
        self.declare_parameter('keyboard_publish_hz', 30.0)
        self.declare_parameter('keyboard_key_timeout_s', 0.15)
        self.declare_parameter('operator_ok_topic', '/teleop/operator_ok')
        self.declare_parameter('ok_overlay_duration_s', 3.0)

        self.profiles_config = str(self.get_parameter('profiles_config').value).strip()
        self.select_topic = str(self.get_parameter('select_topic').value).strip()
        self.mission_id_topic = str(self.get_parameter('mission_id_topic').value).strip()
        self.mission_state_topic = str(self.get_parameter('mission_state_topic').value).strip()
        self.mission_panel_topic = str(self.get_parameter('mission_panel_topic').value).strip()
        publish_hz = max(float(self.get_parameter('publish_hz').value), 0.2)
        self.panel_width = max(int(self.get_parameter('panel_width').value), 320)
        self.panel_height = max(int(self.get_parameter('panel_height').value), 180)
        self.panel_jpeg_quality = int(np.clip(
            int(self.get_parameter('panel_jpeg_quality').value), 1, 100))
        self.gui_enabled = self._as_bool(self.get_parameter('gui_enabled').value)
        self.layout_command_topic = str(
            self.get_parameter('layout_command_topic').value).strip()
        self.layout_status_topic = str(
            self.get_parameter('layout_status_topic').value).strip()
        self.keyboard_drive_ui_enabled = self._as_bool(
            self.get_parameter('keyboard_drive_ui_enabled').value)
        self.keyboard_cmd_vel_topic = str(
            self.get_parameter('keyboard_cmd_vel_topic').value).strip()
        self.keyboard_enabled_topic = str(
            self.get_parameter('keyboard_enabled_topic').value).strip()
        self.keyboard_linear_x_mps = max(
            float(self.get_parameter('keyboard_linear_x_mps').value), 0.0)
        self.keyboard_linear_y_mps = max(
            float(self.get_parameter('keyboard_linear_y_mps').value), 0.0)
        self.keyboard_angular_z_radps = max(
            float(self.get_parameter('keyboard_angular_z_radps').value), 0.0)
        keyboard_publish_hz = max(float(self.get_parameter('keyboard_publish_hz').value), 1.0)
        self.keyboard_key_timeout_s = max(
            float(self.get_parameter('keyboard_key_timeout_s').value), 0.05)
        self.operator_ok_topic = str(self.get_parameter('operator_ok_topic').value).strip()
        self.ok_overlay_duration_s = max(
            float(self.get_parameter('ok_overlay_duration_s').value), 0.1)

        profile_data = self._load_profiles(self.profiles_config)
        self.missions = profile_data['missions']
        initial = str(self.get_parameter('initial_mission').value).strip().upper()
        self.active_mission = self._normalize_mission(
            initial or profile_data.get('default_mission') or 'A') or 'A'
        self.updated_by = 'startup'
        self.root = None
        self.tk = None
        self.messagebox = None
        self.gui_widgets = {}
        self.keyboard_drive_enabled = False
        self.pressed_drive_keys = {}
        self.ok_overlay_until_s = 0.0

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        drive_qos = QoSProfile(depth=1)
        drive_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        drive_qos.durability = DurabilityPolicy.VOLATILE
        self.mission_id_pub = self.create_publisher(
            String, self.mission_id_topic, state_qos)
        self.mission_state_pub = self.create_publisher(
            String, self.mission_state_topic, state_qos)
        self.panel_pub = self.create_publisher(
            CompressedImage, self.mission_panel_topic, 1)
        self.keyboard_cmd_pub = None
        self.keyboard_enabled_pub = None
        if self.keyboard_drive_ui_enabled:
            self.keyboard_cmd_pub = self.create_publisher(
                Twist, self.keyboard_cmd_vel_topic, drive_qos)
            self.keyboard_enabled_pub = self.create_publisher(
                Bool, self.keyboard_enabled_topic, state_qos)
        self.operator_ok_pub = self.create_publisher(String, self.operator_ok_topic, 10)
        self.operator_ok_sub = self.create_subscription(
            String, self.operator_ok_topic, self._operator_ok_callback, 10)
        self.layout_command_pub = None
        if self.layout_command_topic:
            self.layout_command_pub = self.create_publisher(
                String, self.layout_command_topic, 10)
        self.select_sub = self.create_subscription(
            String, self.select_topic, self._select_callback, 10)
        self.layout_status_sub = None
        if self.layout_status_topic:
            self.layout_status_sub = self.create_subscription(
                String, self.layout_status_topic, self._layout_status_callback, 10)
        self.timer = self.create_timer(1.0 / publish_hz, self._publish_state)
        self.keyboard_timer = None
        if self.keyboard_drive_ui_enabled:
            self.keyboard_timer = self.create_timer(
                1.0 / keyboard_publish_hz, self._publish_keyboard_command)

        self._init_gui()
        if self.keyboard_drive_ui_enabled:
            self._publish_keyboard_enabled()
        self._publish_state()
        self.get_logger().info(
            f'mission mode manager active: mission={self.active_mission}, '
            f'select={self.select_topic}, state={self.mission_state_topic}, '
            f'panel={self.mission_panel_topic}, layout={self.layout_command_topic}')

    def run(self):
        if self.root is None:
            rclpy.spin(self)
            return
        self._schedule_ros_spin()
        self.root.mainloop()

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _load_profiles(self, path):
        missions = copy.deepcopy(DEFAULT_MISSIONS)
        data = {
            'default_mission': 'A',
            'missions': missions,
        }
        if not path:
            return data
        try:
            import yaml
            with open(os.path.expanduser(path), 'r', encoding='utf-8') as stream:
                loaded = yaml.safe_load(stream) or {}
        except OSError as exc:
            self.get_logger().warn(f'failed to read mission profiles {path}: {exc}')
            return data
        except Exception as exc:
            self.get_logger().warn(f'failed to parse mission profiles {path}: {exc}')
            return data

        loaded_missions = loaded.get('missions') or {}
        for mission_id, profile in loaded_missions.items():
            key = self._normalize_mission(mission_id)
            if key is None or not isinstance(profile, dict):
                continue
            missions[key] = {
                'title': str(profile.get('title') or missions.get(key, {}).get('title') or key),
                'summary': str(profile.get('summary') or ''),
                'specific_features': [
                    str(item) for item in profile.get('specific_features') or []
                ],
            }
        default = self._normalize_mission(loaded.get('default_mission')) or 'A'
        return {
            'default_mission': default,
            'missions': missions,
        }

    def _normalize_mission(self, value):
        text = str(value or '').strip().upper()
        if not text:
            return None
        if text.startswith('MISSION '):
            text = text.split()[-1]
        key = text[0]
        if key in ('A', 'B', 'C', 'D'):
            return key
        return None

    def _select_callback(self, msg):
        mission = self._normalize_mission(msg.data)
        if mission is None:
            self.get_logger().warn(f'ignoring unknown mission selection: {msg.data!r}')
            return
        self._set_mission(mission, 'topic')

    def _layout_status_callback(self, msg):
        text = str(msg.data or '').strip()
        if not text:
            return
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            self._set_layout_status(f'Layout: {text}')
            return
        action = str(payload.get('action') or '').strip()
        status = str(payload.get('status') or '').strip()
        message = str(payload.get('message') or '').strip()
        parts = []
        if action:
            parts.append(action)
        if status:
            parts.append(status)
        if message:
            parts.append(message)
        self._set_layout_status(f'Layout: {" - ".join(parts) or text}')

    def _operator_ok_callback(self, msg):
        text = str(msg.data or '').strip()
        if not text:
            return
        duration_s = self.ok_overlay_duration_s
        try:
            payload = json.loads(text)
            if str(payload.get('event') or '').strip().lower() != 'ok':
                return
            duration_s = max(float(payload.get('duration_s', duration_s)), 0.1)
        except (json.JSONDecodeError, TypeError, ValueError):
            if text.lower() != 'ok':
                return
        self.ok_overlay_until_s = time.time() + duration_s
        self._publish_state()

    def _set_mission(self, mission, source):
        mission = self._normalize_mission(mission)
        if mission is None:
            return
        changed = mission != self.active_mission
        self.active_mission = mission
        self.updated_by = source
        self._update_gui()
        self._publish_state()
        if changed:
            self.get_logger().info(f'mission switched to {mission} by {source}')

    def _state_payload(self):
        profile = self.missions.get(self.active_mission, {})
        now_s = time.time()
        return {
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'mission_id': self.active_mission,
            'title': str(profile.get('title') or self.active_mission),
            'summary': str(profile.get('summary') or ''),
            'specific_features': [
                str(item) for item in profile.get('specific_features') or []
            ],
            'updated_by': self.updated_by,
            'available_missions': sorted(self.missions.keys()),
            'ok_active': now_s < self.ok_overlay_until_s,
            'ok_until_sec': self.ok_overlay_until_s,
        }

    def _publish_state(self):
        payload = self._state_payload()
        mission_msg = String()
        mission_msg.data = self.active_mission
        self.mission_id_pub.publish(mission_msg)

        state_msg = String()
        state_msg.data = json.dumps(payload, sort_keys=True)
        self.mission_state_pub.publish(state_msg)

        panel = self._render_panel(payload)
        ok, encoded = cv2.imencode(
            '.jpg', panel, [int(cv2.IMWRITE_JPEG_QUALITY), self.panel_jpeg_quality])
        if ok:
            image_msg = CompressedImage()
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = 'teleop_mission_panel'
            image_msg.format = 'jpeg'
            image_msg.data = encoded.tobytes()
            self.panel_pub.publish(image_msg)

    def _render_panel(self, payload):
        width = self.panel_width
        height = self.panel_height
        scale = min(width / 640.0, height / 360.0)
        margin = int(round(20 * scale))
        header_h = int(round(72 * scale))
        def p(x, y):
            return int(round(x * scale)), int(round(y * scale))
        def line(value):
            return max(int(round(value * scale)), 1)
        mission_id = str(payload.get('mission_id') or 'A')
        title = str(payload.get('title') or '').strip()
        color = MISSION_COLORS_BGR.get(mission_id, (80, 80, 80))
        image = np.full((height, width, 3), (30, 32, 36), dtype=np.uint8)
        cv2.rectangle(image, (0, 0), (width - 1, header_h), color, -1)
        self._put_text(
            image, mission_id, p(20, 49), 1.12 * scale, (255, 255, 255), line(2))
        self._put_text(
            image,
            title,
            p(78, 34),
            0.58 * scale,
            (255, 255, 255),
            line(2),
        )
        self._put_text(
            image,
            'MISSION MODE',
            p(78, 60),
            0.42 * scale,
            (255, 255, 255),
            line(1),
        )

        y = int(round(112 * scale))
        y = self._draw_wrapped(
            image,
            str(payload.get('summary') or ''),
            (margin, y),
            width - 2 * margin,
            0.58 * scale,
            (234, 238, 242),
        )

        y += int(round(18 * scale))
        self._put_text(
            image, 'Specific feature slots', (margin, y),
            0.56 * scale, (210, 216, 222), line(1))
        y += int(round(30 * scale))
        for feature in payload.get('specific_features') or []:
            y = self._draw_wrapped(
                image,
                f'- {feature}',
                (int(round(36 * scale)), y),
                width - int(round(70 * scale)),
                0.50 * scale,
                (235, 238, 240),
            ) + int(round(4 * scale))
            if y > height - int(round(55 * scale)):
                break

        button_y = height - int(round(38 * scale))
        for index, key in enumerate(('A', 'B', 'C', 'D')):
            x0 = int(round(20 * scale + index * 70 * scale))
            is_active = key == mission_id
            fill = MISSION_COLORS_BGR.get(key, (80, 80, 80)) if is_active else (54, 56, 62)
            button_w = int(round(52 * scale))
            cv2.rectangle(
                image,
                (x0, button_y - int(round(23 * scale))),
                (x0 + button_w, button_y + int(round(8 * scale))),
                fill,
                -1,
            )
            cv2.rectangle(
                image,
                (x0, button_y - int(round(23 * scale))),
                (x0 + button_w, button_y + int(round(8 * scale))),
                (190, 196, 204),
                line(1),
            )
            self._put_text(
                image, key, (x0 + int(round(18 * scale)), button_y),
                0.58 * scale, (255, 255, 255), line(1))
        self._put_text(
            image,
            f'Updated by {payload.get("updated_by", "--")}',
            (width - int(round(245 * scale)), height - int(round(16 * scale))),
            0.45 * scale,
            (200, 206, 214),
            line(1),
        )
        if payload.get('ok_active'):
            self._draw_ok_overlay(image, scale)
        return image

    def _draw_ok_overlay(self, image, scale):
        height, width = image.shape[:2]
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (width - 1, height - 1), (32, 150, 70), -1)
        cv2.addWeighted(overlay, 0.82, image, 0.18, 0, image)
        text = 'OK'
        font_scale = 5.4 * scale
        thickness = max(int(round(10 * scale)), 2)
        size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        origin = ((width - size[0]) // 2, (height + size[1]) // 2)
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, font_scale,
            (0, 0, 0), thickness + max(int(round(5 * scale)), 2), cv2.LINE_AA)
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, font_scale,
            (255, 255, 255), thickness, cv2.LINE_AA)

    def _draw_wrapped(self, image, text, origin, max_width, scale, color):
        words = str(text).split()
        if not words:
            return origin[1]
        x, y = origin
        line = ''
        for word in words:
            candidate = word if not line else f'{line} {word}'
            size, _ = cv2.getTextSize(candidate, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
            if size[0] > max_width and line:
                self._put_text(image, line, (x, y), scale, color, 1)
                y += int(28 * scale) + 12
                line = word
            else:
                line = candidate
        if line:
            self._put_text(image, line, (x, y), scale, color, 1)
            y += int(28 * scale) + 12
        return y

    def _put_text(self, image, text, origin, scale, color, thickness):
        cv2.putText(
            image, str(text), origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(
            image, str(text), origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            color, thickness, cv2.LINE_AA)

    def _init_gui(self):
        if not self.gui_enabled:
            return
        if not os.environ.get('DISPLAY'):
            self.get_logger().warn('DISPLAY is not set; mission control GUI disabled')
            return
        try:
            import tkinter as tk
            from tkinter import messagebox
            root = tk.Tk()
        except Exception as exc:
            self.get_logger().warn(f'mission control GUI disabled: {exc}')
            return

        self.tk = tk
        self.messagebox = messagebox
        self.root = root
        root.title('Mission Control')
        root.geometry('420x660+80+80')
        root.configure(bg='#202226')
        root.protocol('WM_DELETE_WINDOW', root.iconify)
        root.bind_all('<KeyPress>', self._on_key_press)
        root.bind_all('<KeyRelease>', self._on_key_release)
        root.bind('<FocusOut>', self._on_focus_out)

        title = tk.Label(
            root,
            text='Mission Control',
            bg='#202226',
            fg='#f2f5f7',
            font=('Helvetica', 18, 'bold'),
            anchor='w',
        )
        title.pack(fill='x', padx=16, pady=(14, 4))

        active = tk.Label(
            root,
            text='',
            bg='#202226',
            fg='#f2f5f7',
            font=('Helvetica', 14, 'bold'),
            anchor='w',
        )
        active.pack(fill='x', padx=16, pady=(0, 10))
        self.gui_widgets['active'] = active

        button_frame = tk.Frame(root, bg='#202226')
        button_frame.pack(fill='x', padx=14, pady=4)
        self.gui_widgets['buttons'] = {}
        for mission_id in ('A', 'B', 'C', 'D'):
            button = tk.Button(
                button_frame,
                text=mission_id,
                width=5,
                height=2,
                command=lambda key=mission_id: self._set_mission(key, 'gui'),
                font=('Helvetica', 14, 'bold'),
            )
            button.pack(side='left', padx=4)
            self.gui_widgets['buttons'][mission_id] = button

        summary = tk.Label(
            root,
            text='',
            bg='#202226',
            fg='#d7dde2',
            justify='left',
            wraplength=340,
            anchor='nw',
            font=('Helvetica', 11),
        )
        summary.pack(fill='both', expand=True, padx=16, pady=(12, 4))
        self.gui_widgets['summary'] = summary

        layout_title = tk.Label(
            root,
            text='Window Layout',
            bg='#202226',
            fg='#d7dde2',
            font=('Helvetica', 11, 'bold'),
            anchor='w',
        )
        layout_title.pack(fill='x', padx=16, pady=(6, 2))

        layout_frame = tk.Frame(root, bg='#202226')
        layout_frame.pack(fill='x', padx=14, pady=(0, 4))
        layout_buttons = [
            ('Save', lambda: self._publish_layout_command('save')),
            ('Restore', lambda: self._publish_layout_command('restore')),
            ('Reset', self._confirm_reset_layout),
        ]
        for text, command in layout_buttons:
            button = tk.Button(
                layout_frame,
                text=text,
                width=10,
                height=1,
                command=command,
                font=('Helvetica', 10, 'bold'),
            )
            button.pack(side='left', padx=4)

        layout_status = tk.Label(
            root,
            text='Layout: ready',
            bg='#202226',
            fg='#9ca6af',
            font=('Helvetica', 10),
            anchor='w',
            wraplength=380,
        )
        layout_status.pack(fill='x', padx=16, pady=(0, 8))
        self.gui_widgets['layout_status'] = layout_status

        if self.keyboard_drive_ui_enabled:
            drive_title = tk.Label(
                root,
                text='Keyboard Drive',
                bg='#202226',
                fg='#d7dde2',
                font=('Helvetica', 11, 'bold'),
                anchor='w',
            )
            drive_title.pack(fill='x', padx=16, pady=(6, 2))

            drive_frame = tk.Frame(root, bg='#202226')
            drive_frame.pack(fill='x', padx=14, pady=(0, 4))
            self.gui_widgets['keyboard_enabled_var'] = tk.BooleanVar(value=False)
            drive_enable = tk.Checkbutton(
                drive_frame,
                text='Enable',
                variable=self.gui_widgets['keyboard_enabled_var'],
                command=lambda: self._set_keyboard_drive_enabled(
                    bool(self.gui_widgets['keyboard_enabled_var'].get()), 'gui'),
                bg='#202226',
                fg='#f2f5f7',
                selectcolor='#30343a',
                activebackground='#202226',
                activeforeground='#ffffff',
                font=('Helvetica', 10, 'bold'),
            )
            drive_enable.pack(side='left', padx=4)
            stop_button = tk.Button(
                drive_frame,
                text='STOP',
                width=8,
                height=1,
                command=self._stop_keyboard_drive,
                font=('Helvetica', 10, 'bold'),
                bg='#6b2f2f',
                fg='#ffffff',
            )
            stop_button.pack(side='left', padx=4)
        else:
            drive_frame = tk.Frame(root, bg='#202226')
            drive_frame.pack(fill='x', padx=14, pady=(6, 4))
        ok_button = tk.Button(
            drive_frame,
            text='OK',
            width=8,
            height=1,
            command=lambda: self._send_ok('button'),
            font=('Helvetica', 10, 'bold'),
            bg='#2f7f49',
            fg='#ffffff',
        )
        ok_button.pack(side='left', padx=4)

        if self.keyboard_drive_ui_enabled:
            drive_status = tk.Label(
                root,
                text='Drive: disabled',
                bg='#202226',
                fg='#9ca6af',
                font=('Helvetica', 10),
                anchor='w',
                wraplength=380,
            )
            drive_status.pack(fill='x', padx=16, pady=(0, 8))
            self.gui_widgets['drive_status'] = drive_status

        hint = tk.Label(
            root,
            text=(
                'Keys: A/B/C/D mission, O ok'
                if not self.keyboard_drive_ui_enabled
                else 'Keys: A/B/C/D mission, arrows drive, Shift+Left/Right rotate, Space stop, O ok'
            ),
            bg='#202226',
            fg='#9ca6af',
            font=('Helvetica', 10),
            anchor='w',
        )
        hint.pack(fill='x', padx=16, pady=(0, 12))
        self._update_gui()

    def _on_key_press(self, event):
        keysym = str(getattr(event, 'keysym', '') or '')
        if self.keyboard_drive_ui_enabled and keysym == 'space':
            self._stop_keyboard_drive()
            return
        if self.keyboard_drive_ui_enabled and keysym in ('Up', 'Down', 'Left', 'Right'):
            if self.keyboard_drive_enabled:
                drive_key = self._drive_key_from_event(event)
                if drive_key:
                    self.pressed_drive_keys[drive_key] = time.time()
                    self._update_drive_status()
            return

        char = str(getattr(event, 'char', '') or '')
        if char.lower() == 'o':
            self._send_ok('keyboard')
            return

        mission = self._normalize_mission(getattr(event, 'char', ''))
        if mission is not None:
            self._set_mission(mission, 'keyboard')

    def _on_key_release(self, event):
        if not self.keyboard_drive_ui_enabled:
            return
        keysym = str(getattr(event, 'keysym', '') or '')
        if keysym in ('Up', 'Down'):
            self.pressed_drive_keys.pop(keysym.lower(), None)
        elif keysym in ('Left', 'Right'):
            self.pressed_drive_keys.pop(keysym.lower(), None)
            self.pressed_drive_keys.pop(f'shift_{keysym.lower()}', None)
        elif keysym in ('Shift_L', 'Shift_R'):
            self.pressed_drive_keys.pop('shift_left', None)
            self.pressed_drive_keys.pop('shift_right', None)
        self._update_drive_status()

    def _on_focus_out(self, event):
        del event
        if not self.keyboard_drive_ui_enabled:
            return
        self.pressed_drive_keys.clear()
        self._update_drive_status()

    def _drive_key_from_event(self, event):
        keysym = str(getattr(event, 'keysym', '') or '')
        shift_pressed = bool(int(getattr(event, 'state', 0) or 0) & 0x0001)
        if keysym in ('Left', 'Right') and shift_pressed:
            return f'shift_{keysym.lower()}'
        if keysym in ('Up', 'Down', 'Left', 'Right'):
            return keysym.lower()
        return ''

    def _active_drive_keys(self):
        if not self.pressed_drive_keys:
            return set()
        now_s = time.time()
        active = {
            key for key, stamp_s in self.pressed_drive_keys.items()
            if now_s - stamp_s <= self.keyboard_key_timeout_s
        }
        if len(active) != len(self.pressed_drive_keys):
            self.pressed_drive_keys = {key: self.pressed_drive_keys[key] for key in active}
        return active

    def _publish_keyboard_enabled(self):
        if self.keyboard_enabled_pub is None:
            return
        msg = Bool()
        msg.data = bool(self.keyboard_drive_enabled)
        self.keyboard_enabled_pub.publish(msg)

    def _set_keyboard_drive_enabled(self, enabled, source):
        if not self.keyboard_drive_ui_enabled:
            return
        enabled = bool(enabled)
        if self.keyboard_drive_enabled == enabled:
            self._publish_keyboard_enabled()
            self._update_drive_status()
            return
        self.keyboard_drive_enabled = enabled
        if not enabled:
            self.pressed_drive_keys.clear()
            if self.keyboard_cmd_pub is not None:
                self.keyboard_cmd_pub.publish(Twist())
        self._publish_keyboard_enabled()
        self._update_drive_status()
        self.get_logger().info(f'keyboard drive {"enabled" if enabled else "disabled"} by {source}')

    def _stop_keyboard_drive(self):
        self.pressed_drive_keys.clear()
        if self.keyboard_cmd_pub is not None:
            self.keyboard_cmd_pub.publish(Twist())
        self._update_drive_status()

    def _make_keyboard_twist(self):
        twist = Twist()
        if not self.keyboard_drive_enabled:
            return twist
        active_keys = self._active_drive_keys()
        if 'up' in active_keys and 'down' not in active_keys:
            twist.linear.x = self.keyboard_linear_x_mps
        elif 'down' in active_keys and 'up' not in active_keys:
            twist.linear.x = -self.keyboard_linear_x_mps
        if 'left' in active_keys and 'right' not in active_keys:
            twist.linear.y = self.keyboard_linear_y_mps
        elif 'right' in active_keys and 'left' not in active_keys:
            twist.linear.y = -self.keyboard_linear_y_mps
        if (
            'shift_left' in active_keys and
            'shift_right' not in active_keys
        ):
            twist.angular.z = self.keyboard_angular_z_radps
        elif (
            'shift_right' in active_keys and
            'shift_left' not in active_keys
        ):
            twist.angular.z = -self.keyboard_angular_z_radps
        return twist

    def _publish_keyboard_command(self):
        if not self.keyboard_drive_ui_enabled or self.keyboard_cmd_pub is None:
            return
        if not self.keyboard_drive_enabled:
            return
        self.keyboard_cmd_pub.publish(self._make_keyboard_twist())
        self._publish_keyboard_enabled()

    def _send_ok(self, source):
        now_s = time.time()
        self.ok_overlay_until_s = now_s + self.ok_overlay_duration_s
        payload = {
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'event': 'ok',
            'mission_id': self.active_mission,
            'source': str(source),
            'duration_s': self.ok_overlay_duration_s,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.operator_ok_pub.publish(msg)
        self._publish_state()

    def _update_drive_status(self):
        label = self.gui_widgets.get('drive_status')
        if label is None:
            return
        if not self.keyboard_drive_enabled:
            label.configure(text='Drive: disabled')
            return
        twist = self._make_keyboard_twist()
        label.configure(
            text=(
                'Drive: enabled  '
                f'x={twist.linear.x:+.2f} y={twist.linear.y:+.2f} '
                f'wz={twist.angular.z:+.2f}'
            )
        )

    def _publish_layout_command(self, command):
        command = str(command or '').strip().lower()
        if not command:
            return
        if self.layout_command_pub is None:
            self._set_layout_status('Layout: command topic disabled')
            self.get_logger().warn('layout command topic is disabled')
            return
        msg = String()
        msg.data = command
        self.layout_command_pub.publish(msg)
        self._set_layout_status(f'Layout: {command} requested')
        self.get_logger().info(f'layout command requested: {command}')

    def _confirm_reset_layout(self):
        if self.messagebox is not None:
            confirmed = self.messagebox.askyesno(
                'Reset Layout',
                'Delete the saved operator window layout?',
            )
            if not confirmed:
                return
        self._publish_layout_command('reset')

    def _set_layout_status(self, text):
        label = self.gui_widgets.get('layout_status')
        if label is not None:
            label.configure(text=str(text))

    def _update_gui(self):
        if self.root is None:
            return
        profile = self.missions.get(self.active_mission, {})
        active = self.gui_widgets.get('active')
        if active is not None:
            active.configure(text=f'Active: {self.active_mission}  {profile.get("title", "")}')
        summary = self.gui_widgets.get('summary')
        if summary is not None:
            features = '\n'.join(
                f'- {item}' for item in profile.get('specific_features') or [])
            summary.configure(text=f'{profile.get("summary", "")}\n\n{features}')
        for mission_id, button in (self.gui_widgets.get('buttons') or {}).items():
            if mission_id == self.active_mission:
                button.configure(
                    bg=MISSION_COLORS_TK.get(mission_id, '#555555'),
                    fg='#ffffff',
                    relief='sunken',
                    activebackground=MISSION_COLORS_TK.get(mission_id, '#555555'),
                )
            else:
                button.configure(
                    bg='#3a3d43',
                    fg='#ffffff',
                    relief='raised',
                    activebackground='#555a62',
                )
        keyboard_enabled_var = self.gui_widgets.get('keyboard_enabled_var')
        if keyboard_enabled_var is not None:
            keyboard_enabled_var.set(bool(self.keyboard_drive_enabled))
        self._update_drive_status()

    def _schedule_ros_spin(self):
        if self.root is None:
            return
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.root.after(20, self._schedule_ros_spin)
        else:
            self.root.quit()


def main(args=None):
    rclpy.init(args=args)
    node = MissionModeManager()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
