import json
import os
import re
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import String


DEFAULT_STREAMS = [
    'STATUS|/teleop/operator_status/compressed',
    'BANDWIDTH|/teleop/bandwidth_monitor/compressed',
    'ZED|/teleop/zed/depth_assist/compressed',
    'R WRIST|/teleop/wrist_right/depth_assist/compressed',
    'R COLOR|raw:/camera_right/camera_right/color/image_rect_raw',
]

DEFAULT_MISSING_IMAGE_HINTS = [
    'R COLOR|no raw right color stream - check RealSense color/profile/USB',
]
DEFAULT_STREAM_STATS_STREAMS = [
    'R COLOR|wrist_right_color',
]
DEFAULT_STREAM_ROTATE_DEG = [
    'R COLOR|90',
]

TOOLBAR_HEIGHT = 42
RESIZE_HANDLE_PX = 18
MIN_TILE_WIDTH = 180
MIN_TILE_HEIGHT = 120
MIN_CANVAS_WIDTH = 960
MIN_CANVAS_HEIGHT = 540
DEFAULT_AUTO_CANVAS_WIDTH = 1920
DEFAULT_AUTO_CANVAS_HEIGHT = 1080


class OperatorImageViewer(Node):

    def __init__(self):
        super().__init__('operator_image_viewer')

        self.declare_parameter('window_title', 'Teleop Image Viewer')
        self.declare_parameter('streams', DEFAULT_STREAMS)
        self.declare_parameter('tile_width', 640)
        self.declare_parameter('tile_height', 360)
        self.declare_parameter('columns', 2)
        self.declare_parameter('canvas_width', 0)
        self.declare_parameter('canvas_height', 0)
        self.declare_parameter('auto_canvas_size', True)
        self.declare_parameter('fill_grid_on_reset', True)
        self.declare_parameter('follow_window_size', True)
        self.declare_parameter(
            'layout_store_path',
            '~/.config/ffw_teleop/operator_image_viewer_layout.json')
        self.declare_parameter('show_toolbar', True)
        self.declare_parameter('show_hz', 20.0)
        self.declare_parameter('max_stale_sec', 0.75)
        self.declare_parameter('missing_image_hints', DEFAULT_MISSING_IMAGE_HINTS)
        self.declare_parameter('stream_stats_topic', '/teleop/stream_stats')
        self.declare_parameter('stream_stats_streams', DEFAULT_STREAM_STATS_STREAMS)
        self.declare_parameter('stream_rotate_deg', DEFAULT_STREAM_ROTATE_DEG)
        self.declare_parameter('camera_perf_topic', '/teleop/camera_perf')
        self.declare_parameter('window_x', 1520)
        self.declare_parameter('window_y', 40)
        self.declare_parameter('headless_ok', True)

        self.window_title = str(self.get_parameter('window_title').value).strip()
        if not self.window_title:
            self.window_title = 'Teleop Image Viewer'
        self.tile_width = max(int(self.get_parameter('tile_width').value), 160)
        self.tile_height = max(int(self.get_parameter('tile_height').value), 120)
        self.columns = max(int(self.get_parameter('columns').value), 1)
        self.layout_store_path = os.path.expanduser(
            str(self.get_parameter('layout_store_path').value).strip())
        self.show_toolbar = self._as_bool(self.get_parameter('show_toolbar').value)
        self.toolbar_height = TOOLBAR_HEIGHT if self.show_toolbar else 0
        show_hz = max(float(self.get_parameter('show_hz').value), 1.0)
        self.max_stale_sec = max(float(self.get_parameter('max_stale_sec').value), 0.2)
        self.window_x = int(self.get_parameter('window_x').value)
        self.window_y = int(self.get_parameter('window_y').value)
        self.headless_ok = self._as_bool(self.get_parameter('headless_ok').value)
        self.auto_canvas_size = self._as_bool(self.get_parameter('auto_canvas_size').value)
        self.fill_grid_on_reset = self._as_bool(
            self.get_parameter('fill_grid_on_reset').value)
        self.follow_window_size = self._as_bool(
            self.get_parameter('follow_window_size').value)
        requested_canvas_width = int(self.get_parameter('canvas_width').value)
        requested_canvas_height = int(self.get_parameter('canvas_height').value)
        self.canvas_width, self.canvas_height = self._resolve_canvas_size(
            requested_canvas_width, requested_canvas_height)

        self.streams = self._parse_streams(self.get_parameter('streams').value)
        self.missing_image_hints = self._parse_hints(
            self.get_parameter('missing_image_hints').value)
        self.stream_stats_topic = str(
            self.get_parameter('stream_stats_topic').value).strip()
        self.stream_stats_names = self._parse_hints(
            self.get_parameter('stream_stats_streams').value)
        self.stream_rotate_deg = self._parse_float_hints(
            self.get_parameter('stream_rotate_deg').value)
        self.camera_perf_topic = str(self.get_parameter('camera_perf_topic').value).strip()
        self.stream_by_name = {stream['name']: stream for stream in self.streams}
        self.frames = {
            stream['name']: {
                'image': None,
                'stamp': 0.0,
                'topic': stream['topic'],
                'transport': stream.get('transport', 'compressed'),
            }
            for stream in self.streams
        }
        self.layout_by_name = self._default_layout()
        self.z_order = [stream['name'] for stream in self.streams]
        self.selected_name = ''
        self.drag_mode = ''
        self.drag_start = None
        self.gui_available = False
        self._image_subscriptions = []
        self._last_window_sync_s = 0.0
        self.stream_stats_pub = None
        if self.stream_stats_topic and self.stream_stats_names:
            self.stream_stats_pub = self.create_publisher(String, self.stream_stats_topic, 10)
        self.camera_perf_pub = None
        if self.camera_perf_topic and self.stream_stats_names:
            self.camera_perf_pub = self.create_publisher(String, self.camera_perf_topic, 10)
        self.camera_perf_times = {}
        self.last_camera_perf_publish = {}

        self._load_layout()

        qos = QoSProfile(depth=2)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        for stream in self.streams:
            transport = stream.get('transport', 'compressed')
            msg_type = Image if transport == 'raw' else CompressedImage
            callback = (
                self._make_raw_callback(stream['name'])
                if transport == 'raw'
                else self._make_compressed_callback(stream['name'])
            )
            self._image_subscriptions.append(
                self.create_subscription(
                    msg_type,
                    stream['topic'],
                    callback,
                    qos,
                )
            )

        self._init_window()
        self.timer = self.create_timer(1.0 / show_hz, self._show)
        self.get_logger().info(
            f'operator image viewer active: window={self.window_title!r}, '
            f'streams={len(self.streams)}, canvas={self.canvas_width}x{self.canvas_height}, '
            f'layout={self.layout_store_path or "disabled"}, gui={self.gui_available}, '
            f'color_stats={self.stream_stats_topic if self.stream_stats_pub else "disabled"}')

    def destroy_node(self):
        if self.gui_available:
            try:
                cv2.destroyWindow(self.window_title)
            except cv2.error:
                pass
        super().destroy_node()

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _resolve_canvas_size(self, requested_width, requested_height):
        if not self.auto_canvas_size and requested_width > 0 and requested_height > 0:
            return (
                max(int(requested_width), MIN_CANVAS_WIDTH),
                max(int(requested_height), MIN_CANVAS_HEIGHT),
            )

        screen_width, screen_height = self._detect_screen_size()
        if screen_width <= 0 or screen_height <= 0:
            screen_width = DEFAULT_AUTO_CANVAS_WIDTH
            screen_height = DEFAULT_AUTO_CANVAS_HEIGHT

        width = int(requested_width) if requested_width > 0 else screen_width - 96
        height = (
            int(requested_height)
            if requested_height > 0
            else screen_height - self.toolbar_height - 120
        )
        width = max(width, MIN_CANVAS_WIDTH)
        height = max(height, MIN_CANVAS_HEIGHT)
        return width, height

    def _detect_screen_size(self):
        size = self._detect_screen_size_tk()
        if size:
            return size
        size = self._detect_screen_size_xdpyinfo()
        if size:
            return size
        return 0, 0

    def _detect_screen_size_tk(self):
        if not os.environ.get('DISPLAY'):
            return None
        try:
            import tkinter as tk
            root = tk.Tk()
            root.withdraw()
            width = int(root.winfo_screenwidth())
            height = int(root.winfo_screenheight())
            root.destroy()
            if width > 0 and height > 0:
                return width, height
        except Exception:
            return None
        return None

    def _detect_screen_size_xdpyinfo(self):
        if not os.environ.get('DISPLAY'):
            return None
        try:
            result = subprocess.run(
                ['xdpyinfo'],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.0,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        match = re.search(r'dimensions:\s+(\d+)x(\d+)\s+pixels', result.stdout or '')
        if not match:
            return None
        width = int(match.group(1))
        height = int(match.group(2))
        if width <= 0 or height <= 0:
            return None
        return width, height

    def _parse_streams(self, raw_streams):
        streams = []
        for raw in list(raw_streams or []):
            text = str(raw).strip()
            if not text:
                continue
            if '|' in text:
                name, topic = text.split('|', 1)
            else:
                topic = text
                display_topic = self._strip_transport_prefix(topic.strip())[1]
                name = display_topic.rsplit('/', 2)[0].rsplit('/', 1)[-1].upper()
            name = name.strip() or topic
            topic = topic.strip()
            transport, topic = self._strip_transport_prefix(topic)
            if topic:
                streams.append({'name': name, 'topic': topic, 'transport': transport})
        if streams:
            return streams
        return self._parse_streams(DEFAULT_STREAMS)

    def _strip_transport_prefix(self, topic):
        topic = str(topic or '').strip()
        lower = topic.lower()
        if lower.startswith('raw:'):
            return 'raw', topic[4:].strip()
        if lower.startswith('compressed:'):
            return 'compressed', topic.split(':', 1)[1].strip()
        return 'compressed', topic

    def _parse_hints(self, raw_hints):
        hints = {}
        for raw in list(raw_hints or []):
            text = str(raw).strip()
            if not text or '|' not in text:
                continue
            name, hint = text.split('|', 1)
            name = name.strip()
            hint = hint.strip()
            if name and hint:
                hints[name] = hint
        return hints

    def _parse_float_hints(self, raw_hints):
        hints = {}
        for raw in list(raw_hints or []):
            text = str(raw).strip()
            if not text or '|' not in text:
                continue
            name, value = text.split('|', 1)
            name = name.strip()
            try:
                number = float(str(value).strip())
            except (TypeError, ValueError):
                continue
            if name:
                hints[name] = number
        return hints

    def _default_layout(self):
        layout = {}
        row_count = max((len(self.streams) + self.columns - 1) // self.columns, 1)
        if self.fill_grid_on_reset:
            base_width = max(self.canvas_width // self.columns, MIN_TILE_WIDTH)
            base_height = max(self.canvas_height // row_count, MIN_TILE_HEIGHT)
        else:
            base_width = self.tile_width
            base_height = self.tile_height
        for index, stream in enumerate(self.streams):
            column = index % self.columns
            row = index // self.columns
            x = column * base_width
            y = row * base_height
            width = (
                self.canvas_width - x
                if self.fill_grid_on_reset and column == self.columns - 1
                else base_width
            )
            height = (
                self.canvas_height - y
                if self.fill_grid_on_reset and row == row_count - 1
                else base_height
            )
            rect = {
                'x': x,
                'y': y,
                'width': width,
                'height': height,
            }
            layout[stream['name']] = self._sanitize_rect(rect)
        return layout

    def _load_layout(self):
        if not self.layout_store_path or not os.path.exists(self.layout_store_path):
            return
        try:
            with open(self.layout_store_path, 'r', encoding='utf-8') as stream:
                payload = json.load(stream)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'failed to load image viewer layout: {exc}')
            return

        stored_layout = payload.get('layout') if isinstance(payload, dict) else None
        if not isinstance(stored_layout, dict):
            return
        stored_canvas_width = self._positive_int(payload.get('canvas_width'))
        stored_canvas_height = self._positive_int(payload.get('canvas_height'))
        for name, rect in stored_layout.items():
            if name in self.layout_by_name and isinstance(rect, dict):
                self.layout_by_name[name] = self._sanitize_rect(
                    self._scale_rect_from_saved(rect, stored_canvas_width, stored_canvas_height))

        stored_order = payload.get('z_order', [])
        if isinstance(stored_order, list):
            ordered = [str(name) for name in stored_order if str(name) in self.layout_by_name]
            ordered.extend(name for name in self.layout_by_name.keys() if name not in ordered)
            self.z_order = ordered
        self.get_logger().info(f'loaded image viewer layout: {self.layout_store_path}')

    def _positive_int(self, value):
        try:
            number = int(value)
        except (TypeError, ValueError):
            return 0
        return number if number > 0 else 0

    def _scale_rect_from_saved(self, rect, stored_canvas_width, stored_canvas_height):
        if stored_canvas_width <= 0 or stored_canvas_height <= 0:
            return rect
        scale_x = self.canvas_width / float(stored_canvas_width)
        scale_y = self.canvas_height / float(stored_canvas_height)
        if abs(scale_x - 1.0) < 0.01 and abs(scale_y - 1.0) < 0.01:
            return rect
        return {
            'x': int(round(float(rect.get('x', 0)) * scale_x)),
            'y': int(round(float(rect.get('y', 0)) * scale_y)),
            'width': int(round(float(rect.get('width', self.tile_width)) * scale_x)),
            'height': int(round(float(rect.get('height', self.tile_height)) * scale_y)),
        }

    def _save_layout(self):
        if not self.layout_store_path:
            self.get_logger().warn('image viewer layout save skipped: layout_store_path is empty')
            return
        payload = {
            'version': 1,
            'canvas_width': self.canvas_width,
            'canvas_height': self.canvas_height,
            'layout': self.layout_by_name,
            'z_order': self.z_order,
        }
        try:
            parent = os.path.dirname(self.layout_store_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            tmp_path = f'{self.layout_store_path}.tmp'
            with open(tmp_path, 'w', encoding='utf-8') as stream:
                json.dump(payload, stream, indent=2, sort_keys=True)
                stream.write('\n')
            os.replace(tmp_path, self.layout_store_path)
            self.get_logger().info(f'saved image viewer layout: {self.layout_store_path}')
        except OSError as exc:
            self.get_logger().warn(f'failed to save image viewer layout: {exc}')

    def _reset_layout(self, delete_saved):
        self.layout_by_name = self._default_layout()
        self.z_order = [stream['name'] for stream in self.streams]
        self.selected_name = ''
        self.drag_mode = ''
        self.drag_start = None
        if delete_saved and self.layout_store_path and os.path.exists(self.layout_store_path):
            try:
                os.remove(self.layout_store_path)
                self.get_logger().info(f'removed saved image viewer layout: {self.layout_store_path}')
            except OSError as exc:
                self.get_logger().warn(f'failed to remove saved image viewer layout: {exc}')

    def _sanitize_rect(self, rect):
        width = int(rect.get('width', self.tile_width))
        height = int(rect.get('height', self.tile_height))
        width = max(min(width, self.canvas_width), min(MIN_TILE_WIDTH, self.canvas_width))
        height = max(min(height, self.canvas_height), min(MIN_TILE_HEIGHT, self.canvas_height))
        max_x = max(self.canvas_width - width, 0)
        max_y = max(self.canvas_height - height, 0)
        x = int(np.clip(int(rect.get('x', 0)), 0, max_x))
        y = int(np.clip(int(rect.get('y', 0)), 0, max_y))
        return {'x': x, 'y': y, 'width': width, 'height': height}

    def _init_window(self):
        if not os.environ.get('DISPLAY') and self.headless_ok:
            self.get_logger().warn('DISPLAY is not set; image viewer running headless')
            return
        try:
            cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                self.window_title,
                self.canvas_width,
                self.canvas_height + self.toolbar_height,
            )
            cv2.setMouseCallback(self.window_title, self._mouse_callback)
            if self.window_x >= 0 and self.window_y >= 0:
                cv2.moveWindow(self.window_title, self.window_x, self.window_y)
            self.gui_available = True
        except cv2.error as exc:
            self.get_logger().error(f'failed to create OpenCV viewer window: {exc}')
            self.gui_available = False

    def _make_compressed_callback(self, name):
        def callback(msg):
            data = np.frombuffer(msg.data, dtype=np.uint8)
            image = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if image is None:
                self.get_logger().warn(f'failed to decode compressed image for {name}')
                return
            image = self._rotate_stream_image(name, image)
            self.frames[name]['image'] = image
            self.frames[name]['stamp'] = time.time()
            self._publish_stream_stats(name, msg, image)
            self._publish_camera_perf(name, msg, image)
        return callback

    def _make_raw_callback(self, name):
        def callback(msg):
            image = self._image_msg_to_bgr(msg)
            if image is None:
                self.get_logger().warn(
                    f'failed to convert raw image for {name}: encoding={msg.encoding}')
                return
            image = self._rotate_stream_image(name, image)
            self.frames[name]['image'] = image
            self.frames[name]['stamp'] = time.time()
            self._publish_stream_stats(name, msg, image)
            self._publish_camera_perf(name, msg, image)
        return callback

    def _image_msg_to_bgr(self, msg):
        encoding = str(msg.encoding or '').upper()
        if msg.step <= 0 or msg.height <= 0 or msg.width <= 0:
            return None

        channels_by_encoding = {
            'BGR8': 3,
            'RGB8': 3,
            'BGRA8': 4,
            'RGBA8': 4,
            'MONO8': 1,
            '8UC1': 1,
            'YUYV': 2,
            'YUY2': 2,
            'UYVY': 2,
        }
        channels = channels_by_encoding.get(encoding)
        if channels is not None:
            row_bytes = int(msg.width) * channels
            if msg.step < row_bytes:
                return None
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            expected = int(msg.step) * int(msg.height)
            if raw.size < expected:
                return None
            rows = raw[:expected].reshape((msg.height, msg.step))
            image = rows[:, :row_bytes].reshape((msg.height, msg.width, channels))
            if encoding == 'BGR8':
                return np.ascontiguousarray(image)
            if encoding == 'RGB8':
                return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if encoding == 'BGRA8':
                return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            if encoding == 'RGBA8':
                return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
            if encoding in ('MONO8', '8UC1'):
                return cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2BGR)
            if encoding in ('YUYV', 'YUY2'):
                return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)
            if encoding == 'UYVY':
                return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)

        if encoding in ('16UC1', 'MONO16'):
            dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
            stride = msg.step // dtype.itemsize
            if stride < msg.width:
                return None
            expected = stride * int(msg.height)
            raw = np.frombuffer(msg.data, dtype=dtype)
            if raw.size < expected:
                return None
            image16 = raw[:expected].reshape((msg.height, stride))[:, :msg.width]
            if image16.size == 0:
                return None
            max_value = float(np.nanmax(image16))
            scale = 255.0 / max(max_value, 1.0)
            image8 = cv2.convertScaleAbs(image16, alpha=scale)
            return cv2.cvtColor(image8, cv2.COLOR_GRAY2BGR)
        return None

    def _rotate_stream_image(self, name, image):
        angle = float(self.stream_rotate_deg.get(name, 0.0)) % 360.0
        if abs(angle) < 1e-3:
            return image
        height, width = image.shape[:2]
        if height <= 0 or width <= 0:
            return image
        center = (width / 2.0, height / 2.0)
        matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        cos = abs(matrix[0, 0])
        sin = abs(matrix[0, 1])
        new_width = int((height * sin) + (width * cos))
        new_height = int((height * cos) + (width * sin))
        matrix[0, 2] += (new_width / 2.0) - center[0]
        matrix[1, 2] += (new_height / 2.0) - center[1]
        return cv2.warpAffine(
            image,
            matrix,
            (new_width, new_height),
            flags=cv2.INTER_LINEAR,
            borderValue=(0, 0, 0),
        )

    def _publish_stream_stats(self, name, msg, image):
        if self.stream_stats_pub is None:
            return
        stats_name = self.stream_stats_names.get(name)
        if not stats_name:
            return
        height, width = image.shape[:2]
        payload = {
            'stamp_sec': time.time(),
            'name': stats_name,
            'topic': self.frames.get(name, {}).get('topic', ''),
            'bytes': self._message_byte_count(msg),
            'width': int(width),
            'height': int(height),
            'jpeg_quality': None,
        }
        out = String()
        out.data = json.dumps(payload, sort_keys=True)
        self.stream_stats_pub.publish(out)

    def _publish_camera_perf(self, name, msg, image):
        if self.camera_perf_pub is None:
            return
        stats_name = self.stream_stats_names.get(name)
        if not stats_name:
            return
        now_s = time.time()
        times = self.camera_perf_times.setdefault(name, [])
        times.append(now_s)
        cutoff = now_s - 5.0
        del times[:max(0, len(times) - 200)]
        while times and times[0] < cutoff:
            times.pop(0)
        last_publish = self.last_camera_perf_publish.get(name, 0.0)
        if now_s - last_publish < 1.0:
            return
        self.last_camera_perf_publish[name] = now_s
        height, width = image.shape[:2]
        payload = {
            'stamp_sec': now_s,
            'name': stats_name,
            'topic': self.frames.get(name, {}).get('topic', ''),
            'rx_hz': self._sample_hz(times),
            'output_hz': self._sample_hz(times),
            'bytes': self._message_byte_count(msg),
            'width': int(width),
            'height': int(height),
            'subscriber_count': 1,
            'drop_reason': 'main_rx',
        }
        out = String()
        out.data = json.dumps(payload, sort_keys=True)
        self.camera_perf_pub.publish(out)

    def _message_byte_count(self, msg):
        return len(getattr(msg, 'data', b'') or b'')

    def _sample_hz(self, samples):
        if len(samples) < 2:
            return 0.0
        duration = max(float(samples[-1] - samples[0]), 1e-6)
        return (len(samples) - 1) / duration

    def _show(self):
        if not self.gui_available:
            return
        self._sync_canvas_to_window_size()
        board = self._compose_board()
        try:
            cv2.imshow(self.window_title, board)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as exc:
            self.get_logger().error(f'OpenCV viewer failed; disabling GUI: {exc}')
            self.gui_available = False
            return
        if key in (27, ord('q'), ord('Q')):
            self.gui_available = False
            cv2.destroyWindow(self.window_title)
        elif key in (ord('s'), ord('S')):
            self._save_layout()
        elif key in (ord('r'), ord('R')):
            self._reset_layout(delete_saved=True)
        elif key in (ord('g'), ord('G')):
            self._reset_layout(delete_saved=False)

    def _compose_board(self):
        board = np.full(
            (self.canvas_height + self.toolbar_height, self.canvas_width, 3),
            (22, 24, 28),
            dtype=np.uint8,
        )
        if self.show_toolbar:
            self._draw_toolbar(board)

        canvas = board[self.toolbar_height:self.toolbar_height + self.canvas_height, :]
        now = time.time()
        for name in self.z_order:
            stream = self.stream_by_name.get(name)
            rect = self.layout_by_name.get(name)
            if not stream or not rect:
                continue
            tile = self._make_tile(stream, now, rect['width'], rect['height'], name == self.selected_name)
            x0 = rect['x']
            y0 = rect['y']
            canvas[y0:y0 + rect['height'], x0:x0 + rect['width']] = tile
        return board

    def _sync_canvas_to_window_size(self):
        if not self.follow_window_size or not os.environ.get('DISPLAY'):
            return
        now = time.time()
        if now - self._last_window_sync_s < 0.5:
            return
        self._last_window_sync_s = now
        geometry = self._current_window_geometry()
        if not geometry:
            return
        _, _, width, height = geometry
        target_width = max(int(width), MIN_CANVAS_WIDTH)
        target_height = max(int(height) - self.toolbar_height, MIN_CANVAS_HEIGHT)
        if (
            abs(target_width - self.canvas_width) < 48 and
            abs(target_height - self.canvas_height) < 48
        ):
            return
        old_width = self.canvas_width
        old_height = self.canvas_height
        self.canvas_width = target_width
        self.canvas_height = target_height
        self._scale_layout_to_canvas(old_width, old_height)

    def _current_window_geometry(self):
        geometry = self._current_window_geometry_wmctrl()
        if geometry:
            return geometry
        return self._current_window_geometry_xwininfo()

    def _current_window_geometry_wmctrl(self):
        try:
            result = subprocess.run(
                ['wmctrl', '-lG'],
                check=False,
                capture_output=True,
                text=True,
                timeout=0.5,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        if result.returncode != 0:
            return None
        for line in (result.stdout or '').splitlines():
            parts = line.split(None, 7)
            if len(parts) < 8:
                continue
            title = parts[7]
            if self.window_title not in title:
                continue
            try:
                return (
                    int(parts[2]),
                    int(parts[3]),
                    int(parts[4]),
                    int(parts[5]),
                )
            except ValueError:
                continue
        return None

    def _current_window_geometry_xwininfo(self):
        try:
            result = subprocess.run(
                ['xwininfo', '-name', self.window_title],
                check=False,
                capture_output=True,
                text=True,
                timeout=0.5,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        if result.returncode != 0:
            return None
        fields = {}
        for line in (result.stdout or '').splitlines():
            if ':' not in line:
                continue
            key, value = line.split(':', 1)
            fields[key.strip()] = value.strip()
        try:
            return (
                int(fields['Absolute upper-left X']),
                int(fields['Absolute upper-left Y']),
                int(fields['Width']),
                int(fields['Height']),
            )
        except (KeyError, ValueError):
            return None

    def _scale_layout_to_canvas(self, old_width, old_height):
        if old_width <= 0 or old_height <= 0:
            self.layout_by_name = self._default_layout()
            return
        scale_x = self.canvas_width / float(old_width)
        scale_y = self.canvas_height / float(old_height)
        scaled = {}
        for name, rect in self.layout_by_name.items():
            scaled[name] = self._sanitize_rect({
                'x': int(round(rect.get('x', 0) * scale_x)),
                'y': int(round(rect.get('y', 0) * scale_y)),
                'width': int(round(rect.get('width', self.tile_width) * scale_x)),
                'height': int(round(rect.get('height', self.tile_height) * scale_y)),
            })
        self.layout_by_name = scaled
        self.drag_mode = ''
        self.drag_start = None

    def _draw_toolbar(self, board):
        cv2.rectangle(board, (0, 0), (self.canvas_width, self.toolbar_height), (14, 16, 20), -1)
        for action, label, rect in self._toolbar_buttons():
            x, y, width, height = rect
            color = (58, 64, 74) if action != 'reset' else (64, 54, 54)
            cv2.rectangle(board, (x, y), (x + width, y + height), color, -1)
            cv2.rectangle(board, (x, y), (x + width, y + height), (150, 158, 170), 1)
            self._put_text(board, label, (x + 12, y + 22), 0.48, (238, 240, 244), 1)
        hint = 'Drag tiles; drag bottom-right corner to resize; S/R/G shortcuts'
        self._put_text(board, hint, (296, 27), 0.42, (178, 186, 196), 1)

    def _toolbar_buttons(self):
        return [
            ('save', 'SAVE', (10, 7, 78, 28)),
            ('reset', 'RESET', (96, 7, 84, 28)),
            ('grid', 'GRID', (188, 7, 78, 28)),
        ]

    def _make_tile(self, stream, now, width, height, selected):
        name = stream['name']
        state = self.frames.get(name, {})
        image = state.get('image')
        stamp = float(state.get('stamp') or 0.0)
        stale = stamp <= 0.0 or (now - stamp) > self.max_stale_sec
        if image is None:
            tile = np.full((height, width, 3), (34, 36, 42), dtype=np.uint8)
            hint = self._missing_image_hint(name, stream)
            self._put_center(tile, hint, (166, 172, 180), 0.58)
        else:
            tile = self._fit_image(image, width, height)
            if stale:
                overlay = np.full_like(tile, (20, 20, 20))
                tile = cv2.addWeighted(tile, 0.45, overlay, 0.55, 0)
                self._put_center(tile, 'stale', (80, 180, 255), 0.74)

        border = (30, 210, 255) if selected else (74, 78, 86)
        cv2.rectangle(tile, (0, 0), (width - 1, height - 1), border, 2 if selected else 1)
        cv2.rectangle(tile, (0, 0), (width - 1, 30), (12, 14, 18), -1)
        self._put_text(tile, name, (10, 21), 0.56, (238, 240, 244), 1)
        age = '--' if stamp <= 0.0 else f'{now - stamp:.1f}s'
        self._put_text(tile, age, (max(width - 62, 48), 21), 0.46, (190, 196, 204), 1)
        self._draw_resize_handle(tile)
        return tile

    def _missing_image_hint(self, name, stream):
        topic = stream.get('topic') or ''
        publisher_count = self.count_publishers(topic) if topic else 0
        topic_text = self._short_topic(topic)
        if publisher_count <= 0:
            transport = stream.get('transport', 'compressed')
            source = 'raw' if transport == 'raw' else 'compressed'
            return f'no {source} publisher\npubs=0 {topic_text}'
        hint = self.missing_image_hints.get(name, 'waiting for image')
        return f'{hint}\npubs={publisher_count} {topic_text}'

    def _short_topic(self, topic, max_len=58):
        topic = str(topic or '').strip()
        if len(topic) <= max_len:
            return topic
        return '...' + topic[-max(max_len - 3, 1):]

    def _fit_image(self, image, target_width, target_height):
        src_h, src_w = image.shape[:2]
        if src_h <= 0 or src_w <= 0:
            return np.full((target_height, target_width, 3), (34, 36, 42), dtype=np.uint8)
        scale = min(target_width / float(src_w), target_height / float(src_h))
        new_w = max(int(src_w * scale), 1)
        new_h = max(int(src_h * scale), 1)
        interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        resized = cv2.resize(image, (new_w, new_h), interpolation=interpolation)
        tile = np.full((target_height, target_width, 3), (10, 10, 12), dtype=np.uint8)
        x0 = (target_width - new_w) // 2
        y0 = (target_height - new_h) // 2
        tile[y0:y0 + new_h, x0:x0 + new_w] = resized
        return tile

    def _draw_resize_handle(self, tile):
        height, width = tile.shape[:2]
        points = np.array([
            [width - RESIZE_HANDLE_PX, height - 2],
            [width - 2, height - RESIZE_HANDLE_PX],
            [width - 2, height - 2],
        ], dtype=np.int32)
        cv2.fillConvexPoly(tile, points, (90, 96, 108))
        cv2.polylines(tile, [points], True, (190, 198, 208), 1, cv2.LINE_AA)

    def _mouse_callback(self, event, x, y, flags, param):
        del flags, param
        if event == cv2.EVENT_LBUTTONDOWN:
            self._mouse_down(x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_mode:
            self._mouse_drag(x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drag_mode = ''
            self.drag_start = None

    def _mouse_down(self, x, y):
        if self.show_toolbar and y < self.toolbar_height:
            for action, _, rect in self._toolbar_buttons():
                if self._point_in_rect(x, y, rect):
                    self._run_toolbar_action(action)
                    return
            return

        point = self._canvas_point(x, y)
        if point is None:
            return
        canvas_x, canvas_y = point
        name = self._find_top_tile(canvas_x, canvas_y)
        if not name:
            self.selected_name = ''
            return

        self.selected_name = name
        self._bring_to_front(name)
        rect = self.layout_by_name[name]
        self.drag_mode = 'resize' if self._in_resize_handle(canvas_x, canvas_y, rect) else 'move'
        self.drag_start = {
            'mouse_x': canvas_x,
            'mouse_y': canvas_y,
            'rect': dict(rect),
        }

    def _mouse_drag(self, x, y):
        if not self.selected_name or not self.drag_start:
            return
        point = self._canvas_point(x, y)
        if point is None:
            return
        canvas_x, canvas_y = point
        dx = canvas_x - self.drag_start['mouse_x']
        dy = canvas_y - self.drag_start['mouse_y']
        start_rect = self.drag_start['rect']
        rect = dict(start_rect)
        if self.drag_mode == 'resize':
            rect['width'] = start_rect['width'] + dx
            rect['height'] = start_rect['height'] + dy
        else:
            rect['x'] = start_rect['x'] + dx
            rect['y'] = start_rect['y'] + dy
        self.layout_by_name[self.selected_name] = self._sanitize_rect(rect)

    def _canvas_point(self, x, y):
        canvas_y = y - self.toolbar_height
        if x < 0 or x >= self.canvas_width or canvas_y < 0 or canvas_y >= self.canvas_height:
            return None
        return x, canvas_y

    def _find_top_tile(self, x, y):
        for name in reversed(self.z_order):
            rect = self.layout_by_name.get(name)
            if not rect:
                continue
            if rect['x'] <= x < rect['x'] + rect['width'] and rect['y'] <= y < rect['y'] + rect['height']:
                return name
        return ''

    def _in_resize_handle(self, x, y, rect):
        return (
            x >= rect['x'] + rect['width'] - RESIZE_HANDLE_PX and
            y >= rect['y'] + rect['height'] - RESIZE_HANDLE_PX
        )

    def _bring_to_front(self, name):
        self.z_order = [item for item in self.z_order if item != name]
        self.z_order.append(name)

    def _run_toolbar_action(self, action):
        if action == 'save':
            self._save_layout()
        elif action == 'reset':
            self._reset_layout(delete_saved=True)
        elif action == 'grid':
            self._reset_layout(delete_saved=False)

    def _point_in_rect(self, x, y, rect):
        rx, ry, width, height = rect
        return rx <= x <= rx + width and ry <= y <= ry + height

    def _put_center(self, image, text, color, scale):
        lines = str(text).splitlines() or ['']
        sizes = [
            cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0]
            for line in lines
        ]
        line_height = max((size[1] for size in sizes), default=14) + 8
        total_height = line_height * len(lines)
        y = max((image.shape[0] - total_height) // 2 + line_height, 20)
        for line, size in zip(lines, sizes):
            x = max((image.shape[1] - size[0]) // 2, 4)
            self._put_text(image, line, (x, y), scale, color, 1)
            y += line_height

    def _put_text(self, image, text, origin, scale, color, thickness):
        cv2.putText(
            image,
            str(text),
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )


def main(args=None):
    rclpy.init(args=args)
    node = OperatorImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
