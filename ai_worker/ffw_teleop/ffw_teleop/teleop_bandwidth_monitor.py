import json
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


STREAM_ROWS = (
    ('zed', 'ZED'),
    ('wrist_right', 'R WRIST'),
    ('wrist_right_color', 'R COLOR'),
)
DEFAULT_EXPECTED_TOPICS = {
    'wrist_right_color': '/camera_right/camera_right/color/image_rect_raw',
}


class TeleopBandwidthMonitor(Node):

    def __init__(self):
        super().__init__('teleop_bandwidth_monitor')

        self.declare_parameter('stream_stats_topic', '/teleop/stream_stats')
        self.declare_parameter('camera_perf_topic', '/teleop/camera_perf')
        self.declare_parameter('monitor_topic', '/teleop/bandwidth_monitor')
        self.declare_parameter('panel_topic', '/teleop/bandwidth_monitor/compressed')
        self.declare_parameter('available_mbps', 350.0)
        self.declare_parameter('window_s', 3.0)
        self.declare_parameter('stale_timeout_s', 1.5)
        self.declare_parameter('publish_hz', 2.0)
        self.declare_parameter('panel_width', 1280)
        self.declare_parameter('panel_height', 720)
        self.declare_parameter('panel_jpeg_quality', 95)
        self.declare_parameter('network_interface', '')
        self.declare_parameter('network_tx_enabled', True)
        self.declare_parameter('target_fps', 30.0)
        self.declare_parameter('usb_estimate_enabled', True)
        self.declare_parameter('usb_available_mbps', 320.0)
        self.declare_parameter('usb_depth_bytes_per_pixel', 2.0)
        self.declare_parameter('usb_color_bytes_per_pixel', 3.0)
        self.declare_parameter('usb_overhead_factor', 1.15)
        self.declare_parameter('usb_wrist_left_depth_profile', '')
        self.declare_parameter('usb_wrist_right_depth_profile', '')
        self.declare_parameter('usb_wrist_left_color_profile', '')
        self.declare_parameter('usb_wrist_right_color_profile', '')
        self.declare_parameter('usb_wrist_left_depth_enabled', True)
        self.declare_parameter('usb_wrist_right_depth_enabled', True)
        self.declare_parameter('usb_wrist_left_color_enabled', True)
        self.declare_parameter('usb_wrist_right_color_enabled', True)
        self.declare_parameter(
            'wrist_right_color_topic',
            DEFAULT_EXPECTED_TOPICS['wrist_right_color'])
        self.declare_parameter('wrist_right_color_compressed_topic', '')

        self.stream_stats_topic = str(
            self.get_parameter('stream_stats_topic').value).strip()
        self.camera_perf_topic = str(
            self.get_parameter('camera_perf_topic').value).strip()
        self.monitor_topic = str(self.get_parameter('monitor_topic').value).strip()
        self.panel_topic = str(self.get_parameter('panel_topic').value).strip()
        self.available_mbps = max(float(self.get_parameter('available_mbps').value), 0.1)
        self.window_s = max(float(self.get_parameter('window_s').value), 0.5)
        self.stale_timeout_s = max(float(self.get_parameter('stale_timeout_s').value), 0.2)
        publish_hz = max(float(self.get_parameter('publish_hz').value), 0.2)
        self.panel_width = max(int(self.get_parameter('panel_width').value), 480)
        self.panel_height = max(int(self.get_parameter('panel_height').value), 220)
        self.panel_jpeg_quality = int(np.clip(
            int(self.get_parameter('panel_jpeg_quality').value), 1, 100))
        self.network_tx_enabled = self._as_bool(
            self.get_parameter('network_tx_enabled').value)
        self.network_interface = str(
            self.get_parameter('network_interface').value).strip()
        self.target_fps = max(float(self.get_parameter('target_fps').value), 0.1)
        self.usb_estimate_enabled = self._as_bool(
            self.get_parameter('usb_estimate_enabled').value)
        self.usb_available_mbps = max(
            float(self.get_parameter('usb_available_mbps').value), 1.0)
        self.usb_depth_bytes_per_pixel = max(
            float(self.get_parameter('usb_depth_bytes_per_pixel').value), 0.1)
        self.usb_color_bytes_per_pixel = max(
            float(self.get_parameter('usb_color_bytes_per_pixel').value), 0.1)
        self.usb_overhead_factor = max(
            float(self.get_parameter('usb_overhead_factor').value), 1.0)
        if self.network_tx_enabled and not self.network_interface:
            self.network_interface = self._detect_network_interface()
        self.last_tx_sample = None
        self.samples = {}
        self.camera_perf = {}
        wrist_right_color_topic = str(
            self.get_parameter('wrist_right_color_topic').value).strip()
        legacy_color_topic = str(
            self.get_parameter('wrist_right_color_compressed_topic').value).strip()
        if legacy_color_topic:
            wrist_right_color_topic = legacy_color_topic
        self.expected_topics = {
            'wrist_right_color': wrist_right_color_topic,
        }

        self.create_subscription(
            String, self.stream_stats_topic, self._stream_stats_callback, 100)
        if self.camera_perf_topic:
            self.create_subscription(
                String, self.camera_perf_topic, self._camera_perf_callback, 100)
        self.monitor_pub = self.create_publisher(String, self.monitor_topic, 10)
        self.panel_pub = self.create_publisher(CompressedImage, self.panel_topic, 1)
        self.timer = self.create_timer(1.0 / publish_hz, self._publish)

        self.get_logger().info(
            f'teleop bandwidth monitor: stats={self.stream_stats_topic}, '
            f'perf={self.camera_perf_topic or "disabled"}, '
            f'panel={self.panel_topic}, budget={self.available_mbps:.1f} Mbps, '
            f'net_if={self.network_interface or "disabled"}, '
            f'color_topics={self.expected_topics}')

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _stream_stats_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('ignoring malformed stream stats JSON')
            return

        name = str(payload.get('name') or '').strip()
        if not name:
            return
        try:
            byte_count = int(payload.get('bytes', 0))
        except (TypeError, ValueError):
            return
        if byte_count <= 0:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        sample = {
            't': now,
            'bytes': byte_count,
            'topic': str(payload.get('topic') or ''),
            'width': self._int_or_none(payload.get('width')),
            'height': self._int_or_none(payload.get('height')),
            'jpeg_quality': self._int_or_none(payload.get('jpeg_quality')),
        }
        samples = self.samples.setdefault(name, [])
        samples.append(sample)
        self.samples[name] = [
            item for item in samples
            if now - item['t'] <= max(self.window_s, self.stale_timeout_s)
        ]

    def _camera_perf_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('ignoring malformed camera perf JSON')
            return

        name = str(payload.get('name') or '').strip()
        if not name:
            return
        payload['_received_sec'] = self.get_clock().now().nanoseconds / 1e9
        self.camera_perf[name] = payload

    def _publish(self):
        now = self.get_clock().now().nanoseconds / 1e9
        streams = {}
        for name, _ in STREAM_ROWS:
            streams[name] = self._stream_snapshot(name, now)
            streams[name]['camera_perf'] = self._camera_perf_snapshot(name, now)
        for name in sorted(set(self.samples.keys()) - set(streams.keys())):
            streams[name] = self._stream_snapshot(name, now)
            streams[name]['camera_perf'] = self._camera_perf_snapshot(name, now)

        total_mbps = sum(
            stream['mbps'] for stream in streams.values()
            if stream.get('fresh')
        )
        usage_percent = 100.0 * total_mbps / self.available_mbps
        payload = {
            'stamp_sec': now,
            'available_mbps': self.available_mbps,
            'total_mbps': total_mbps,
            'usage_percent': usage_percent,
            'headroom_mbps': self.available_mbps - total_mbps,
            'net_tx_mbps': self._network_tx_mbps(now),
            'usb_estimate': self._usb_estimate(),
            'camera_perf': {
                name: self._camera_perf_snapshot(name, now)
                for name in sorted(self.camera_perf.keys())
            },
            'streams': streams,
        }

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.monitor_pub.publish(msg)
        self._publish_panel(payload)

    def _stream_snapshot(self, name, now):
        samples = [
            sample for sample in self.samples.get(name, [])
            if now - sample['t'] <= self.window_s
        ]
        self.samples[name] = samples
        if not samples or now - samples[-1]['t'] > self.stale_timeout_s:
            latest = samples[-1] if samples else {}
            topic = latest.get('topic') or self.expected_topics.get(name, '')
            publisher_count = self.count_publishers(topic) if topic else None
            return {
                'fresh': False,
                'has_samples': bool(samples),
                'fps': 0.0,
                'mbps': 0.0,
                'bytes_per_frame': int(latest.get('bytes', 0) or 0),
                'width': latest.get('width'),
                'height': latest.get('height'),
                'jpeg_quality': latest.get('jpeg_quality'),
                'topic': topic,
                'publishers': publisher_count,
            }

        duration = max(min(self.window_s, now - samples[0]['t']), 0.2)
        total_bytes = sum(sample['bytes'] for sample in samples)
        latest = samples[-1]
        topic = latest.get('topic') or self.expected_topics.get(name, '')
        return {
            'fresh': True,
            'has_samples': True,
            'fps': len(samples) / duration,
            'mbps': total_bytes * 8.0 / duration / 1e6,
            'bytes_per_frame': total_bytes / max(len(samples), 1),
            'width': latest.get('width'),
            'height': latest.get('height'),
            'jpeg_quality': latest.get('jpeg_quality'),
            'topic': topic,
            'publishers': self.count_publishers(topic) if topic else None,
        }

    def _camera_perf_snapshot(self, name, now):
        payload = self.camera_perf.get(name)
        if not payload:
            return {}
        received = self._float_or_none(payload.get('_received_sec'))
        stale = received is None or now - received > max(self.stale_timeout_s * 2.0, 2.0)
        snapshot = {
            'fresh': not stale,
            'stale': stale,
        }
        for key in (
            'topic',
            'assist_mode',
            'depth_input_hz',
            'base_input_hz',
            'rx_hz',
            'output_hz',
            'process_ms',
            'jpeg_ms',
            'bytes',
            'subscriber_count',
            'drop_reason',
            'width',
            'height',
        ):
            if key in payload:
                snapshot[key] = payload.get(key)
        return snapshot

    def _publish_panel(self, payload):
        if self.count_subscribers(self.panel_topic) <= 0:
            return
        image = self._render_panel(payload)
        ok, encoded = cv2.imencode(
            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), self.panel_jpeg_quality])
        if not ok:
            self.get_logger().warn('failed to JPEG-encode bandwidth monitor panel')
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'teleop_bandwidth_monitor'
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        self.panel_pub.publish(msg)

    def _render_panel(self, payload):
        width = self.panel_width
        height = self.panel_height
        scale = min(width / 640.0, height / 360.0)
        margin = int(round(18 * scale))
        def p(x, y):
            return int(round(x * scale)), int(round(y * scale))
        def line(value):
            return max(int(round(value * scale)), 1)
        image = np.full((height, width, 3), (30, 32, 36), dtype=np.uint8)
        total_mbps = float(payload.get('total_mbps') or 0.0)
        usage = float(payload.get('usage_percent') or 0.0)
        headroom = float(payload.get('headroom_mbps') or 0.0)
        header_color = self._usage_color(usage)
        cv2.rectangle(image, (0, 0), (width, int(round(58 * scale))), header_color, -1)
        self._put_text(
            image, 'BANDWIDTH MONITOR', p(16, 38),
            0.82 * scale, (255, 255, 255), line(2))
        self._put_text(
            image,
            f'{total_mbps:5.1f} / {self.available_mbps:.0f} Mbps  {usage:4.0f}%',
            p(330, 38),
            0.58 * scale,
            (255, 255, 255),
            line(2),
        )

        self._draw_usage_bar(
            image, margin, int(round(76 * scale)), width - 2 * margin,
            int(round(18 * scale)), usage)
        self._put_text(
            image,
            f'HEADROOM {headroom:5.1f} Mbps',
            p(18, 116),
            0.52 * scale,
            (226, 232, 238),
            line(1),
        )
        net_tx = payload.get('net_tx_mbps')
        net_text = 'NET TX --' if net_tx is None else f'NET TX {float(net_tx):5.1f} Mbps'
        self._put_text(
            image, net_text, p(340, 116), 0.52 * scale, (226, 232, 238), line(1))

        usb = payload.get('usb_estimate') or {}
        usb_text, usb_color = self._usb_summary_text(usb)
        self._put_text(image, usb_text, p(18, 140), 0.44 * scale, usb_color, line(1))

        streams = payload.get('streams') or {}
        y = int(round(168 * scale))
        row_step = int(round(34 * scale))
        for name, label in STREAM_ROWS:
            self._draw_stream_row(image, name, label, streams.get(name) or {}, y, scale)
            y += row_step

        self._put_text(
            image,
            f'Rolling window {self.window_s:.1f}s  Budget {self.available_mbps:.0f} Mbps',
            (margin, height - int(round(18 * scale))),
            0.42 * scale,
            (172, 184, 194),
            line(1),
        )
        return image

    def _draw_usage_bar(self, image, x, y, width, height, usage_percent):
        cv2.rectangle(image, (x, y), (x + width, y + height), (64, 67, 74), -1)
        fill_width = int(width * min(max(usage_percent, 0.0), 100.0) / 100.0)
        if fill_width > 0:
            cv2.rectangle(
                image,
                (x, y),
                (x + fill_width, y + height),
                self._usage_color(usage_percent),
                -1,
            )
        cv2.rectangle(image, (x, y), (x + width, y + height), (160, 168, 176), 1)

    def _draw_stream_row(self, image, name, label, stream, y, scale):
        def line(value):
            return max(int(round(value * scale)), 1)
        fresh = bool(stream.get('fresh'))
        color = (226, 232, 238) if fresh else (126, 134, 144)
        if fresh:
            fps = float(stream.get('fps') or 0.0)
            mbps = float(stream.get('mbps') or 0.0)
            usage = 100.0 * mbps / self.available_mbps
            width = stream.get('width')
            height = stream.get('height')
            quality = stream.get('jpeg_quality')
            res = f'{width}x{height}' if width and height else '--'
            q_text = f'Q{int(quality)}' if quality is not None else 'Q--'
            target_fps = self._target_fps_for_stream(name)
            fps_state = 'LOW FPS' if fps < target_fps * 0.8 else 'OK'
            perf_text = self._perf_text(stream)
            text = f'{label:<7} {fps:4.1f}Hz {mbps:5.1f}M {res:<9} {q_text} {fps_state} {perf_text}'
            bar_color = self._usage_color(usage)
        else:
            state = self._stale_state_text(stream)
            perf_text = self._perf_text(stream)
            text = f'{label:<7} -- Hz   -- M     {state} {perf_text}'
            usage = 0.0
            bar_color = (86, 90, 98)
        self._put_text(
            image, text, (int(round(18 * scale)), y), 0.46 * scale, color, line(1))
        bar_width = int(round(104 * scale))
        x = self.panel_width - bar_width - int(round(18 * scale))
        y_top = y - int(round(15 * scale))
        y_bottom = y - int(round(4 * scale))
        cv2.rectangle(image, (x, y_top), (x + bar_width, y_bottom), (58, 61, 68), -1)
        fill_width = int(bar_width * min(max(usage, 0.0), 100.0) / 100.0)
        if fill_width > 0:
            cv2.rectangle(image, (x, y_top), (x + fill_width, y_bottom), bar_color, -1)
        cv2.rectangle(image, (x, y_top), (x + bar_width, y_bottom), (130, 138, 148), line(1))

    def _stale_state_text(self, stream):
        publishers = stream.get('publishers')
        if publishers == 0:
            return 'NO PUB'
        if publishers and not stream.get('has_samples'):
            return 'NO STATS'
        return 'STALE'

    def _perf_text(self, stream):
        perf = stream.get('camera_perf') or {}
        if not perf:
            return 'S-- O-- P-- J--'
        if perf.get('stale'):
            return 'S stale O-- P-- J--'
        depth_hz = self._float_or_none(perf.get('depth_input_hz'))
        base_hz = self._float_or_none(perf.get('base_input_hz'))
        rx_hz = self._float_or_none(perf.get('rx_hz'))
        mode = str(perf.get('assist_mode') or '').strip().lower()
        if base_hz is not None and mode in ('tf_header', 'tf_only'):
            src_label = 'B'
            src_hz = base_hz
        elif depth_hz is not None:
            src_label = 'D'
            src_hz = depth_hz
        elif rx_hz is not None:
            src_label = 'R'
            src_hz = rx_hz
        elif base_hz is not None:
            src_label = 'B'
            src_hz = base_hz
        else:
            src_label = 'S'
            src_hz = None
        out_hz = self._float_or_none(perf.get('output_hz'))
        process_ms = self._float_or_none(perf.get('process_ms'))
        jpeg_ms = self._float_or_none(perf.get('jpeg_ms'))
        src = '--' if src_hz is None else f'{src_hz:.1f}'
        out = '--' if out_hz is None else f'{out_hz:.1f}'
        proc = '--' if process_ms is None else f'{process_ms:.0f}'
        jpeg = '--' if jpeg_ms is None else f'{jpeg_ms:.0f}'
        drop = str(perf.get('drop_reason') or '')
        if drop and drop not in ('published', 'main_rx'):
            return f'{src_label}{src} O{out} P{proc} J{jpeg} {drop[:8]}'
        return f'{src_label}{src} O{out} P{proc} J{jpeg}'

    def _target_fps_for_stream(self, name):
        profile_param_by_name = {
            'wrist_left': 'usb_wrist_left_depth_profile',
            'wrist_right': 'usb_wrist_right_depth_profile',
            'wrist_left_color': 'usb_wrist_left_color_profile',
            'wrist_right_color': 'usb_wrist_right_color_profile',
        }
        profile_param = profile_param_by_name.get(name)
        if not profile_param:
            return self.target_fps
        _, _, fps = self._parse_camera_profile(self.get_parameter(profile_param).value)
        return fps if fps > 0.0 else self.target_fps

    def _usage_color(self, usage_percent):
        if usage_percent >= 90.0:
            return (54, 68, 224)
        if usage_percent >= 70.0:
            return (42, 154, 232)
        return (65, 166, 92)

    def _usb_summary_text(self, usb):
        if not self.usb_estimate_enabled or not usb:
            return 'USB REQ --', (172, 184, 194)
        total = float(usb.get('total_mbps') or 0.0)
        available = float(usb.get('available_mbps') or self.usb_available_mbps)
        usage = 100.0 * total / max(available, 1.0)
        profiles = usb.get('profiles') or []
        active = ', '.join(
            f"{item.get('label')} {item.get('profile')}"
            for item in profiles
            if item.get('enabled') and item.get('profile')
        )
        if len(active) > 80:
            active = active[:77] + '...'
        text = f'USB REQ ~{total:5.0f}/{available:.0f} Mbps {usage:4.0f}%'
        if active:
            text = f'{text}  {active}'
        return text, self._usage_color(usage)

    def _usb_estimate(self):
        if not self.usb_estimate_enabled:
            return {}
        specs = (
            ('L-D', 'usb_wrist_left_depth_profile', 'usb_wrist_left_depth_enabled', 'depth'),
            ('R-D', 'usb_wrist_right_depth_profile', 'usb_wrist_right_depth_enabled', 'depth'),
            ('L-C', 'usb_wrist_left_color_profile', 'usb_wrist_left_color_enabled', 'color'),
            ('R-C', 'usb_wrist_right_color_profile', 'usb_wrist_right_color_enabled', 'color'),
        )
        profiles = []
        total = 0.0
        for label, profile_param, enabled_param, kind in specs:
            profile = str(self.get_parameter(profile_param).value or '').strip()
            enabled = self._as_bool(self.get_parameter(enabled_param).value)
            width, height, fps = self._parse_camera_profile(profile)
            bpp = (
                self.usb_depth_bytes_per_pixel
                if kind == 'depth'
                else self.usb_color_bytes_per_pixel
            )
            mbps = 0.0
            if enabled and width > 0 and height > 0 and fps > 0.0:
                mbps = width * height * fps * bpp * 8.0 / 1e6
                mbps *= self.usb_overhead_factor
            total += mbps
            profiles.append({
                'label': label,
                'profile': profile,
                'enabled': enabled,
                'kind': kind,
                'estimated_mbps': mbps,
            })
        return {
            'available_mbps': self.usb_available_mbps,
            'total_mbps': total,
            'usage_percent': 100.0 * total / self.usb_available_mbps,
            'profiles': profiles,
        }

    def _parse_camera_profile(self, profile):
        parts = [part.strip() for part in str(profile or '').split(',')]
        if len(parts) != 3:
            return 0, 0, 0.0
        try:
            return int(parts[0]), int(parts[1]), float(parts[2])
        except ValueError:
            return 0, 0, 0.0

    def _put_text(self, image, text, origin, scale, color, thickness):
        cv2.putText(
            image, str(text), origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            (0, 0, 0), thickness + 2, cv2.LINE_AA)
        cv2.putText(
            image, str(text), origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            color, thickness, cv2.LINE_AA)

    def _network_tx_mbps(self, now):
        if not self.network_tx_enabled or not self.network_interface:
            return None
        tx_bytes = self._read_tx_bytes(self.network_interface)
        if tx_bytes is None:
            return None
        if self.last_tx_sample is None:
            self.last_tx_sample = (now, tx_bytes)
            return None
        prev_time, prev_bytes = self.last_tx_sample
        self.last_tx_sample = (now, tx_bytes)
        dt = now - prev_time
        if dt <= 0.0 or tx_bytes < prev_bytes:
            return None
        return (tx_bytes - prev_bytes) * 8.0 / dt / 1e6

    def _detect_network_interface(self):
        route_path = '/proc/net/route'
        try:
            with open(route_path, 'r', encoding='utf-8') as stream:
                for line in stream.readlines()[1:]:
                    parts = line.split()
                    if len(parts) >= 2 and parts[1] == '00000000':
                        return parts[0]
        except OSError:
            pass

        net_path = '/sys/class/net'
        try:
            for name in sorted(os.listdir(net_path)):
                if name == 'lo':
                    continue
                state_path = os.path.join(net_path, name, 'operstate')
                try:
                    with open(state_path, 'r', encoding='utf-8') as stream:
                        if stream.read().strip() == 'up':
                            return name
                except OSError:
                    continue
        except OSError:
            return ''
        return ''

    def _read_tx_bytes(self, interface):
        path = os.path.join('/sys/class/net', interface, 'statistics', 'tx_bytes')
        try:
            with open(path, 'r', encoding='utf-8') as stream:
                return int(stream.read().strip())
        except (OSError, ValueError):
            return None

    def _int_or_none(self, value):
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _float_or_none(self, value):
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def _first_float(self, *values):
        for value in values:
            result = self._float_or_none(value)
            if result is not None:
                return result
        return None


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBandwidthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
