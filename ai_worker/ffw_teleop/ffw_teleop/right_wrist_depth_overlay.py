import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from std_msgs.msg import String


class WristDepthOverlay(Node):

    def __init__(self):
        super().__init__('wrist_depth_overlay')

        self.declare_parameter('depth_topic', '/camera_right/camera_right/depth/image_rect_raw')
        self.declare_parameter('base_image_topic', '')
        self.declare_parameter('base_image_fallback_topics', [''])
        self.declare_parameter('overlay_topic', '/teleop/wrist_right/depth_overlay')
        self.declare_parameter('compressed_topic', '/teleop/wrist_right/depth_overlay/compressed')
        self.declare_parameter('assist_topic', '/teleop/wrist_right/depth_assist/compressed')
        self.declare_parameter(
            'color_compressed_topic',
            '/teleop/wrist_right/color/compressed')
        self.declare_parameter('base_compressed_topic', '/teleop/wrist_right/color/compressed')
        self.declare_parameter('center_distance_topic', '/teleop/wrist_right/center_distance_m')
        self.declare_parameter('metrics_topic', '/teleop/wrist_right/depth_metrics')
        self.declare_parameter('stream_stats_topic', '/teleop/stream_stats')
        self.declare_parameter('stream_stats_name', '')
        self.declare_parameter('camera_perf_topic', '/teleop/camera_perf')
        self.declare_parameter('side', 'right')
        self.declare_parameter('feedback_visual_mode', 'assist')
        self.declare_parameter('subscribe_base_image', False)
        self.declare_parameter('publish_raw_overlay', False)
        self.declare_parameter('publish_base_compressed', False)
        self.declare_parameter('base_compressed_fps', 5.0)
        self.declare_parameter('base_compressed_jpeg_quality', 60)
        self.declare_parameter('publish_metrics', True)
        self.declare_parameter('publish_fps', 30.0)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_depth_m', 0.07)
        self.declare_parameter('max_depth_m', 0.70)
        self.declare_parameter('roi_size_px', 32)
        self.declare_parameter('jpeg_quality', 88)
        self.declare_parameter('colormap', 'TURBO')
        self.declare_parameter('depth_colormap', '')
        self.declare_parameter('base_alpha', 0.70)
        self.declare_parameter('depth_alpha', 0.30)
        self.declare_parameter('base_image_timeout_s', 0.5)
        self.declare_parameter('show_depth_contours', True)
        self.declare_parameter('contour_near_depth_m', 0.45)
        self.declare_parameter('contour_min_area_px', 30.0)
        self.declare_parameter('invalid_depth_mode', 'base_only')
        self.declare_parameter('assist_component_margin_m', 0.08)
        self.declare_parameter('assist_offset_threshold_px', 24)
        self.declare_parameter('assist_target_depth_m', 0.30)
        self.declare_parameter('assist_depth_tolerance_m', 0.04)
        self.declare_parameter('view_preset', 'driver_90')
        self.declare_parameter('view_rotate_deg', 90.0)
        self.declare_parameter('view_flip_horizontal', False)
        self.declare_parameter('view_flip_vertical', False)
        self.declare_parameter('gripper_target_offset_x_px', 0)
        self.declare_parameter('gripper_target_offset_y_px', 96)
        self.declare_parameter('band_red_max_m', 0.06)
        self.declare_parameter('band_green_min_m', 0.06)
        self.declare_parameter('band_green_max_m', 0.13)
        self.declare_parameter('band_orange_min_m', 0.13)
        self.declare_parameter('band_orange_max_m', 0.20)
        self.declare_parameter('band_alpha', 0.45)
        self.declare_parameter('band_min_area_px', 20.0)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.base_image_topic = str(self.get_parameter('base_image_topic').value).strip()
        self.base_image_topics = self._unique_strings([
            self.base_image_topic,
            *self._string_list(self.get_parameter('base_image_fallback_topics').value),
        ])
        self.overlay_topic = self.get_parameter('overlay_topic').value
        self.compressed_topic = self.get_parameter('compressed_topic').value
        self.assist_topic = self.get_parameter('assist_topic').value
        self.color_compressed_topic = str(
            self.get_parameter('color_compressed_topic').value).strip()
        self.base_compressed_topic = self.get_parameter('base_compressed_topic').value
        self.center_distance_topic = self.get_parameter('center_distance_topic').value
        self.metrics_topic = self.get_parameter('metrics_topic').value
        self.stream_stats_topic = str(self.get_parameter('stream_stats_topic').value).strip()
        self.camera_perf_topic = str(self.get_parameter('camera_perf_topic').value).strip()
        self.side = str(self.get_parameter('side').value).strip() or 'unknown'
        self.stream_stats_name = str(self.get_parameter('stream_stats_name').value).strip()
        if not self.stream_stats_name:
            self.stream_stats_name = f'wrist_{self.side}'
        self.feedback_visual_mode = str(
            self.get_parameter('feedback_visual_mode').value).strip().lower()
        if self.feedback_visual_mode not in ('assist', 'overlay', 'off'):
            self.get_logger().warn(
                f'unknown feedback_visual_mode={self.feedback_visual_mode}; using assist')
            self.feedback_visual_mode = 'assist'
        requested_base_image = self._as_bool(
            self.get_parameter('subscribe_base_image').value)
        self.subscribe_base_image = requested_base_image and bool(self.base_image_topics)
        if requested_base_image and not self.base_image_topics:
            self.get_logger().warn(
                'raw wrist color subscription requested but no base image topics are configured')
        self.publish_raw_overlay = self._as_bool(
            self.get_parameter('publish_raw_overlay').value)
        requested_base_compressed = self._as_bool(
            self.get_parameter('publish_base_compressed').value)
        self.publish_base_compressed = requested_base_compressed
        if self.publish_base_compressed and not self.subscribe_base_image:
            self.get_logger().warn(
                'base compressed relay requested but raw wrist color subscription is disabled')
        self.base_compressed_fps = max(
            float(self.get_parameter('base_compressed_fps').value), 0.1)
        self.base_compressed_jpeg_quality = int(np.clip(
            int(self.get_parameter('base_compressed_jpeg_quality').value), 1, 100))
        self.publish_metrics = self._as_bool(
            self.get_parameter('publish_metrics').value)
        self.publish_fps = max(float(self.get_parameter('publish_fps').value), 0.1)
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.roi_size_px = max(int(self.get_parameter('roi_size_px').value), 4)
        self.jpeg_quality = int(np.clip(int(self.get_parameter('jpeg_quality').value), 1, 100))
        depth_colormap = str(self.get_parameter('depth_colormap').value).strip()
        if not depth_colormap:
            depth_colormap = str(self.get_parameter('colormap').value)
        self.colormap = self._resolve_colormap(depth_colormap)
        self.base_alpha = max(float(self.get_parameter('base_alpha').value), 0.0)
        self.depth_alpha = max(float(self.get_parameter('depth_alpha').value), 0.0)
        self.base_image_timeout_s = max(
            float(self.get_parameter('base_image_timeout_s').value), 0.0)
        self.show_depth_contours = self._as_bool(
            self.get_parameter('show_depth_contours').value)
        self.contour_near_depth_m = float(self.get_parameter('contour_near_depth_m').value)
        self.contour_min_area_px = max(
            float(self.get_parameter('contour_min_area_px').value), 0.0)
        self.invalid_depth_mode = str(self.get_parameter('invalid_depth_mode').value)
        self.assist_component_margin_m = max(
            float(self.get_parameter('assist_component_margin_m').value), 0.0)
        self.assist_offset_threshold_px = max(
            int(self.get_parameter('assist_offset_threshold_px').value), 1)
        self.assist_target_depth_m = float(self.get_parameter('assist_target_depth_m').value)
        self.assist_depth_tolerance_m = max(
            float(self.get_parameter('assist_depth_tolerance_m').value), 0.0)
        self.view_preset = str(self.get_parameter('view_preset').value).strip() or 'custom'
        self.view_rotate_deg = float(self.get_parameter('view_rotate_deg').value)
        self.view_flip_horizontal = self._as_bool(
            self.get_parameter('view_flip_horizontal').value)
        self.view_flip_vertical = self._as_bool(
            self.get_parameter('view_flip_vertical').value)
        self.gripper_target_offset_x_px = int(
            self.get_parameter('gripper_target_offset_x_px').value)
        self.gripper_target_offset_y_px = int(
            self.get_parameter('gripper_target_offset_y_px').value)
        self.band_red_max_m = float(self.get_parameter('band_red_max_m').value)
        self.band_green_min_m = float(self.get_parameter('band_green_min_m').value)
        self.band_green_max_m = float(self.get_parameter('band_green_max_m').value)
        self.band_orange_min_m = float(self.get_parameter('band_orange_min_m').value)
        self.band_orange_max_m = float(self.get_parameter('band_orange_max_m').value)
        self.band_alpha = float(np.clip(float(self.get_parameter('band_alpha').value), 0.0, 1.0))
        self.band_min_area_px = max(float(self.get_parameter('band_min_area_px').value), 0.0)
        self.publish_period_ns = int(1_000_000_000 / self.publish_fps)
        self.base_compressed_period_ns = int(1_000_000_000 / self.base_compressed_fps)
        self.last_publish_time = None
        self.last_base_compressed_publish_time = None
        self.last_warn_time = None
        self.last_perf_publish_time = None
        self.depth_input_times = []
        self.base_input_times = []
        self.output_times = []
        self.last_process_ms = None
        self.last_jpeg_ms = None
        self.last_output_bytes = 0
        self.last_drop_reason = 'startup'
        self.latest_base_image = None
        self.latest_base_image_time = None
        self.latest_base_image_header = None

        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._depth_callback, qos_profile_sensor_data)
        self.base_image_subs = []
        if self.subscribe_base_image:
            for topic in self.base_image_topics:
                self.base_image_subs.append(
                    self.create_subscription(
                        Image, topic, self._base_image_callback, qos_profile_sensor_data)
                )
        self.overlay_pub = None
        if self.publish_raw_overlay:
            self.overlay_pub = self.create_publisher(
                Image, self.overlay_topic, qos_profile_sensor_data)
        self.compressed_pub = self.create_publisher(
            CompressedImage, self.compressed_topic, qos_profile_sensor_data)
        self.assist_pub = self.create_publisher(
            CompressedImage, self.assist_topic, qos_profile_sensor_data)
        self.base_compressed_pub = None
        if self.publish_base_compressed and self.base_compressed_topic:
            self.base_compressed_pub = self.create_publisher(
                CompressedImage, self.base_compressed_topic, qos_profile_sensor_data)
        self.center_pub = self.create_publisher(
            Float32, self.center_distance_topic, qos_profile_sensor_data)
        self.metrics_pub = None
        if self.publish_metrics and self.metrics_topic:
            self.metrics_pub = self.create_publisher(String, self.metrics_topic, 10)
        self.stream_stats_pub = None
        if self.stream_stats_topic:
            self.stream_stats_pub = self.create_publisher(String, self.stream_stats_topic, 10)
        self.camera_perf_pub = None
        if self.camera_perf_topic:
            self.camera_perf_pub = self.create_publisher(String, self.camera_perf_topic, 10)

        self.get_logger().info(
            f'wrist depth feedback({self.side}): mode={self.feedback_visual_mode}, '
            f'depth={self.depth_topic}, '
            f'base={self.base_image_topics if self.subscribe_base_image else "disabled"} '
            f'-> raw={self.overlay_topic if self.publish_raw_overlay else "disabled"}, '
            f'overlay_compressed={self.compressed_topic}, assist={self.assist_topic}, '
            f'color_compressed_topic={self.color_compressed_topic or "disabled"}, '
            f'base_compressed_relay={self.base_compressed_topic if self.base_compressed_pub else "disabled"}, '
            f'stats={self.stream_stats_topic or "disabled"} '
            f'view={self.view_preset} rot={self.view_rotate_deg:.0f}deg '
            f'flip_h={self.view_flip_horizontal} flip_v={self.view_flip_vertical} '
            f'gripper_target_offset=({self.gripper_target_offset_x_px}, '
            f'{self.gripper_target_offset_y_px})px display')
        if self.publish_base_compressed and self.subscribe_base_image:
            self.create_timer(2.0, self._color_diagnostic_tick)

    def _resolve_colormap(self, name):
        attr = f'COLORMAP_{name.strip().upper()}'
        return getattr(cv2, attr, cv2.COLORMAP_JET)

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _unique_strings(self, values):
        unique = []
        seen = set()
        for value in values:
            text = str(value).strip()
            if not text or text in seen:
                continue
            unique.append(text)
            seen.add(text)
        return unique

    def _string_list(self, value):
        if value is None:
            return []
        if isinstance(value, str):
            return [value]
        try:
            return [str(item) for item in value]
        except TypeError:
            return [str(value)]

    def _warn_throttled(self, message):
        now = self.get_clock().now()
        if self.last_warn_time is None or (now - self.last_warn_time).nanoseconds > 5_000_000_000:
            self.get_logger().warn(message)
            self.last_warn_time = now

    def _record_event(self, samples, now_s=None):
        if now_s is None:
            now_s = time.monotonic()
        samples.append(float(now_s))
        cutoff = float(now_s) - 5.0
        del samples[:max(0, len(samples) - 200)]
        while samples and samples[0] < cutoff:
            samples.pop(0)

    def _sample_hz(self, samples):
        if len(samples) < 2:
            return 0.0
        duration = max(float(samples[-1] - samples[0]), 1e-6)
        return (len(samples) - 1) / duration

    def _maybe_publish_camera_perf(self, now=None):
        if self.camera_perf_pub is None:
            return
        now = now or self.get_clock().now()
        if (self.last_perf_publish_time is not None and
                (now - self.last_perf_publish_time).nanoseconds < 1_000_000_000):
            return
        self.last_perf_publish_time = now
        msg = String()
        msg.data = json.dumps({
            'stamp_sec': now.nanoseconds / 1e9,
            'name': self.stream_stats_name,
            'topic': self.assist_topic,
            'depth_input_hz': self._sample_hz(self.depth_input_times),
            'base_input_hz': self._sample_hz(self.base_input_times),
            'output_hz': self._sample_hz(self.output_times),
            'process_ms': self._clean_float(self.last_process_ms),
            'jpeg_ms': self._clean_float(self.last_jpeg_ms),
            'bytes': int(self.last_output_bytes),
            'subscriber_count': int(self.count_subscribers(self.assist_topic)),
            'drop_reason': self.last_drop_reason,
        }, sort_keys=True)
        self.camera_perf_pub.publish(msg)

    def _has_subscribers(self, topic):
        return bool(topic) and self.count_subscribers(topic) > 0

    def _base_image_callback(self, msg):
        self._record_event(self.base_input_times)
        base_image = self._image_to_bgr(msg)
        if base_image is None:
            return
        self.latest_base_image = base_image
        self.latest_base_image_time = self.get_clock().now()
        self.latest_base_image_header = msg.header
        if (
            self.base_compressed_pub is not None and
            self._has_subscribers(self.base_compressed_topic) and
            self._base_compressed_due(self.latest_base_image_time)
        ):
            self._publish_base_compressed_image(msg.header, base_image)
            self.last_base_compressed_publish_time = self.latest_base_image_time

    def _depth_callback(self, msg):
        callback_start = time.perf_counter()
        now = self.get_clock().now()
        self._record_event(self.depth_input_times)
        if (self.last_publish_time is not None and
                (now - self.last_publish_time).nanoseconds < self.publish_period_ns):
            self.last_drop_reason = 'throttle'
            self._maybe_publish_camera_perf(now)
            return
        self.last_publish_time = now

        depth_m = self._image_to_depth_meters(msg)
        if depth_m is None:
            self.last_drop_reason = 'invalid_depth'
            self._maybe_publish_camera_perf(now)
            return

        metrics, assist_draw = self._depth_assist_metrics(depth_m, msg.header.stamp)
        center_distance = metrics.get('center_m')
        self._publish_center_distance(center_distance)
        self._publish_metrics(metrics)

        needs_assist = (
            self.feedback_visual_mode == 'assist' and self._has_subscribers(self.assist_topic))
        needs_raw_overlay = (
            self.feedback_visual_mode == 'overlay' and
            self.publish_raw_overlay and
            self._has_subscribers(self.overlay_topic)
        )
        needs_compressed_overlay = (
            self.feedback_visual_mode == 'overlay' and self._has_subscribers(self.compressed_topic))
        if not (needs_assist or needs_raw_overlay or needs_compressed_overlay):
            self.last_drop_reason = 'no_subscribers'
            self.last_process_ms = (time.perf_counter() - callback_start) * 1000.0
            self._maybe_publish_camera_perf(now)
            return

        base_image = None
        if needs_assist or needs_raw_overlay or needs_compressed_overlay:
            base_image = self._get_base_image(depth_m.shape, now)

        if needs_assist:
            assist = self._make_assist_image(depth_m, metrics, assist_draw, base_image)
            byte_count, jpeg_ms = self._publish_assist_image(msg, assist)
            if byte_count > 0:
                self._record_event(self.output_times)
                self.last_output_bytes = byte_count
                self.last_jpeg_ms = jpeg_ms
                self.last_drop_reason = 'published'
            else:
                self.last_drop_reason = 'jpeg_failed'

        if not (needs_raw_overlay or needs_compressed_overlay):
            self.last_process_ms = (time.perf_counter() - callback_start) * 1000.0
            self._maybe_publish_camera_perf(now)
            return

        overlay = self._make_overlay(depth_m, center_distance, base_image)
        if needs_raw_overlay:
            self._publish_overlay_image(msg, overlay)
        if needs_compressed_overlay:
            self._publish_compressed_image(msg, overlay)
        self.last_process_ms = (time.perf_counter() - callback_start) * 1000.0
        self._maybe_publish_camera_perf(now)

    def _image_to_bgr(self, msg):
        encoding = msg.encoding.upper()
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
        if channels is None:
            self._warn_throttled(f'unsupported base image encoding: {msg.encoding}')
            return None
        if msg.step <= 0 or msg.height <= 0 or msg.width <= 0:
            self._warn_throttled('received malformed base image')
            return None

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.step * msg.height
        if raw.size < expected:
            self._warn_throttled('base image data is shorter than expected')
            return None

        rows = raw[:expected].reshape((msg.height, msg.step))
        image = rows[:, :msg.width * channels].reshape((msg.height, msg.width, channels))

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
        return None

    def _image_to_depth_meters(self, msg):
        encoding = msg.encoding.upper()
        if encoding in ('16UC1', 'MONO16'):
            dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
            scale = self.depth_scale
        elif encoding == '32FC1':
            dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
            scale = 1.0
        else:
            self._warn_throttled(f'unsupported depth encoding: {msg.encoding}')
            return None

        if msg.step <= 0 or msg.height <= 0 or msg.width <= 0:
            self._warn_throttled('received malformed depth image')
            return None

        stride = msg.step // dtype.itemsize
        expected = stride * msg.height
        raw = np.frombuffer(msg.data, dtype=dtype)
        if raw.size < expected:
            self._warn_throttled('depth image data is shorter than expected')
            return None

        image = raw[:expected].reshape((msg.height, stride))[:, :msg.width]
        return image.astype(np.float32, copy=False) * scale

    def _valid_mask(self, depth_m):
        return (
            np.isfinite(depth_m) &
            (depth_m >= self.min_depth_m) &
            (depth_m <= self.max_depth_m)
        )

    def _stamp_sec(self, stamp):
        return float(stamp.sec + stamp.nanosec / 1e9)

    def _clean_float(self, value, digits=4):
        if value is None:
            return None
        value = float(value)
        if not math.isfinite(value):
            return None
        return round(value, digits)

    def _median_depth_at(self, depth_m, cx, cy, size_px):
        height, width = depth_m.shape[:2]
        half = max(size_px // 2, 1)
        x0 = max(int(cx) - half, 0)
        x1 = min(int(cx) + half, width)
        y0 = max(int(cy) - half, 0)
        y1 = min(int(cy) + half, height)
        roi = depth_m[y0:y1, x0:x1]
        valid = self._valid_mask(roi)
        if not np.any(valid):
            return float('nan')
        return float(np.median(roi[valid]))

    def _target_band(self, target_m):
        if target_m is None or not math.isfinite(float(target_m)):
            return 'unknown'
        target_m = float(target_m)
        if target_m < self.band_red_max_m:
            return 'too_close'
        if self.band_green_min_m <= target_m < self.band_green_max_m:
            return 'close'
        if self.band_orange_min_m <= target_m <= self.band_orange_max_m:
            return 'grasp_range'
        return 'outside'

    def _depth_assist_metrics(self, depth_m, stamp):
        target_pixels = self._gripper_target_pixels(depth_m.shape)
        target_px = target_pixels['raw_target_px']
        target_m = self._median_depth_at(
            depth_m, target_px['x'], target_px['y'], self.roi_size_px)
        metrics = {
            'side': self.side,
            'stamp_sec': self._clean_float(self._stamp_sec(stamp), 6),
            'center_m': self._clean_float(target_m),
            'target_m': self._clean_float(target_m),
            'camera_center_m': None,
            'grid_m': {},
            'nearest_valid': False,
            'nearest_depth_m': None,
            'nearest_cx_px': None,
            'nearest_cy_px': None,
            'nearest_area_px': None,
            'axis_valid': False,
            'axis_angle_deg': None,
            'offset_x_px': None,
            'offset_y_px': None,
            'center_minus_nearest_m': None,
            'target_minus_nearest_m': None,
            'hint': 'TARGET_DEPTH_ONLY',
            'target_band': self._target_band(target_m),
            'gripper_target': target_pixels,
            'view': {
                'preset': self.view_preset,
                'rotate_deg': self._clean_float(self.view_rotate_deg, 1),
                'flip_horizontal': self.view_flip_horizontal,
                'flip_vertical': self.view_flip_vertical,
            },
        }
        draw = {}
        draw['target_pixels'] = target_pixels
        return metrics, draw

    def _center_distance(self, depth_m):
        height, width = depth_m.shape[:2]
        half = self.roi_size_px // 2
        cx = width // 2
        cy = height // 2
        x0 = max(cx - half, 0)
        x1 = min(cx + half, width)
        y0 = max(cy - half, 0)
        y1 = min(cy + half, height)
        roi = depth_m[y0:y1, x0:x1]
        valid = self._valid_mask(roi)
        if not np.any(valid):
            return float('nan')
        return float(np.median(roi[valid]))

    def _get_base_image(self, depth_shape, now):
        if self.latest_base_image is None or self.latest_base_image_time is None:
            return self._depth_grayscale_base(depth_shape)
        age_s = (now - self.latest_base_image_time).nanoseconds / 1e9
        if self.base_image_timeout_s > 0.0 and age_s > self.base_image_timeout_s:
            return self._depth_grayscale_base(depth_shape)

        height, width = depth_shape[:2]
        base = self.latest_base_image
        if base.shape[0] != height or base.shape[1] != width:
            base = cv2.resize(base, (width, height), interpolation=cv2.INTER_LINEAR)
        return np.ascontiguousarray(base)

    def _has_fresh_base_image(self, now):
        if self.latest_base_image is None or self.latest_base_image_time is None:
            return False
        age_s = (now - self.latest_base_image_time).nanoseconds / 1e9
        return self.base_image_timeout_s <= 0.0 or age_s <= self.base_image_timeout_s

    def _color_diagnostic_tick(self):
        if self.base_compressed_pub is None or not self._has_subscribers(self.base_compressed_topic):
            return
        publisher_counts = {
            topic: self.count_publishers(topic)
            for topic in self.base_image_topics
        }
        if self.latest_base_image_time is None:
            self._warn_throttled(
                f'waiting for wrist color image; topics={publisher_counts}. '
                'Check RealSense enable_color/color_profile/USB bandwidth.')
            return
        age_s = (self.get_clock().now() - self.latest_base_image_time).nanoseconds / 1e9
        if age_s > max(self.base_image_timeout_s, 1.0):
            self._warn_throttled(
                f'wrist color image is stale ({age_s:.1f}s); topics={publisher_counts}.')

    def _base_compressed_due(self, now):
        if self.last_base_compressed_publish_time is None:
            return True
        return (
            now - self.last_base_compressed_publish_time
        ).nanoseconds >= self.base_compressed_period_ns

    def _depth_grayscale_base(self, depth_shape):
        return np.zeros((depth_shape[0], depth_shape[1], 3), dtype=np.uint8)

    def _format_depth(self, value):
        if value is None:
            return '--'
        return f'{float(value):.2f}'

    def _band_valid_mask(self, depth_m):
        return (
            np.isfinite(depth_m) &
            (depth_m > 0.0) &
            (depth_m <= self.max_depth_m)
        )

    def _draw_depth_band(self, image, mask, color):
        if not np.any(mask):
            return
        mask_u8 = mask.astype(np.uint8) * 255
        kernel = np.ones((3, 3), np.uint8)
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, kernel)
        if not np.any(mask_u8):
            return

        overlay = image.copy()
        overlay[mask_u8 > 0] = color
        cv2.addWeighted(overlay, self.band_alpha, image, 1.0 - self.band_alpha, 0.0, image)

        contours, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [
            contour for contour in contours
            if cv2.contourArea(contour) >= self.band_min_area_px
        ]
        if contours:
            cv2.drawContours(image, contours, -1, color, 2, cv2.LINE_AA)

    def _view_transform_matrix(self, width, height):
        angle = self.view_rotate_deg % 360.0
        if not math.isclose(angle, 0.0, abs_tol=1e-3):
            center = (width / 2.0, height / 2.0)
            matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
            cos = abs(matrix[0, 0])
            sin = abs(matrix[0, 1])
            new_width = int((height * sin) + (width * cos))
            new_height = int((height * cos) + (width * sin))
            matrix[0, 2] += (new_width / 2.0) - center[0]
            matrix[1, 2] += (new_height / 2.0) - center[1]
            return matrix, new_width, new_height
        matrix = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
        return matrix, width, height

    def _apply_affine_to_pixel(self, matrix, pixel):
        x, y = float(pixel[0]), float(pixel[1])
        return (
            float(matrix[0, 0] * x + matrix[0, 1] * y + matrix[0, 2]),
            float(matrix[1, 0] * x + matrix[1, 1] * y + matrix[1, 2]),
        )

    def _gripper_target_pixels(self, depth_shape):
        height, width = depth_shape[:2]
        raw_center = (width / 2.0, height / 2.0)
        matrix, display_width, display_height = self._view_transform_matrix(width, height)
        transformed_center = list(self._apply_affine_to_pixel(matrix, raw_center))
        if self.view_flip_horizontal:
            transformed_center[0] = (display_width - 1) - transformed_center[0]
        if self.view_flip_vertical:
            transformed_center[1] = (display_height - 1) - transformed_center[1]

        display_target = [
            transformed_center[0] + float(self.gripper_target_offset_x_px),
            transformed_center[1] + float(self.gripper_target_offset_y_px),
        ]
        pre_flip_target = display_target.copy()
        if self.view_flip_horizontal:
            pre_flip_target[0] = (display_width - 1) - pre_flip_target[0]
        if self.view_flip_vertical:
            pre_flip_target[1] = (display_height - 1) - pre_flip_target[1]

        inverse = cv2.invertAffineTransform(matrix)
        raw_target = self._apply_affine_to_pixel(inverse, pre_flip_target)
        raw_x = int(round(min(max(raw_target[0], 0.0), width - 1)))
        raw_y = int(round(min(max(raw_target[1], 0.0), height - 1)))
        display_x = int(round(min(max(display_target[0], 0.0), display_width - 1)))
        display_y = int(round(min(max(display_target[1], 0.0), display_height - 1)))
        return {
            'raw_center_px': {'x': int(round(raw_center[0])), 'y': int(round(raw_center[1]))},
            'raw_target_px': {'x': raw_x, 'y': raw_y},
            'display_target_px': {'x': display_x, 'y': display_y},
            'display_offset_px': {
                'x': int(self.gripper_target_offset_x_px),
                'y': int(self.gripper_target_offset_y_px),
            },
        }

    def _apply_view_transform(self, image):
        transformed = image
        height, width = transformed.shape[:2]
        matrix, new_width, new_height = self._view_transform_matrix(width, height)
        if new_width != width or new_height != height or not np.allclose(
            matrix, np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
        ):
            transformed = cv2.warpAffine(
                transformed,
                matrix,
                (new_width, new_height),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0),
            )

        flip_code = None
        if self.view_flip_horizontal and self.view_flip_vertical:
            flip_code = -1
        elif self.view_flip_horizontal:
            flip_code = 1
        elif self.view_flip_vertical:
            flip_code = 0
        if flip_code is not None:
            transformed = cv2.flip(transformed, flip_code)
        return np.ascontiguousarray(transformed)

    def _make_assist_image(self, depth_m, metrics, draw, base_image):
        if base_image is None:
            base_image = self._depth_grayscale_base(depth_m.shape)
        if base_image.shape[:2] != depth_m.shape[:2]:
            height, width = depth_m.shape[:2]
            base_image = cv2.resize(base_image, (width, height), interpolation=cv2.INTER_LINEAR)
        image = np.ascontiguousarray(base_image.copy())

        valid = self._band_valid_mask(depth_m)
        red_mask = valid & (depth_m < self.band_red_max_m)
        green_mask = (
            valid &
            (depth_m >= self.band_green_min_m) &
            (depth_m < self.band_green_max_m)
        )
        orange_mask = (
            valid &
            (depth_m >= self.band_orange_min_m) &
            (depth_m <= self.band_orange_max_m)
        )

        self._draw_depth_band(image, orange_mask, (0, 140, 255))
        self._draw_depth_band(image, green_mask, (0, 220, 0))
        self._draw_depth_band(image, red_mask, (0, 0, 255))

        height, width = depth_m.shape[:2]
        camera_center = (width // 2, height // 2)
        target_pixels = draw.get('target_pixels') or {}
        raw_target = target_pixels.get('raw_target_px') or {'x': camera_center[0], 'y': camera_center[1]}
        target = (int(raw_target.get('x', camera_center[0])), int(raw_target.get('y', camera_center[1])))
        cv2.drawMarker(image, camera_center, (180, 180, 180), cv2.MARKER_CROSS, 16, 1)
        cv2.drawMarker(image, target, (255, 255, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.circle(image, target, max(self.roi_size_px // 2, 5), (255, 255, 255), 1, cv2.LINE_AA)
        if target != camera_center:
            cv2.line(image, camera_center, target, (180, 180, 180), 1, cv2.LINE_AA)

        image = self._apply_view_transform(image)
        height, width = image.shape[:2]
        cv2.rectangle(image, (0, 0), (width - 1, 62), (16, 18, 20), -1)
        cv2.rectangle(image, (0, 0), (width - 1, 62), (230, 230, 230), 1)

        target_text = self._format_depth(metrics.get('target_m'))
        view_text = (
            f'{self.side.upper()} {self.view_preset} ROT {self.view_rotate_deg:.0f}deg '
            f'TARGET {self.gripper_target_offset_x_px:+d},{self.gripper_target_offset_y_px:+d}px'
        )
        self._put_text(image, view_text, (8, 24), 0.50)
        self._put_text(
            image,
            f'TARGET {target_text}m',
            (8, 48),
            0.46,
        )

        legend_y = height - 12
        self._put_text(image, 'GRASP 13-20cm', (8, legend_y), 0.42)
        cv2.rectangle(image, (94, max(legend_y - 12, 0)), (116, legend_y - 2), (0, 140, 255), -1)
        self._put_text(image, 'CLOSE 6-13cm', (128, legend_y), 0.42)
        cv2.rectangle(image, (236, max(legend_y - 12, 0)), (258, legend_y - 2), (0, 220, 0), -1)
        self._put_text(image, 'TOO CLOSE <6cm', (270, legend_y), 0.42)
        cv2.rectangle(image, (404, max(legend_y - 12, 0)), (426, legend_y - 2), (0, 0, 255), -1)
        return np.ascontiguousarray(image)

    def _make_overlay(self, depth_m, center_distance, base_image):
        valid = self._valid_mask(depth_m)
        clipped = np.clip(depth_m, self.min_depth_m, self.max_depth_m)
        normalized = np.zeros(depth_m.shape, dtype=np.uint8)
        scale = 255.0 / max(self.max_depth_m - self.min_depth_m, 1e-6)
        normalized[valid] = ((self.max_depth_m - clipped[valid]) * scale).astype(np.uint8)

        depth_color = cv2.applyColorMap(normalized, self.colormap)
        fallback_base = base_image
        if not np.any(fallback_base):
            gray = np.zeros(depth_m.shape, dtype=np.uint8)
            gray[valid] = (255 - normalized[valid] // 2).astype(np.uint8)
            fallback_base = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        overlay = cv2.addWeighted(
            fallback_base, self.base_alpha, depth_color, self.depth_alpha, 0.0)
        if self.invalid_depth_mode.strip().lower() == 'base_only':
            overlay[~valid] = fallback_base[~valid]
        else:
            overlay[~valid] = (20, 20, 20)

        if self.show_depth_contours:
            self._draw_depth_contours(overlay, depth_m, valid)

        height, width = depth_m.shape[:2]
        half = self.roi_size_px // 2
        cx = width // 2
        cy = height // 2
        cv2.rectangle(
            overlay,
            (max(cx - half, 0), max(cy - half, 0)),
            (min(cx + half, width - 1), min(cy + half, height - 1)),
            (255, 255, 255),
            1,
        )
        cv2.drawMarker(
            overlay, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 18, 1)

        if center_distance is not None and math.isfinite(center_distance):
            center_text = f'CENTER {center_distance:.2f} m'
        else:
            center_text = 'CENTER -- m'
        range_text = f'{self.min_depth_m:.2f}-{self.max_depth_m:.2f} m'
        self._put_text(overlay, center_text, (10, 26), 0.70)
        self._put_text(overlay, range_text, (10, 50), 0.55)
        return np.ascontiguousarray(overlay)

    def _draw_depth_contours(self, overlay, depth_m, valid):
        near_depth = self.contour_near_depth_m
        if near_depth <= 0.0:
            near_depth = self.min_depth_m + 0.35 * (self.max_depth_m - self.min_depth_m)
        near_mask = (valid & (depth_m <= near_depth)).astype(np.uint8) * 255
        if not np.any(near_mask):
            return
        kernel = np.ones((3, 3), np.uint8)
        near_mask = cv2.morphologyEx(near_mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(near_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [
            contour for contour in contours
            if cv2.contourArea(contour) >= self.contour_min_area_px
        ]
        if contours:
            cv2.drawContours(overlay, contours, -1, (255, 255, 255), 1, cv2.LINE_AA)

    def _put_text(self, image, text, origin, scale):
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (255, 255, 255), 1, cv2.LINE_AA)

    def _publish_center_distance(self, center_distance):
        msg = Float32()
        msg.data = float(center_distance) if center_distance is not None else float('nan')
        self.center_pub.publish(msg)

    def _publish_metrics(self, metrics):
        if self.metrics_pub is None:
            return
        msg = String()
        msg.data = json.dumps(metrics, sort_keys=True)
        self.metrics_pub.publish(msg)

    def _publish_overlay_image(self, source_msg, overlay):
        if self.overlay_pub is None:
            return
        msg = Image()
        msg.header = source_msg.header
        msg.height = overlay.shape[0]
        msg.width = overlay.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = overlay.shape[1] * 3
        msg.data = overlay.tobytes()
        self.overlay_pub.publish(msg)

    def _publish_compressed_image(self, source_msg, overlay):
        self._publish_jpeg(
            self.compressed_pub, source_msg.header, overlay, 'failed to JPEG-encode depth overlay')

    def _publish_assist_image(self, source_msg, image):
        return self._publish_jpeg(
            self.assist_pub,
            source_msg.header,
            image,
            'failed to JPEG-encode depth assist',
            self.stream_stats_name,
            self.assist_topic,
        )

    def _publish_base_compressed_image(self, header, image):
        stats_name = f'{self.stream_stats_name}_color' if self.stream_stats_name else ''
        self._publish_jpeg(
            self.base_compressed_pub,
            header,
            image,
            'failed to JPEG-encode base image',
            stats_name,
            self.base_compressed_topic,
            self.base_compressed_jpeg_quality,
        )

    def _publish_jpeg(self, publisher, header, image, warn_message,
                      stats_name=None, published_topic='', jpeg_quality=None):
        if publisher is None:
            return 0, 0.0
        quality = self.jpeg_quality if jpeg_quality is None else int(jpeg_quality)
        encode_start = time.perf_counter()
        ok, encoded = cv2.imencode(
            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        jpeg_ms = (time.perf_counter() - encode_start) * 1000.0
        if not ok:
            self._warn_throttled(warn_message)
            return 0, jpeg_ms
        msg = CompressedImage()
        msg.header = header
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        publisher.publish(msg)
        if stats_name:
            self._publish_stream_stats(stats_name, published_topic, image, len(msg.data), quality)
        return len(msg.data), jpeg_ms

    def _publish_stream_stats(self, name, topic, image, byte_count, jpeg_quality=None):
        if self.stream_stats_pub is None:
            return
        msg = String()
        msg.data = json.dumps({
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'name': str(name),
            'topic': str(topic),
            'bytes': int(byte_count),
            'width': int(image.shape[1]),
            'height': int(image.shape[0]),
            'jpeg_quality': int(self.jpeg_quality if jpeg_quality is None else jpeg_quality),
        }, sort_keys=True)
        self.stream_stats_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WristDepthOverlay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
