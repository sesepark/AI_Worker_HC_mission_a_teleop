import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import String
import tf2_ros


class ZedDepthAssist(Node):

    def __init__(self):
        super().__init__('zed_depth_assist')

        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('base_image_topic', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('base_image_fallback_topics', [
            '/zed/zed_node/rgb/image_rect_color',
        ])
        self.declare_parameter('camera_info_topic', '/zed/zed_node/left/camera_info')
        self.declare_parameter('camera_info_fallback_topics', [
            '/zed/zed_node/left/camera_info',
            '/zed/zed_node/rgb/camera_info',
        ])
        self.declare_parameter('assist_topic', '/teleop/zed/depth_assist/compressed')
        self.declare_parameter('metrics_topic', '/teleop/zed/depth_metrics')
        self.declare_parameter('stream_stats_topic', '/teleop/stream_stats')
        self.declare_parameter('stream_stats_name', 'zed')
        self.declare_parameter('camera_perf_topic', '/teleop/camera_perf')
        self.declare_parameter('operator_ok_topic', '/teleop/operator_ok')
        self.declare_parameter('ok_overlay_duration_s', 3.0)
        self.declare_parameter('assist_mode', 'tf_header')
        self.declare_parameter('publish_fps', 30.0)
        self.declare_parameter('jpeg_quality', 88)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 4.0)
        self.declare_parameter('base_image_timeout_s', 0.5)
        self.declare_parameter('camera_optical_frame', '')
        self.declare_parameter('camera_frame_fallbacks', [
            'zedm_left_camera_optical_frame',
            'zed_left_camera_optical_frame',
        ])
        self.declare_parameter('use_latest_tf', True)
        self.declare_parameter('tf_lookup_timeout_s', 0.005)
        self.declare_parameter('left_hand_frame', 'end_effector_l_link')
        self.declare_parameter('right_hand_frame', 'end_effector_r_link')
        self.declare_parameter('left_hand_frame_fallbacks', [
            'end_effector_l_link',
            'gripper_l_rh_p12_rn_base',
            'arm_l_link7',
        ])
        self.declare_parameter('right_hand_frame_fallbacks', [
            'end_effector_r_link',
            'gripper_r_rh_p12_rn_base',
            'arm_r_link7',
        ])
        self.declare_parameter('left_arm_links', [
            'arm_l_link4',
            'arm_l_link5',
            'arm_l_link6',
            'arm_l_link7',
            'end_effector_l_link',
        ])
        self.declare_parameter('right_arm_links', [
            'arm_r_link4',
            'arm_r_link5',
            'arm_r_link6',
            'arm_r_link7',
            'end_effector_r_link',
        ])
        self.declare_parameter('hand_roi_radius_px', 110)
        self.declare_parameter('robot_mask_radius_px', 22)
        self.declare_parameter('robot_mask_dilate_px', 18)
        self.declare_parameter('near_hand_radius_m', 0.35)
        self.declare_parameter('max_objects_per_hand', 3)
        self.declare_parameter('component_min_area_px', 80.0)
        self.declare_parameter('enable_near_hand_objects', False)

        self.depth_topic = str(self.get_parameter('depth_topic').value).strip()
        self.base_image_topic = str(self.get_parameter('base_image_topic').value).strip()
        self.base_image_topics = self._unique_strings([
            self.base_image_topic,
            *self._string_list(self.get_parameter('base_image_fallback_topics').value),
            '/zed/zed_node/left/image_rect_color',
            '/zed/zed_node/rgb/image_rect_color',
        ])
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value).strip()
        self.camera_info_topics = self._unique_strings([
            self.camera_info_topic,
            *self._string_list(self.get_parameter('camera_info_fallback_topics').value),
            '/zed/zed_node/left/camera_info',
            '/zed/zed_node/rgb/camera_info',
        ])
        self.assist_topic = str(self.get_parameter('assist_topic').value).strip()
        self.metrics_topic = str(self.get_parameter('metrics_topic').value).strip()
        self.stream_stats_topic = str(self.get_parameter('stream_stats_topic').value).strip()
        self.stream_stats_name = str(
            self.get_parameter('stream_stats_name').value).strip() or 'zed'
        self.camera_perf_topic = str(self.get_parameter('camera_perf_topic').value).strip()
        self.operator_ok_topic = str(self.get_parameter('operator_ok_topic').value).strip()
        self.ok_overlay_duration_s = max(
            float(self.get_parameter('ok_overlay_duration_s').value), 0.1)
        self.assist_mode = str(self.get_parameter('assist_mode').value).strip().lower()
        if self.assist_mode not in ('tf_header', 'tf_only', 'depth'):
            self.get_logger().warn(
                f'unknown ZED assist_mode={self.assist_mode!r}; using tf_header')
            self.assist_mode = 'tf_header'
        self.publish_fps = max(float(self.get_parameter('publish_fps').value), 0.1)
        self.jpeg_quality = int(np.clip(int(self.get_parameter('jpeg_quality').value), 1, 100))
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.base_image_timeout_s = max(
            float(self.get_parameter('base_image_timeout_s').value), 0.0)
        self.camera_optical_frame = str(self.get_parameter('camera_optical_frame').value).strip()
        self.camera_frame_fallbacks = [
            str(frame).strip() for frame in self.get_parameter('camera_frame_fallbacks').value
            if str(frame).strip()
        ]
        self.use_latest_tf = self._as_bool(self.get_parameter('use_latest_tf').value)
        tf_timeout_s = max(float(self.get_parameter('tf_lookup_timeout_s').value), 0.0)
        self.tf_lookup_timeout = Duration(nanoseconds=int(tf_timeout_s * 1_000_000_000))
        self.left_hand_frame = str(self.get_parameter('left_hand_frame').value).strip()
        self.right_hand_frame = str(self.get_parameter('right_hand_frame').value).strip()
        self.left_hand_frames = self._unique_strings([
            self.left_hand_frame,
            *self._string_list(self.get_parameter('left_hand_frame_fallbacks').value),
        ])
        self.right_hand_frames = self._unique_strings([
            self.right_hand_frame,
            *self._string_list(self.get_parameter('right_hand_frame_fallbacks').value),
        ])
        self.left_arm_links = [
            str(frame).strip() for frame in self.get_parameter('left_arm_links').value
            if str(frame).strip()
        ]
        self.right_arm_links = [
            str(frame).strip() for frame in self.get_parameter('right_arm_links').value
            if str(frame).strip()
        ]
        self.hand_roi_radius_px = max(int(self.get_parameter('hand_roi_radius_px').value), 12)
        self.robot_mask_radius_px = max(int(self.get_parameter('robot_mask_radius_px').value), 1)
        self.robot_mask_dilate_px = max(int(self.get_parameter('robot_mask_dilate_px').value), 0)
        self.near_hand_radius_m = max(float(self.get_parameter('near_hand_radius_m').value), 0.05)
        self.max_objects_per_hand = max(int(self.get_parameter('max_objects_per_hand').value), 1)
        self.component_min_area_px = max(
            float(self.get_parameter('component_min_area_px').value), 1.0)
        self.enable_near_hand_objects = self._as_bool(
            self.get_parameter('enable_near_hand_objects').value)
        if self.enable_near_hand_objects and self.assist_mode != 'depth':
            self.get_logger().warn(
                'enable_near_hand_objects requires ZED depth; switching assist_mode to depth')
            self.assist_mode = 'depth'
        self.uses_depth_image = self.assist_mode == 'depth'

        self.publish_period_ns = int(1_000_000_000 / self.publish_fps)
        self.last_publish_time = None
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
        self.latest_base_image_topic = None
        self.latest_camera_info = None
        self.latest_intrinsics = None
        self.latest_camera_info_topic = None
        self.ok_overlay_until_s = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.depth_sub = None
        if self.uses_depth_image:
            self.depth_sub = self.create_subscription(
                Image, self.depth_topic, self._depth_callback, qos_profile_sensor_data)
        self.base_subs = [
            self.create_subscription(
                Image,
                topic,
                lambda msg, topic=topic: self._base_image_callback(msg, topic),
                qos_profile_sensor_data,
            )
            for topic in self.base_image_topics
        ]
        self.base_sub = self.base_subs[0] if self.base_subs else None
        self.info_subs = [
            self.create_subscription(
                CameraInfo,
                topic,
                lambda msg, topic=topic: self._camera_info_callback(msg, topic),
                qos_profile_sensor_data,
            )
            for topic in self.camera_info_topics
        ]
        self.assist_pub = self.create_publisher(
            CompressedImage, self.assist_topic, qos_profile_sensor_data)
        self.metrics_pub = self.create_publisher(String, self.metrics_topic, 10)
        self.stream_stats_pub = None
        if self.stream_stats_topic:
            self.stream_stats_pub = self.create_publisher(String, self.stream_stats_topic, 10)
        self.camera_perf_pub = None
        if self.camera_perf_topic:
            self.camera_perf_pub = self.create_publisher(String, self.camera_perf_topic, 10)
        self.operator_ok_sub = None
        if self.operator_ok_topic:
            self.operator_ok_sub = self.create_subscription(
                String, self.operator_ok_topic, self._operator_ok_callback, 10)

        self.get_logger().info(
            f'ZED arm-relative depth assist: mode={self.assist_mode}, '
            f'depth={self.depth_topic if self.uses_depth_image else "disabled"}, '
            f'base={self.base_image_topics}, info={self.camera_info_topics} '
            f'-> {self.assist_topic} at {self.publish_fps:.1f} fps, '
            f'stats={self.stream_stats_topic or "disabled"}')

    def _operator_ok_callback(self, msg):
        duration_s = self.ok_overlay_duration_s
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            payload = {}
        if isinstance(payload, dict):
            try:
                duration_s = max(float(payload.get('duration_s', duration_s)), 0.1)
            except (TypeError, ValueError):
                duration_s = self.ok_overlay_duration_s
        self.ok_overlay_until_s = time.time() + duration_s

    def _string_list(self, value):
        if value is None:
            return []
        if isinstance(value, str):
            return [value]
        try:
            return [str(item) for item in value]
        except TypeError:
            return [str(value)]

    def _unique_strings(self, values):
        result = []
        for value in values:
            text = str(value).strip()
            if text and text not in result:
                result.append(text)
        return result

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

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
        payload = {
            'stamp_sec': now.nanoseconds / 1e9,
            'name': self.stream_stats_name,
            'topic': self.assist_topic,
            'assist_mode': self.assist_mode,
            'base_image_topic': self.latest_base_image_topic,
            'base_image_topics': self.base_image_topics,
            'depth_input_hz': (
                self._sample_hz(self.depth_input_times) if self.uses_depth_image else None),
            'base_input_hz': self._sample_hz(self.base_input_times),
            'output_hz': self._sample_hz(self.output_times),
            'process_ms': self._clean_float(self.last_process_ms, 2),
            'jpeg_ms': self._clean_float(self.last_jpeg_ms, 2),
            'bytes': int(self.last_output_bytes),
            'subscriber_count': int(self.count_subscribers(self.assist_topic)),
            'drop_reason': self.last_drop_reason,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.camera_perf_pub.publish(msg)

    def _base_image_callback(self, msg, topic=None):
        self._record_event(self.base_input_times)
        if not self.uses_depth_image:
            now = self.get_clock().now()
            callback_start = time.perf_counter()
            if (self.last_publish_time is not None and
                    (now - self.last_publish_time).nanoseconds < self.publish_period_ns):
                self.last_drop_reason = 'throttle'
                self._maybe_publish_camera_perf(now)
                return
            self.last_publish_time = now

            image = self._image_to_bgr(msg)
            if image is None:
                self.last_drop_reason = 'invalid_base_image'
                self._maybe_publish_camera_perf(now)
                return
            self.latest_base_image = image
            self.latest_base_image_time = now
            self.latest_base_image_header = msg.header
            self.latest_base_image_topic = topic
            self._publish_tf_header_assist(msg, image, now, callback_start)
            return

        image = self._image_to_bgr(msg)
        if image is None:
            return
        self.latest_base_image = image
        self.latest_base_image_time = self.get_clock().now()
        self.latest_base_image_header = msg.header
        self.latest_base_image_topic = topic

    def _camera_info_callback(self, msg, topic):
        self.latest_camera_info = msg
        self.latest_camera_info_topic = topic
        self.latest_intrinsics = {
            'frame_id': msg.header.frame_id,
            'width': int(msg.width),
            'height': int(msg.height),
            'fx': float(msg.k[0]),
            'fy': float(msg.k[4]),
            'cx': float(msg.k[2]),
            'cy': float(msg.k[5]),
        }

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

        metrics, draw = self._arm_relative_metrics(depth_m, msg)
        self._publish_metrics(metrics)

        if self.count_subscribers(self.assist_topic) <= 0:
            self.last_drop_reason = 'no_subscribers'
            self.last_process_ms = (time.perf_counter() - callback_start) * 1000.0
            self._maybe_publish_camera_perf(now)
            return

        base = self._get_base_image(depth_m.shape, now)
        image = self._make_assist_image(metrics, draw, base)
        byte_count, jpeg_ms = self._publish_jpeg(msg.header, image)
        if byte_count > 0:
            self._record_event(self.output_times)
            self.last_output_bytes = byte_count
            self.last_jpeg_ms = jpeg_ms
            self.last_drop_reason = 'published'
        else:
            self.last_drop_reason = 'jpeg_failed'
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
            self._warn_throttled(f'unsupported ZED base image encoding: {msg.encoding}')
            return None
        if msg.step <= 0 or msg.height <= 0 or msg.width <= 0:
            self._warn_throttled('received malformed ZED base image')
            return None

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.step * msg.height
        if raw.size < expected:
            self._warn_throttled('ZED base image data is shorter than expected')
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
            self._warn_throttled(f'unsupported ZED depth encoding: {msg.encoding}')
            return None

        if msg.step <= 0 or msg.height <= 0 or msg.width <= 0:
            self._warn_throttled('received malformed ZED depth image')
            return None

        stride = msg.step // dtype.itemsize
        expected = stride * msg.height
        raw = np.frombuffer(msg.data, dtype=dtype)
        if raw.size < expected:
            self._warn_throttled('ZED depth image data is shorter than expected')
            return None

        image = raw[:expected].reshape((msg.height, stride))[:, :msg.width]
        return image.astype(np.float32, copy=False) * scale

    def _valid_mask(self, depth_m):
        return (
            np.isfinite(depth_m) &
            (depth_m >= self.min_depth_m) &
            (depth_m <= self.max_depth_m)
        )

    def _clean_float(self, value, digits=4):
        if value is None:
            return None
        value = float(value)
        if not math.isfinite(value):
            return None
        return round(value, digits)

    def _stamp_sec(self, stamp):
        return float(stamp.sec + stamp.nanosec / 1e9)

    def _scaled_intrinsics(self, depth_shape):
        if self.latest_intrinsics is None:
            return None
        if self.latest_intrinsics['fx'] <= 0.0 or self.latest_intrinsics['fy'] <= 0.0:
            return None
        height, width = depth_shape[:2]
        info_width = max(self.latest_intrinsics['width'], 1)
        info_height = max(self.latest_intrinsics['height'], 1)
        sx = float(width) / float(info_width)
        sy = float(height) / float(info_height)
        return {
            'fx': self.latest_intrinsics['fx'] * sx,
            'fy': self.latest_intrinsics['fy'] * sy,
            'cx': self.latest_intrinsics['cx'] * sx,
            'cy': self.latest_intrinsics['cy'] * sy,
            'width': width,
            'height': height,
            'frame_id': self.latest_intrinsics['frame_id'],
        }

    def _camera_frame_candidates(self, depth_msg):
        candidates = []
        for frame in (
            self.camera_optical_frame,
            self.latest_intrinsics.get('frame_id') if self.latest_intrinsics else '',
            depth_msg.header.frame_id,
            *self.camera_frame_fallbacks,
        ):
            frame = str(frame).strip()
            if frame and frame not in candidates:
                candidates.append(frame)
        return candidates

    def _tf_time(self, stamp):
        if self.use_latest_tf or (stamp.sec == 0 and stamp.nanosec == 0):
            return Time()
        return Time.from_msg(stamp)

    def _camera_frame_candidates_from_header(self, header):
        candidates = []
        for frame in (
            self.camera_optical_frame,
            self.latest_intrinsics.get('frame_id') if self.latest_intrinsics else '',
            getattr(header, 'frame_id', ''),
            *self.camera_frame_fallbacks,
        ):
            frame = str(frame).strip()
            if frame and frame not in candidates:
                candidates.append(frame)
        return candidates

    def _lookup_link_point(self, camera_frame, source_frame, stamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                source_frame,
                self._tf_time(stamp),
                timeout=self.tf_lookup_timeout,
            )
        except Exception:
            return None
        translation = transform.transform.translation
        return np.array([translation.x, translation.y, translation.z], dtype=np.float32)

    def _lookup_first_link_point(self, camera_frame, source_frames, stamp):
        for source_frame in source_frames:
            point = self._lookup_link_point(camera_frame, source_frame, stamp)
            if point is not None:
                return source_frame, point
        return '', None

    def _project_point(self, point, intrinsics):
        if point is None or not np.all(np.isfinite(point)) or float(point[2]) <= 0.02:
            return None
        u = intrinsics['fx'] * float(point[0]) / float(point[2]) + intrinsics['cx']
        v = intrinsics['fy'] * float(point[1]) / float(point[2]) + intrinsics['cy']
        if not math.isfinite(u) or not math.isfinite(v):
            return None
        return (int(round(u)), int(round(v)))

    def _point_in_image(self, pixel, shape):
        if pixel is None:
            return False
        height, width = shape[:2]
        return 0 <= pixel[0] < width and 0 <= pixel[1] < height

    def _collect_arm_projection(
            self, camera_frame, depth_msg, intrinsics, depth_shape, include_links=True):
        sides = {
            'left': {
                'label': 'L',
                'hand_frames': self.left_hand_frames,
                'links': self.left_arm_links,
                'color': (70, 220, 120),
            },
            'right': {
                'label': 'R',
                'hand_frames': self.right_hand_frames,
                'links': self.right_arm_links,
                'color': (255, 110, 220),
            },
        }
        projections = {}
        for side, config in sides.items():
            hand_frame, hand_point = self._lookup_first_link_point(
                camera_frame, config['hand_frames'], depth_msg.header.stamp)
            hand_pixel = self._project_point(hand_point, intrinsics)
            link_draw = []
            if hand_point is not None and include_links:
                for frame in config['links']:
                    point = self._lookup_link_point(camera_frame, frame, depth_msg.header.stamp)
                    pixel = self._project_point(point, intrinsics)
                    link_draw.append({
                        'frame': frame,
                        'point': point,
                        'pixel': pixel,
                        'visible': self._point_in_image(pixel, depth_shape),
                    })
            projections[side] = {
                'label': config['label'],
                'color': config['color'],
                'hand_frame': hand_frame,
                'hand_frame_candidates': config['hand_frames'],
                'hand_point': hand_point,
                'hand_pixel': hand_pixel,
                'hand_visible': self._point_in_image(hand_pixel, depth_shape),
                'links': link_draw,
            }
        return projections

    def _collect_hand_depths(self, camera_frame, stamp):
        configs = {
            'left': self.left_hand_frames,
            'right': self.right_hand_frames,
        }
        hands = {}
        for side, frames in configs.items():
            hand_frame, hand_point = self._lookup_first_link_point(camera_frame, frames, stamp)
            hand_depth = None
            hand_valid = False
            if (hand_point is not None and np.all(np.isfinite(hand_point)) and
                    float(hand_point[2]) > 0.02):
                hand_depth = self._clean_float(float(hand_point[2]))
                hand_valid = True
            hands[side] = {
                'valid': hand_valid,
                'frame': hand_frame,
                'frame_candidates': frames,
                'pixel': None,
                'visible': False,
                'hand_depth_m': hand_depth,
                'hand_point_m': (
                    [self._clean_float(value) for value in hand_point]
                    if hand_point is not None else None),
                'object_candidate_count': 0,
                'nearest_object': None,
                'objects': [],
            }
        return hands

    def _arm_relative_metrics_tf(self, header):
        stamp = header.stamp
        camera_candidates = self._camera_frame_candidates_from_header(header)
        last_error = ''
        for camera_frame in camera_candidates:
            hands = self._collect_hand_depths(camera_frame, stamp)
            left_ok = hands['left']['hand_depth_m'] is not None
            right_ok = hands['right']['hand_depth_m'] is not None
            if left_ok or right_ok:
                break
            last_error = f'no TF from {camera_frame} to hand frames'
        else:
            return {
                'stamp_sec': self._clean_float(self._stamp_sec(header.stamp), 6),
                'status': 'waiting_tf',
                'assist_mode': self.assist_mode,
                'camera_frame_candidates': camera_candidates,
                'message': last_error or 'no camera frame candidates',
            }, {
                'status_text': 'WAITING TF',
                'projections': {},
                'objects': {},
                'enable_near_hand_objects': False,
            }

        compare = self._compare_hands(
            hands['left']['hand_depth_m'],
            hands['right']['hand_depth_m'],
        )
        metrics = {
            'stamp_sec': self._clean_float(self._stamp_sec(header.stamp), 6),
            'status': 'ok',
            'assist_mode': self.assist_mode,
            'camera_frame': camera_frame,
            'camera_info_frame': (
                self.latest_intrinsics.get('frame_id') if self.latest_intrinsics else None),
            'camera_info_topic': self.latest_camera_info_topic,
            'hands': hands,
            'gripper_depth_compare': compare,
        }
        draw = {
            'status_text': '',
            'camera_frame': camera_frame,
            'projections': {},
            'objects': {},
            'enable_near_hand_objects': False,
        }
        return metrics, draw

    def _publish_tf_header_assist(self, source_msg, base_image, now, callback_start):
        metrics, draw = self._arm_relative_metrics_tf(source_msg.header)
        self._publish_metrics(metrics)

        if self.count_subscribers(self.assist_topic) <= 0:
            self.last_drop_reason = 'no_subscribers'
            self.last_process_ms = (time.perf_counter() - callback_start) * 1000.0
            self._maybe_publish_camera_perf(now)
            return

        image = self._make_assist_image(metrics, draw, base_image)
        byte_count, jpeg_ms = self._publish_jpeg(source_msg.header, image)
        if byte_count > 0:
            self._record_event(self.output_times)
            self.last_output_bytes = byte_count
            self.last_jpeg_ms = jpeg_ms
            self.last_drop_reason = 'published'
        else:
            self.last_drop_reason = 'jpeg_failed'
        self.last_process_ms = (time.perf_counter() - callback_start) * 1000.0
        self._maybe_publish_camera_perf(now)

    def _draw_robot_mask(self, shape, projections):
        mask = np.zeros(shape[:2], dtype=np.uint8)
        for projection in projections.values():
            visible_pixels = [
                item['pixel'] for item in projection['links']
                if item.get('visible')
            ]
            for pixel in visible_pixels:
                cv2.circle(mask, pixel, self.robot_mask_radius_px, 255, -1)
            for p0, p1 in zip(visible_pixels, visible_pixels[1:]):
                cv2.line(mask, p0, p1, 255, self.robot_mask_radius_px * 2)
            if projection.get('hand_visible'):
                cv2.circle(mask, projection['hand_pixel'], self.robot_mask_radius_px + 4, 255, -1)
        if self.robot_mask_dilate_px > 0 and np.any(mask):
            ksize = self.robot_mask_dilate_px * 2 + 1
            kernel = np.ones((ksize, ksize), np.uint8)
            mask = cv2.dilate(mask, kernel, iterations=1)
        return mask > 0

    def _pixel_points_for_mask(self, depth_m, mask, intrinsics):
        ys, xs = np.nonzero(mask)
        if xs.size == 0:
            return xs, ys, None
        z = depth_m[ys, xs].astype(np.float32, copy=False)
        x = (xs.astype(np.float32) - intrinsics['cx']) * z / intrinsics['fx']
        y = (ys.astype(np.float32) - intrinsics['cy']) * z / intrinsics['fy']
        points = np.column_stack((x, y, z))
        return xs, ys, points

    def _objects_near_hand(self, side_projection, depth_m, valid, robot_mask, intrinsics):
        if not side_projection.get('hand_visible') or side_projection.get('hand_point') is None:
            return [], None, 0

        height, width = depth_m.shape[:2]
        hx, hy = side_projection['hand_pixel']
        roi_mask = np.zeros(depth_m.shape, dtype=np.uint8)
        cv2.circle(roi_mask, (hx, hy), self.hand_roi_radius_px, 255, -1)
        candidate = (roi_mask > 0) & valid & (~robot_mask)
        if not np.any(candidate):
            return [], candidate, 0

        xs, ys, points = self._pixel_points_for_mask(depth_m, candidate, intrinsics)
        if points is None:
            return [], candidate, 0

        hand_point = side_projection['hand_point'].astype(np.float32, copy=False)
        distances = np.linalg.norm(points - hand_point.reshape(1, 3), axis=1)
        near_index = distances <= self.near_hand_radius_m
        near_mask = np.zeros(depth_m.shape, dtype=np.uint8)
        near_mask[ys[near_index], xs[near_index]] = 255
        if not np.any(near_mask):
            return [], near_mask > 0, 0

        kernel = np.ones((3, 3), np.uint8)
        near_mask = cv2.morphologyEx(near_mask, cv2.MORPH_OPEN, kernel)
        near_mask = cv2.morphologyEx(near_mask, cv2.MORPH_CLOSE, kernel)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            near_mask, connectivity=8)
        objects = []
        component_count = max(int(num_labels) - 1, 0)
        for label_id in range(1, num_labels):
            area = float(stats[label_id, cv2.CC_STAT_AREA])
            if area < self.component_min_area_px:
                continue
            component_mask = labels == label_id
            comp_xs, comp_ys, comp_points = self._pixel_points_for_mask(
                depth_m, component_mask, intrinsics)
            if comp_points is None:
                continue
            comp_distances = np.linalg.norm(comp_points - hand_point.reshape(1, 3), axis=1)
            depth_median = float(np.median(comp_points[:, 2]))
            point_median = np.median(comp_points, axis=0)
            contours, _ = cv2.findContours(
                component_mask.astype(np.uint8) * 255,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            contour = max(contours, key=cv2.contourArea) if contours else None
            cx = int(round(float(centroids[label_id][0])))
            cy = int(round(float(centroids[label_id][1])))
            depth_delta = depth_median - float(hand_point[2])
            distance_3d = float(np.median(comp_distances))
            objects.append({
                'valid': True,
                'cx_px': cx,
                'cy_px': cy,
                'area_px': int(area),
                'depth_m': self._clean_float(depth_median),
                'depth_delta_m': self._clean_float(depth_delta),
                'distance_3d_m': self._clean_float(distance_3d),
                'point_m': [self._clean_float(value) for value in point_median],
                'contour': contour,
            })

        objects.sort(key=lambda item: (
            float('inf') if item['distance_3d_m'] is None else item['distance_3d_m'],
            abs(float('inf') if item['depth_delta_m'] is None else item['depth_delta_m']),
        ))
        return objects[:self.max_objects_per_hand], near_mask > 0, component_count

    def _compare_hands(self, left_depth, right_depth):
        if left_depth is None or right_depth is None:
            return {
                'farther': 'unknown',
                'delta_m': None,
            }
        delta = float(right_depth) - float(left_depth)
        if abs(delta) < 0.02:
            farther = 'tie'
        else:
            farther = 'right' if delta > 0.0 else 'left'
        return {
            'farther': farther,
            'delta_m': self._clean_float(abs(delta)),
            'signed_right_minus_left_m': self._clean_float(delta),
        }

    def _arm_relative_metrics(self, depth_m, depth_msg):
        intrinsics = self._scaled_intrinsics(depth_m.shape)
        if intrinsics is None:
            return {
                'stamp_sec': self._clean_float(self._stamp_sec(depth_msg.header.stamp), 6),
                'status': 'waiting_camera_info',
                'camera_info_topics': self.camera_info_topics,
            }, {
                'status_text': 'WAITING CAMERA_INFO',
                'projections': {},
                'robot_mask': np.zeros(depth_m.shape, dtype=bool),
                'enable_near_hand_objects': self.enable_near_hand_objects,
            }

        camera_candidates = self._camera_frame_candidates(depth_msg)
        last_error = ''
        for camera_frame in camera_candidates:
            projections = self._collect_arm_projection(
                camera_frame, depth_msg, intrinsics, depth_m.shape,
                include_links=self.enable_near_hand_objects)
            left_ok = projections['left']['hand_point'] is not None
            right_ok = projections['right']['hand_point'] is not None
            if left_ok or right_ok:
                break
            last_error = f'no TF from {camera_frame} to hand frames'
        else:
            return {
                'stamp_sec': self._clean_float(self._stamp_sec(depth_msg.header.stamp), 6),
                'status': 'waiting_tf',
                'camera_frame_candidates': camera_candidates,
                'message': last_error or 'no camera frame candidates',
            }, {
                'status_text': 'WAITING TF',
                'projections': {},
                'robot_mask': np.zeros(depth_m.shape, dtype=bool),
                'enable_near_hand_objects': self.enable_near_hand_objects,
            }

        if self.enable_near_hand_objects:
            robot_mask = self._draw_robot_mask(depth_m.shape, projections)
            valid = self._valid_mask(depth_m)
        else:
            robot_mask = np.zeros(depth_m.shape, dtype=bool)
            valid = None
        hands = {}
        draw_objects = {}
        for side, projection in projections.items():
            if self.enable_near_hand_objects:
                objects, object_mask, object_candidate_count = self._objects_near_hand(
                    projection, depth_m, valid, robot_mask, intrinsics)
            else:
                objects = []
                object_mask = np.zeros(depth_m.shape, dtype=bool)
                object_candidate_count = 0
            nearest = objects[0] if objects else None
            hand_point = projection.get('hand_point')
            hand_depth = None
            hand_valid = False
            if hand_point is not None and math.isfinite(float(hand_point[2])) and float(hand_point[2]) > 0.02:
                hand_depth = self._clean_float(float(hand_point[2]))
                hand_valid = True
            hands[side] = {
                'valid': hand_valid,
                'frame': projection.get('hand_frame'),
                'frame_candidates': projection.get('hand_frame_candidates'),
                'pixel': self._pixel_dict(projection.get('hand_pixel')),
                'visible': bool(projection.get('hand_visible', False)),
                'hand_depth_m': hand_depth,
                'hand_point_m': [self._clean_float(value) for value in hand_point] if hand_point is not None else None,
                'object_candidate_count': object_candidate_count,
                'nearest_object': self._object_public(nearest),
                'objects': [self._object_public(obj) for obj in objects],
            }
            draw_objects[side] = {
                'mask': object_mask,
                'objects': objects,
            }

        compare = self._compare_hands(
            hands['left']['hand_depth_m'],
            hands['right']['hand_depth_m'],
        )
        metrics = {
            'stamp_sec': self._clean_float(self._stamp_sec(depth_msg.header.stamp), 6),
            'status': 'ok',
            'camera_frame': camera_frame,
            'camera_info_frame': intrinsics.get('frame_id'),
            'camera_info_topic': self.latest_camera_info_topic,
            'hands': hands,
            'gripper_depth_compare': compare,
        }
        draw = {
            'status_text': '',
            'camera_frame': camera_frame,
            'projections': projections,
            'robot_mask': robot_mask,
            'objects': draw_objects,
            'enable_near_hand_objects': self.enable_near_hand_objects,
        }
        return metrics, draw

    def _pixel_dict(self, pixel):
        if pixel is None:
            return None
        return {
            'x': int(pixel[0]),
            'y': int(pixel[1]),
        }

    def _object_public(self, obj):
        if not obj:
            return None
        return {
            'valid': bool(obj.get('valid', False)),
            'cx_px': obj.get('cx_px'),
            'cy_px': obj.get('cy_px'),
            'area_px': obj.get('area_px'),
            'depth_m': obj.get('depth_m'),
            'depth_delta_m': obj.get('depth_delta_m'),
            'distance_3d_m': obj.get('distance_3d_m'),
            'point_m': obj.get('point_m'),
        }

    def _get_base_image(self, depth_shape, now):
        if self.latest_base_image is None or self.latest_base_image_time is None:
            return np.zeros((depth_shape[0], depth_shape[1], 3), dtype=np.uint8)
        age_s = (now - self.latest_base_image_time).nanoseconds / 1e9
        if self.base_image_timeout_s > 0.0 and age_s > self.base_image_timeout_s:
            return np.zeros((depth_shape[0], depth_shape[1], 3), dtype=np.uint8)

        height, width = depth_shape[:2]
        base = self.latest_base_image
        if base.shape[0] != height or base.shape[1] != width:
            base = cv2.resize(base, (width, height), interpolation=cv2.INTER_LINEAR)
        return np.ascontiguousarray(base)

    def _format_depth(self, value):
        if value is None:
            return '--'
        return f'{float(value):.2f}'

    def _format_delta(self, value):
        if value is None:
            return '--'
        return f'{float(value):+.2f}'

    def _is_valid_depth(self, value):
        if value is None:
            return False
        return math.isfinite(float(value))

    def _hand_depth_header_state(self, left_depth, right_depth, compare):
        neutral = (32, 34, 36)
        same_green = (0, 150, 70)
        far_light_start = (72, 176, 255)
        far_light_end = (132, 222, 255)
        close_dark_start = (20, 138, 245)
        close_dark_end = (0, 82, 224)
        colors = {
            'left': neutral,
            'right': neutral,
        }
        labels = {
            'left': '--',
            'right': '--',
        }

        left_valid = self._is_valid_depth(left_depth)
        right_valid = self._is_valid_depth(right_depth)
        if left_valid:
            labels['left'] = 'DEPTH'
        if right_valid:
            labels['right'] = 'DEPTH'
        if not (left_valid and right_valid):
            return colors, labels

        farther = compare.get('farther')
        if farther == 'tie':
            colors['left'] = same_green
            colors['right'] = same_green
            labels['left'] = 'SAME DEPTH'
            labels['right'] = 'SAME DEPTH'
            return colors, labels

        intensity = self._depth_delta_intensity(compare)
        far_orange = self._lerp_bgr(far_light_start, far_light_end, intensity)
        close_orange = self._lerp_bgr(close_dark_start, close_dark_end, intensity)
        if farther == 'left':
            colors['left'] = far_orange
            colors['right'] = close_orange
            labels['left'] = 'FARTHER'
            labels['right'] = 'CLOSER'
            return colors, labels
        if farther == 'right':
            colors['left'] = close_orange
            colors['right'] = far_orange
            labels['left'] = 'CLOSER'
            labels['right'] = 'FARTHER'
            return colors, labels
        return colors, labels

    def _depth_delta_intensity(self, compare):
        delta = compare.get('delta_m')
        if delta is None:
            delta = compare.get('signed_right_minus_left_m')
        try:
            delta = abs(float(delta))
        except (TypeError, ValueError):
            return 0.0
        return float(np.clip((delta - 0.02) / (0.25 - 0.02), 0.0, 1.0))

    def _lerp_bgr(self, start, end, t):
        t = float(np.clip(t, 0.0, 1.0))
        return tuple(int(round(a + (b - a) * t)) for a, b in zip(start, end))

    def _assist_header_height(self, show_detail=False):
        return 112 if show_detail else 86

    def _compose_assist_canvas(self, camera_image, header_height):
        height, width = camera_image.shape[:2]
        header_height = max(int(header_height), 1)
        canvas_shape = (height + header_height, width) + camera_image.shape[2:]
        canvas = np.zeros(canvas_shape, dtype=camera_image.dtype)
        if canvas.ndim == 3 and canvas.shape[2] >= 3:
            canvas[:header_height, :] = (16, 18, 20)
        canvas[header_height:header_height + height, :] = camera_image
        return canvas

    def _draw_hand_depth_header(self, image, left, right, compare, header_height=86):
        height, width = image.shape[:2]
        header_height = min(max(int(header_height), 1), height)
        mid_x = width // 2
        left_depth = left.get('hand_depth_m')
        right_depth = right.get('hand_depth_m')
        colors, labels = self._hand_depth_header_state(left_depth, right_depth, compare)
        panels = [
            ('left', 'LEFT', (0, 0), (max(mid_x - 1, 0), header_height - 1)),
            ('right', 'RIGHT', (mid_x, 0), (width - 1, header_height - 1)),
        ]
        for side, title, top_left, bottom_right in panels:
            cv2.rectangle(image, top_left, bottom_right, colors[side], -1)
            text_x = top_left[0] + 8
            self._put_text(image, title, (text_x, 31), 0.70)
            self._put_text(image, labels[side], (text_x, 64), 0.52)
        cv2.line(image, (mid_x, 0), (mid_x, header_height - 1), (235, 235, 235), 1)
        cv2.rectangle(image, (0, 0), (width - 1, header_height - 1), (235, 235, 235), 1)

    def _make_assist_image(self, metrics, draw, base_image):
        camera_image = np.ascontiguousarray(base_image.copy())
        height, width = camera_image.shape[:2]

        status = metrics.get('status')
        if status != 'ok':
            status_text = draw.get('status_text') or str(status).upper()
            header_height = self._assist_header_height()
            image = self._compose_assist_canvas(camera_image, header_height)
            cv2.rectangle(image, (0, 0), (width - 1, header_height - 1), (16, 18, 20), -1)
            cv2.rectangle(image, (0, 0), (width - 1, header_height - 1), (0, 0, 255), 2)
            self._put_text(image, f'ZED ARM DEPTH: {status_text}', (8, 32), 0.72)
            message = str(metrics.get('message') or '')
            if message:
                self._put_text(image, message[:70], (8, 63), 0.48)
            self._draw_ok_overlay(image)
            return image

        projections = draw.get('projections') or {}
        show_near_hand_objects = bool(draw.get('enable_near_hand_objects', False))
        if show_near_hand_objects:
            for side in ('left', 'right'):
                projection = projections.get(side)
                if not projection:
                    continue
                color = projection['color']
                visible = [
                    item['pixel'] for item in projection.get('links', [])
                    if item.get('visible')
                ]
                for p0, p1 in zip(visible, visible[1:]):
                    cv2.line(camera_image, p0, p1, color, 4, cv2.LINE_AA)
                for point in visible:
                    cv2.circle(camera_image, point, 5, color, -1, cv2.LINE_AA)
                if projection.get('hand_visible'):
                    hand_pixel = projection['hand_pixel']
                    cv2.circle(camera_image, hand_pixel, self.hand_roi_radius_px, color, 1, cv2.LINE_AA)
                    cv2.circle(camera_image, hand_pixel, 9, color, -1, cv2.LINE_AA)
                    cv2.circle(camera_image, hand_pixel, 13, (255, 255, 255), 1, cv2.LINE_AA)

            object_draw = draw.get('objects') or {}
            for side, data in object_draw.items():
                color = (0, 255, 255) if side == 'left' else (0, 180, 255)
                for index, obj in enumerate(data.get('objects') or []):
                    contour = obj.get('contour')
                    if contour is not None:
                        thickness = 3 if index == 0 else 1
                        cv2.drawContours(camera_image, [contour], -1, color, thickness, cv2.LINE_AA)
                    center = (obj.get('cx_px'), obj.get('cy_px'))
                    if center[0] is not None and center[1] is not None:
                        cv2.circle(camera_image, center, 5, color, -1, cv2.LINE_AA)
                        label = 'L' if side == 'left' else 'R'
                        self._put_text(
                            camera_image,
                            f'{label} {self._format_delta(obj.get("depth_delta_m"))}m',
                            (center[0] + 8, max(center[1] - 8, 16)),
                            0.46,
                        )

        hands = metrics.get('hands') or {}
        left = hands.get('left') or {}
        right = hands.get('right') or {}
        compare = metrics.get('gripper_depth_compare') or {}
        header_height = self._assist_header_height(show_near_hand_objects)
        image = self._compose_assist_canvas(camera_image, header_height)
        self._draw_hand_depth_header(image, left, right, compare, header_height)
        if show_near_hand_objects:
            left_obj = left.get('nearest_object') or {}
            right_obj = right.get('nearest_object') or {}
            self._put_text(
                image,
                f'L OBJ-HAND {self._format_delta(left_obj.get("depth_delta_m"))}m '
                f'3D {self._format_depth(left_obj.get("distance_3d_m"))}m   '
                f'R OBJ-HAND {self._format_delta(right_obj.get("depth_delta_m"))}m '
                f'3D {self._format_depth(right_obj.get("distance_3d_m"))}m',
                (8, max(header_height - 14, 82)),
                0.44,
            )
        self._draw_ok_overlay(image)
        return image

    def _draw_ok_overlay(self, image):
        if time.time() >= self.ok_overlay_until_s:
            return
        height, width = image.shape[:2]
        overlay = image.copy()
        cv2.rectangle(overlay, (0, 0), (width - 1, height - 1), (20, 120, 45), -1)
        cv2.addWeighted(overlay, 0.42, image, 0.58, 0.0, image)

        text = 'OK'
        scale = max(min(width, height) / 130.0, 1.5)
        thickness = max(int(round(scale * 4.0)), 4)
        (text_w, text_h), baseline = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
        origin = (
            max((width - text_w) // 2, 0),
            max((height + text_h) // 2 - baseline, text_h),
        )
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            (0, 0, 0), thickness + 6, cv2.LINE_AA)
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
            (255, 255, 255), thickness, cv2.LINE_AA)

    def _put_text(self, image, text, origin, scale):
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (255, 255, 255), 1, cv2.LINE_AA)

    def _publish_metrics(self, metrics):
        msg = String()
        msg.data = json.dumps(metrics, sort_keys=True)
        self.metrics_pub.publish(msg)

    def _publish_jpeg(self, header, image):
        encode_start = time.perf_counter()
        ok, encoded = cv2.imencode(
            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        jpeg_ms = (time.perf_counter() - encode_start) * 1000.0
        if not ok:
            self._warn_throttled('failed to JPEG-encode ZED arm-relative depth assist')
            return 0, jpeg_ms
        msg = CompressedImage()
        msg.header = header
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        self.assist_pub.publish(msg)
        self._publish_stream_stats(image, len(msg.data))
        return len(msg.data), jpeg_ms

    def _publish_stream_stats(self, image, byte_count):
        if self.stream_stats_pub is None:
            return
        msg = String()
        msg.data = json.dumps({
            'stamp_sec': self.get_clock().now().nanoseconds / 1e9,
            'name': self.stream_stats_name,
            'topic': self.assist_topic,
            'bytes': int(byte_count),
            'width': int(image.shape[1]),
            'height': int(image.shape[0]),
            'jpeg_quality': int(self.jpeg_quality),
        }, sort_keys=True)
        self.stream_stats_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZedDepthAssist()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
