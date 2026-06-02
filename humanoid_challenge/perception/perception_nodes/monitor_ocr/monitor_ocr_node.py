#!/usr/bin/env python3
"""
ROS2 노드: 카메라 이미지 → 대시보드 OCR → task list 서비스 응답

Subscribe:
  /<image_topic>  (sensor_msgs/Image)

Service:
  /mission_a/task_list  (mission_interfaces/srv/GetTaskList)

Parameters:
  image_topic      (str,   default='/zed/zed_node/rgb/image_rect_color')
  process_interval (float, default=2.0)  OCR 최소 주기 (초)
"""
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from perception_nodes.monitor_ocr.paddle_ocr import make_ocr

from perception_nodes.monitor_ocr.ocr_pipeline_parts import (
    default_yolo_model_path,
    load_yolo_model,
    process_frame_parts,
)
from perception_nodes.monitor_ocr.frame_aggregator import FrameAggregatorParts

from mission_interfaces.msg import TaskItem
from mission_interfaces.srv import GetTaskList


class MonitorOCRNode(Node):

    def __init__(self):
        super().__init__('monitor_ocr_node')

        # 파라미터
        self.declare_parameter('image_topic',      '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('process_interval', 2.0)
        self.declare_parameter('task_list_service_name', '/mission_a/task_list')
        self.declare_parameter('task_list_service_timeout_sec', 20.0)
        self.declare_parameter('task_list_service_frame_count', 3)

        image_topic           = self.get_parameter('image_topic').value
        self.process_interval = self.get_parameter('process_interval').value
        self._task_list_service_name = self.get_parameter('task_list_service_name').value
        self._task_list_service_timeout_sec = float(
            self.get_parameter('task_list_service_timeout_sec').value)
        self._task_list_service_frame_count = int(
            self.get_parameter('task_list_service_frame_count').value)

        self.declare_parameter('yolo_model_path', default_yolo_model_path())
        yolo_path = str(self.get_parameter('yolo_model_path').value)
        try:
            loaded_yolo_path = load_yolo_model(yolo_path)
            if loaded_yolo_path:
                self.get_logger().info(f'YOLO 모델 로드 완료: {loaded_yolo_path}')
            else:
                self.get_logger().warn(
                    f'YOLO 모델 파일 없음: {yolo_path}; 이미지 기반 테이블 감지 사용')
        except Exception as exc:
            self.get_logger().warn(
                f'YOLO 모델 로드 실패: {exc}; 이미지 기반 테이블 감지 사용')

        # PaddleOCR 초기화 (시간이 걸리므로 먼저 로그)
        self.get_logger().info('PaddleOCR 초기화 중...')
        self.ocr_en  = make_ocr('en',     det_thresh=0.08, det_box_thresh=0.15, det_unclip=3.0)
        self.get_logger().info('PaddleOCR 초기화 완료')

        self.bridge      = CvBridge()
        self._aggregator = FrameAggregatorParts(window=10)
        self._lock           = threading.Lock()
        self._aggregator_lock = threading.Lock()
        self._pending_img    = None
        self._processing     = False
        self._last_proc_time = 0.0
        self._force_next_frame = False
        self._result_condition = threading.Condition()
        self._latest_result = None
        self._latest_result_seq = 0
        self._task_list_request_active = False
        self._task_list_request_lock = threading.Lock()

        self._task_list_service = self.create_service(
            GetTaskList,
            self._task_list_service_name,
            self._handle_get_task_list,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            f'GetTaskList service ready: {self._task_list_service_name}')

        # Subscriber (BEST_EFFORT: 드롭 허용, 항상 최신 프레임만)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.sub = self.create_subscription(Image, image_topic, self._image_cb, sub_qos)

        # 백그라운드 OCR 스레드
        self._ocr_thread = threading.Thread(target=self._ocr_worker, daemon=True)
        self._ocr_thread.start()

        self.get_logger().info(f'구독 토픽: {image_topic}')
        self.get_logger().info(f'OCR 주기: {self.process_interval}s')
        self.get_logger().info('모드: 부품 수량 테이블')

    # ── 콜백 ─────────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        """이미지 수신 콜백 - 스로틀링 후 pending에 저장."""
        now = time.time()
        with self._lock:
            force_next_frame = self._force_next_frame
            should_process = self._task_list_request_active

        if not should_process:
            return

        if not force_next_frame and now - self._last_proc_time < self.process_interval:
            return
        if self._processing:
            return

        try:
            # bgra8(ZED) 또는 bgr8 모두 BGR로 변환
            encoding = msg.encoding
            if encoding in ('bgra8', 'rgba8'):
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 변환 실패: {e}')
            return

        with self._lock:
            if not self._task_list_request_active:
                return
            self._pending_img = img
            if force_next_frame:
                self._force_next_frame = False

    # ── OCR 워커 스레드 ──────────────────────────────────────────────────────

    def _ocr_worker(self):
        while rclpy.ok():
            img = None
            with self._lock:
                if self._pending_img is not None:
                    img = self._pending_img
                    self._pending_img = None

            if img is not None:
                self._processing     = True
                self._last_proc_time = time.time()
                try:
                    raw = process_frame_parts(self.ocr_en, img)
                    with self._aggregator_lock:
                        result = self._aggregator.update(raw)
                    self._store_latest_result(result)
                    parts_log = "  ".join(
                        f"{p['name']}:{p['count']}" for p in result['parts'])
                    self.get_logger().info(
                        f"[부품] {parts_log}  {raw['elapsed_ms']}ms"
                        f"  ({result['frames_used']}프레임 집계)")
                except Exception as e:
                    self.get_logger().error(f'OCR 처리 실패: {e}')
                finally:
                    self._processing = False
            else:
                time.sleep(0.05)

    def _store_latest_result(self, result: dict) -> None:
        with self._result_condition:
            self._latest_result = result
            self._latest_result_seq += 1
            self._result_condition.notify_all()

    @staticmethod
    def _parts_payload(result: dict) -> tuple[list[str], list[int]]:
        names = []
        counts = []
        for item in result.get('parts', []) or []:
            if not isinstance(item, dict):
                continue
            names.append(str(item.get('name', '')))
            try:
                counts.append(int(item.get('count', -1)))
            except (TypeError, ValueError):
                counts.append(-1)
        return names, counts

    @staticmethod
    def _all_counts_recognized(result: dict | None) -> bool:
        if result is None:
            return False
        names, counts = MonitorOCRNode._parts_payload(result)
        return bool(names) and all(count >= 0 for count in counts)

    @staticmethod
    def _fill_task_list_response(response, result: dict | None, success: bool, message: str):
        response.success = bool(success)
        response.message = message

        if result is None:
            response.screen_detected = False
            response.all_counts_recognized = False
            response.frames_used = 0
            response.parts = []
            return response

        names, counts = MonitorOCRNode._parts_payload(result)
        response.screen_detected = bool(result.get('latest_screen_detected', False))
        response.all_counts_recognized = MonitorOCRNode._all_counts_recognized(result)
        response.frames_used = int(result.get('frames_used', 0) or 0)
        response.parts = [
            TaskItem(name=name, count=count)
            for name, count in zip(names, counts)
        ]
        return response

    def _handle_get_task_list(self, request, response):
        with self._task_list_request_lock:
            if self._task_list_request_active:
                return self._fill_task_list_response(
                    response,
                    None,
                    False,
                    'another task_list request is already running',
                )
            self._task_list_request_active = True

        latest_seen = None
        try:
            timeout_sec = float(request.timeout_sec)
            if timeout_sec <= 0.0:
                timeout_sec = self._task_list_service_timeout_sec

            frame_count = int(request.frame_count)
            if frame_count <= 0:
                frame_count = self._task_list_service_frame_count

            with self._result_condition:
                start_seq = self._latest_result_seq
                if hasattr(self._aggregator, 'reset'):
                    with self._aggregator_lock:
                        self._aggregator.reset()

            with self._lock:
                self._pending_img = None
                self._force_next_frame = True

            deadline = time.monotonic() + max(0.1, timeout_sec)
            while time.monotonic() < deadline:
                with self._result_condition:
                    remaining = max(0.0, deadline - time.monotonic())
                    self._result_condition.wait(timeout=min(0.2, remaining))
                    if self._latest_result_seq <= start_seq:
                        continue
                    latest_seen = self._latest_result

                frames_used = int(latest_seen.get('frames_used', 0) or 0)
                if frames_used >= frame_count:
                    all_counts_recognized = self._all_counts_recognized(latest_seen)
                    screen_detected = bool(latest_seen.get('latest_screen_detected', False))
                    success = bool(screen_detected and all_counts_recognized)
                    message = (
                        'task list ready' if success
                        else 'task list OCR completed with unrecognized counts'
                    )
                    return self._fill_task_list_response(
                        response, latest_seen, success, message)

            return self._fill_task_list_response(
                response,
                latest_seen,
                False,
                f'task_list OCR timed out before collecting {frame_count} frames',
            )
        finally:
            with self._lock:
                self._force_next_frame = False
                self._pending_img = None
            with self._task_list_request_lock:
                self._task_list_request_active = False


# ─── 진입점 ──────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MonitorOCRNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
