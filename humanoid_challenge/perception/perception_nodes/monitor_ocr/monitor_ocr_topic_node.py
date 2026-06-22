#!/usr/bin/env python3
"""
ROS2 노드: 카메라 이미지 → 대시보드 OCR → /monitor_ocr/result 토픽 발행

검증된 토픽 파이프라인(monitor_ocr → tray_manage_node)을 위한 토픽 발행 노드.
기존 서비스 노드(monitor_ocr_node.py, GetTaskList)와 병행 사용하며, 동일한 공용
OCR 파이프라인(process_frame_parts + FrameAggregatorParts)을 재사용한다.

Subscribe:
  /<image_topic>  (sensor_msgs/Image)

Publish:
  /monitor_ocr/result  (std_msgs/String, JSON)
    {"frames_used": int, "parts": [{"name": str, "count": int}, ...],
     "latest_elapsed_ms": float, "latest_screen_detected": bool}

Parameters:
  image_topic      (str,   default='/zed/zed_node/rgb/image_rect_color')
  result_topic     (str,   default='/monitor_ocr/result')
  process_interval (float, default=2.0)  OCR 최소 주기 (초)
  yolo_model_path  (str,   default=perception/model/monitor_ocr_best.pt)
  aggregator_window(int,   default=10)
"""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from perception_nodes.monitor_ocr.paddle_ocr import make_ocr
from perception_nodes.monitor_ocr.ocr_pipeline_parts import (
    default_yolo_model_path,
    load_yolo_model,
    process_frame_parts,
)
from perception_nodes.monitor_ocr.frame_aggregator import FrameAggregatorParts


class MonitorOCRTopicNode(Node):

    def __init__(self):
        super().__init__('monitor_ocr_topic_node')

        self.declare_parameter('image_topic',      '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('result_topic',     '/monitor_ocr/result')
        self.declare_parameter('process_interval', 2.0)
        self.declare_parameter('aggregator_window', 10)

        image_topic           = str(self.get_parameter('image_topic').value)
        result_topic          = str(self.get_parameter('result_topic').value)
        self.process_interval = float(self.get_parameter('process_interval').value)
        aggregator_window     = int(self.get_parameter('aggregator_window').value)

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

        self.get_logger().info('PaddleOCR 초기화 중...')
        self.ocr_en = make_ocr('en', det_thresh=0.08, det_box_thresh=0.15, det_unclip=3.0)
        self.get_logger().info('PaddleOCR 초기화 완료')

        self.bridge = CvBridge()
        self._aggregator = FrameAggregatorParts(window=aggregator_window)
        self._lock = threading.Lock()
        self._pending_img = None
        self._processing = False
        self._last_proc_time = 0.0

        self.pub = self.create_publisher(String, result_topic, 10)

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.sub = self.create_subscription(Image, image_topic, self._image_cb, sub_qos)

        self._ocr_thread = threading.Thread(target=self._ocr_worker, daemon=True)
        self._ocr_thread.start()

        self.get_logger().info(
            f'MonitorOCRTopicNode ready. image_topic={image_topic}, '
            f'result_topic={result_topic}, process_interval={self.process_interval}s')

    def _image_cb(self, msg: Image):
        now = time.time()
        with self._lock:
            if now - self._last_proc_time < self.process_interval:
                return
            if self._processing:
                return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 변환 실패: {e}')
            return

        with self._lock:
            self._pending_img = img

    def _ocr_worker(self):
        while rclpy.ok():
            img = None
            with self._lock:
                if self._pending_img is not None:
                    img = self._pending_img
                    self._pending_img = None

            if img is None:
                time.sleep(0.05)
                continue

            self._processing = True
            self._last_proc_time = time.time()
            try:
                raw = process_frame_parts(self.ocr_en, img)
                result = self._aggregator.update(raw)
                out = String()
                out.data = json.dumps(result, ensure_ascii=False, default=int)
                self.pub.publish(out)
                parts_log = "  ".join(
                    f"{p['name']}:{p['count']}" for p in result.get('parts', []))
                self.get_logger().info(
                    f"[부품] {parts_log}  {raw.get('elapsed_ms')}ms"
                    f"  ({result.get('frames_used')}프레임 집계)")
            except Exception as e:
                self.get_logger().error(f'OCR 처리 실패: {e}')
            finally:
                self._processing = False


def main(args=None):
    rclpy.init(args=args)
    node = MonitorOCRTopicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
