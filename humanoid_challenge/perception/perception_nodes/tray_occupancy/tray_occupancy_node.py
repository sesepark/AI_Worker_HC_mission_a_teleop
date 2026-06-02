#!/usr/bin/env python3
from __future__ import annotations

import os
import threading
import time
from contextlib import suppress
from typing import Sequence, Tuple

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from mission_interfaces.msg import Detection2D
from mission_interfaces.srv import GetTrayDetections
from perception.msg import PartDetection, PartDetectionArray


Point = Tuple[float, float]


def default_tray_model_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory("perception"),
            "model",
            "tray_occupancy_best.pt",
        )
    except Exception:
        return os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "..",
                "model",
                "tray_occupancy_best.pt",
            )
        )


def point_in_bbox(point: Point, bbox: Sequence[int], margin_px: float) -> bool:
    if len(bbox) < 4:
        return False
    x, y = point
    x1, y1, x2, y2 = bbox[:4]
    return (x1 - margin_px) <= x <= (x2 + margin_px) and (y1 - margin_px) <= y <= (y2 + margin_px)


def point_in_polygon(point: Point, xs: Sequence[float], ys: Sequence[float]) -> bool:
    if len(xs) < 3 or len(xs) != len(ys):
        return False

    x, y = point
    inside = False
    j = len(xs) - 1
    for i in range(len(xs)):
        xi, yi = xs[i], ys[i]
        xj, yj = xs[j], ys[j]
        crosses = (yi > y) != (yj > y)
        if crosses:
            x_at_y = (xj - xi) * (y - yi) / ((yj - yi) or 1e-9) + xi
            if x < x_at_y:
                inside = not inside
        j = i
    return inside


class TrayOccupancyNode(Node):
    def __init__(self) -> None:
        super().__init__("tray_occupancy_node")

        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("image_topic", "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("tray_detection_service_name", "/mission_a/tray_detections")
        self.declare_parameter("tray_detection_service_timeout_sec", 3.0)
        self.declare_parameter("tray_detection_service_frame_count", 1)
        self.declare_parameter(
            "tray_model_path",
            os.environ.get(
                "TRAY_MODEL_PATH",
                default_tray_model_path(),
            ),
        )
        self.declare_parameter("tray_conf_threshold", 0.50)
        self.declare_parameter("tray_iou_threshold", 0.35)
        self.declare_parameter("tray_imgsz", 640)
        self.declare_parameter("tray_max_age_sec", 1.0)
        self.declare_parameter("tray_process_interval_sec", 0.10)
        self.declare_parameter("part_min_confidence", 0.30)
        self.declare_parameter("bbox_margin_px", 0.0)
        self.declare_parameter("source_camera_filter", "")
        self.declare_parameter("use_bottom_center", True)

        detections_topic = str(self.get_parameter("detections_topic").value)
        image_topic = str(self.get_parameter("image_topic").value)
        self.service_name = str(self.get_parameter("tray_detection_service_name").value)
        self.service_timeout_sec = float(
            self.get_parameter("tray_detection_service_timeout_sec").value)
        self.service_frame_count = int(
            self.get_parameter("tray_detection_service_frame_count").value)
        tray_model_path = str(self.get_parameter("tray_model_path").value)
        self.tray_conf_threshold = float(self.get_parameter("tray_conf_threshold").value)
        self.tray_iou_threshold = float(self.get_parameter("tray_iou_threshold").value)
        self.tray_imgsz = int(self.get_parameter("tray_imgsz").value)
        self.tray_max_age_sec = float(self.get_parameter("tray_max_age_sec").value)
        self.tray_process_interval_sec = float(self.get_parameter("tray_process_interval_sec").value)
        self.part_min_confidence = float(self.get_parameter("part_min_confidence").value)
        self.bbox_margin_px = float(self.get_parameter("bbox_margin_px").value)
        self.source_camera_filter = str(self.get_parameter("source_camera_filter").value)
        self.use_bottom_center = bool(self.get_parameter("use_bottom_center").value)

        self.latest_trays = []
        self.latest_tray_frame_id = ""
        self.latest_tray_stamp = None
        self.latest_tray_wall_time = 0.0
        self.latest_detections: list[PartDetection] = []
        self.latest_detection_header = None
        self.last_tray_process_time = 0.0
        self.force_next_image = False

        self.state_lock = threading.Lock()
        self.result_condition = threading.Condition()
        self.request_guard = threading.Lock()
        self.request_active = False
        self.request_frames = 0
        self.latest_result = None
        self.latest_result_seq = 0

        from cv_bridge import CvBridge
        from ultralytics import YOLO

        self.get_logger().info(f"Loading tray YOLO model: {tray_model_path}")
        self.bridge = CvBridge()
        self.tray_model = YOLO(tray_model_path)

        self.service = self.create_service(
            GetTrayDetections,
            self.service_name,
            self.handle_get_tray_detections,
            callback_group=ReentrantCallbackGroup(),
        )
        self.sub = self.create_subscription(
            PartDetectionArray,
            detections_topic,
            self.detections_callback,
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"TrayOccupancyNode ready. detections_topic={detections_topic}, "
            f"image_topic={image_topic}, service={self.service_name}"
        )

    def image_callback(self, msg: Image) -> None:
        with self.result_condition:
            if not self.request_active:
                return

        now = self.get_clock().now().nanoseconds * 1e-9
        with self.state_lock:
            force_next_image = self.force_next_image
            if not force_next_image and now - self.last_tray_process_time < self.tray_process_interval_sec:
                return
            self.last_tray_process_time = now
            self.force_next_image = False

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            result = self.tray_model.predict(
                img,
                conf=self.tray_conf_threshold,
                iou=self.tray_iou_threshold,
                imgsz=self.tray_imgsz,
                verbose=False,
            )[0]
        except Exception as exc:
            self.get_logger().warn(f"Tray YOLO inference failed: {exc}")
            return

        trays = []
        boxes = result.boxes if result.boxes is not None else []
        masks = result.masks.xy if result.masks is not None else None
        model_names = getattr(self.tray_model, "names", {}) or {}

        for idx, box in enumerate(boxes):
            cls = int(box.cls.item()) if hasattr(box.cls, "item") else int(box.cls)
            conf = float(box.conf.item()) if hasattr(box.conf, "item") else float(box.conf)
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            mask_x = []
            mask_y = []

            if masks is not None and idx < len(masks):
                mask_xy = masks[idx]
                if len(mask_xy) >= 3:
                    mask_x = [float(p[0]) for p in mask_xy]
                    mask_y = [float(p[1]) for p in mask_xy]

            trays.append(
                {
                    "class_id": cls,
                    "class_name": self.class_name(model_names, cls),
                    "confidence": conf,
                    "bbox": [x1, y1, x2, y2],
                    "mask_x": mask_x,
                    "mask_y": mask_y,
                }
            )

        with self.state_lock:
            self.latest_trays = trays
            self.latest_tray_frame_id = msg.header.frame_id
            self.latest_tray_stamp = msg.header.stamp
            self.latest_tray_wall_time = now

    def detections_callback(self, msg: PartDetectionArray) -> None:
        detections = [
            d for d in msg.detections
            if float(d.confidence) >= self.part_min_confidence
            and (not self.source_camera_filter or d.source_camera == self.source_camera_filter)
        ]

        with self.state_lock:
            self.latest_detections = detections
            self.latest_detection_header = msg.header

        self.maybe_store_observation()

    def maybe_store_observation(self) -> None:
        with self.result_condition:
            if not self.request_active:
                return

        with self.state_lock:
            trays, tray_frame_id, tray_stamp = self.current_tray_snapshot_locked()
            detection_header = self.latest_detection_header
            parts_in_tray = self.parts_inside_tray_locked(trays, self.latest_detections)

        tray_msgs = [
            self.tray_to_msg(tray, tray_frame_id, tray_stamp)
            for tray in trays
        ]
        part_msgs = [
            self.part_to_msg(part, detection_header)
            for part in parts_in_tray
        ]

        self.store_observation({
            "tray_detected": bool(tray_msgs),
            "trays": tray_msgs,
            "parts": part_msgs,
        })

    def current_tray_snapshot_locked(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.latest_tray_wall_time <= 0.0:
            return [], self.latest_tray_frame_id, self.latest_tray_stamp
        if now - self.latest_tray_wall_time > self.tray_max_age_sec:
            return [], self.latest_tray_frame_id, self.latest_tray_stamp
        return self.latest_trays, self.latest_tray_frame_id, self.latest_tray_stamp

    def parts_inside_tray_locked(self, trays, detections: list[PartDetection]) -> list[PartDetection]:
        if not trays:
            return []

        parts = []
        for det in detections:
            point = self.part_point(det)
            if any(self.point_inside_tray(point, tray) for tray in trays):
                parts.append(det)
        return parts

    def part_point(self, det: PartDetection) -> Point:
        if self.use_bottom_center and len(det.bbox) >= 4:
            x1, _, x2, y2 = det.bbox[:4]
            return (float(x1 + x2) / 2.0, float(y2))
        return (float(det.center_x), float(det.center_y))

    def point_inside_tray(self, point: Point, tray) -> bool:
        if point_in_polygon(point, tray["mask_x"], tray["mask_y"]):
            return True
        return point_in_bbox(point, tray["bbox"], self.bbox_margin_px)

    def store_observation(self, result: dict) -> None:
        with self.result_condition:
            if not self.request_active:
                return
            self.request_frames += 1
            result["frames_used"] = self.request_frames
            self.latest_result = result
            self.latest_result_seq += 1
            self.result_condition.notify_all()

    def handle_get_tray_detections(self, request, response):
        with self.request_guard:
            with self.result_condition:
                if self.request_active:
                    return self.fill_response(
                        response,
                        None,
                        False,
                        "another tray detection request is already running",
                    )
                self.request_active = True
                self.request_frames = 0
                self.latest_result = None
                start_seq = self.latest_result_seq

        with self.state_lock:
            self.latest_trays = []
            self.latest_tray_frame_id = ""
            self.latest_tray_stamp = None
            self.latest_tray_wall_time = 0.0
            self.last_tray_process_time = 0.0
            self.force_next_image = True

        latest_seen = None
        try:
            timeout_sec = float(request.timeout_sec)
            if timeout_sec <= 0.0:
                timeout_sec = self.service_timeout_sec

            frame_count = int(request.frame_count)
            if frame_count <= 0:
                frame_count = self.service_frame_count

            deadline = time.monotonic() + max(0.1, timeout_sec)
            while time.monotonic() < deadline:
                with self.result_condition:
                    remaining = max(0.0, deadline - time.monotonic())
                    self.result_condition.wait(timeout=min(0.2, remaining))
                    if self.latest_result_seq <= start_seq:
                        continue
                    latest_seen = self.latest_result

                frames_used = int(latest_seen.get("frames_used", 0) or 0)
                if frames_used >= frame_count:
                    success = bool(latest_seen.get("tray_detected", False))
                    message = "tray detections ready" if success else "tray not detected"
                    return self.fill_response(response, latest_seen, success, message)

            return self.fill_response(
                response,
                latest_seen,
                False,
                f"tray detection timed out before collecting {frame_count} frames",
            )
        finally:
            with self.result_condition:
                self.request_active = False
                self.result_condition.notify_all()
            with self.state_lock:
                self.force_next_image = False

    @staticmethod
    def fill_response(response, result: dict | None, success: bool, message: str):
        response.success = bool(success)
        response.message = message

        if result is None:
            response.tray_detected = False
            response.frames_used = 0
            response.trays = []
            response.parts = []
            return response

        response.tray_detected = bool(result.get("tray_detected", False))
        response.frames_used = int(result.get("frames_used", 0) or 0)
        response.trays = list(result.get("trays", []) or [])
        response.parts = list(result.get("parts", []) or [])
        return response

    @staticmethod
    def tray_to_msg(tray, frame_id: str, stamp) -> Detection2D:
        msg = Detection2D()
        msg.header.frame_id = frame_id or ""
        if stamp is not None:
            msg.header.stamp = stamp
        msg.class_id = int(tray["class_id"])
        msg.class_name = str(tray["class_name"])
        msg.confidence = float(tray["confidence"])
        msg.bbox = [int(v) for v in tray["bbox"]]
        msg.source = "tray_yolo"
        x1, y1, x2, y2 = msg.bbox[:4]
        msg.center_x = float(x1 + x2) / 2.0
        msg.center_y = float(y1 + y2) / 2.0
        msg.mask_x = [float(v) for v in tray["mask_x"]]
        msg.mask_y = [float(v) for v in tray["mask_y"]]
        return msg

    @staticmethod
    def part_to_msg(det: PartDetection, header) -> Detection2D:
        msg = Detection2D()
        if header is not None:
            msg.header = header
        msg.class_id = int(det.class_id)
        msg.class_name = str(det.class_name)
        msg.confidence = float(det.confidence)
        msg.bbox = [int(v) for v in det.bbox]
        msg.source = str(det.source_camera)
        msg.center_x = float(det.center_x)
        msg.center_y = float(det.center_y)
        msg.mask_x = [float(v) for v in det.mask_x]
        msg.mask_y = [float(v) for v in det.mask_y]
        return msg

    @staticmethod
    def class_name(model_names, cls: int) -> str:
        if isinstance(model_names, dict):
            return str(model_names.get(cls, f"class_{cls}"))
        if isinstance(model_names, (list, tuple)) and cls < len(model_names):
            return str(model_names[cls])
        return f"class_{cls}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrayOccupancyNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        with suppress(Exception):
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
