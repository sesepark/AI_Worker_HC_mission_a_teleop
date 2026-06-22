#!/usr/bin/env python3
"""OpenCV viewer for monitor OCR images and `/monitor_ocr/result`."""
from __future__ import annotations

import json
import threading

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from perception_nodes.monitor_ocr.ocr_pipeline_parts import find_display_parts


class MonitorOCRViewer(Node):
    def __init__(self) -> None:
        super().__init__("monitor_ocr_viewer")

        self.declare_parameter("image_topic", "/zed/zed_node/rgb/image_rect_color")
        self.declare_parameter("result_topic", "/monitor_ocr/result")

        image_topic = str(self.get_parameter("image_topic").value)
        result_topic = str(self.get_parameter("result_topic").value)

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.frame = None
        self.result = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, image_topic, self.image_callback, qos)
        self.create_subscription(String, result_topic, self.result_callback, 10)
        self.create_timer(1.0 / 30.0, self.render)

        self.get_logger().info(
            f"Monitor OCR viewer ready. image_topic={image_topic}, "
            f"result_topic={result_topic}"
        )

    def image_callback(self, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"cv_bridge conversion failed: {exc}")
            return

        with self.lock:
            self.frame = image

    def result_callback(self, msg: String) -> None:
        try:
            result = json.loads(msg.data)
        except Exception:
            return

        with self.lock:
            self.result = result

    def render(self) -> None:
        with self.lock:
            frame = self.frame.copy() if self.frame is not None else None
            result = dict(self.result) if isinstance(self.result, dict) else None

        if frame is None:
            return

        cv2.imshow("Monitor OCR Viewer", self.draw(frame, result))
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Viewer shutdown requested.")
            cv2.destroyAllWindows()
            rclpy.shutdown()
        elif key == ord("s"):
            import time

            path = f"/tmp/monitor_ocr_capture_{int(time.time())}.png"
            cv2.imwrite(path, frame)
            self.get_logger().info(f"Saved {path}")

    def draw(self, frame, result):
        image = frame.copy()

        bbox = result.get("bbox") if result else None
        if not bbox:
            bbox = find_display_parts(frame)

        if bbox:
            x, y, w, h = [int(v) for v in bbox]
            detected = bool(
                result
                and result.get("latest_screen_detected", result.get("screen_detected", False))
            )
            color = (0, 255, 0) if detected else (0, 165, 255)
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            cv2.putText(
                image,
                "MONITOR",
                (x, max(y - 6, 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
            )

        if result:
            parts = result.get("parts", []) or []
            lines = [
                f"[{result.get('frames_used', '-')}F] "
                f"{result.get('latest_elapsed_ms', '-')}ms",
                f"screen: {result.get('latest_screen_detected', False)}",
                "",
            ]
            lines.extend(
                f"{item.get('name', '-')}: {item.get('count', '-')}"
                for item in parts
                if isinstance(item, dict)
            )

            pad = 8
            line_height = 24
            panel_width = 360
            panel_height = len(lines) * line_height + pad * 2
            x0 = max(10, frame.shape[1] - panel_width - 10)
            y0 = 10

            overlay = image.copy()
            cv2.rectangle(
                overlay,
                (x0, y0),
                (x0 + panel_width, y0 + panel_height),
                (0, 0, 0),
                -1,
            )
            cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
            cv2.rectangle(
                image,
                (x0, y0),
                (x0 + panel_width, y0 + panel_height),
                (200, 200, 200),
                1,
            )

            for index, line in enumerate(lines):
                if not line:
                    continue
                cv2.putText(
                    image,
                    line,
                    (x0 + pad, y0 + pad + line_height * (index + 1) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

        return image


def main(args=None):
    rclpy.init(args=args)
    node = MonitorOCRViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
