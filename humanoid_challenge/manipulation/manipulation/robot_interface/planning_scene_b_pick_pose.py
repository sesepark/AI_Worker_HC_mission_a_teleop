from __future__ import annotations

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)

from manipulation.robot_interface.planning_scene_b_pick import (
    FRAME_ID,
    ZONE_B_BOX_POSITION,
    clear_all_objects,
    get_zone_b_markers,
    remove_zone_b,
    setup_zone_b,
)


PACKAGE_NAME = "manipulation"
PACKAGE_SHARE_DIR = get_package_share_directory(PACKAGE_NAME)
ZONE_B_PICK_POSE_CONFIG_PATH = os.path.join(
    PACKAGE_SHARE_DIR,
    "config",
    "zone_b_pick_pose.yaml",
)


def _load_pick_pose_config() -> dict:
    with open(ZONE_B_PICK_POSE_CONFIG_PATH, "r") as f:
        return yaml.safe_load(f)["zone_b_pick_pose"]


_CFG = _load_pick_pose_config()
_POINT = _CFG["point"]

ZONE_B_PICK_POSE_POINT_POSITION = (
    float(ZONE_B_BOX_POSITION[0]),
    float(_POINT["y"]),
    float(ZONE_B_BOX_POSITION[2]),
)
ZONE_B_PICK_POSE_POINT_DIAMETER = float(_POINT.get("diameter", 0.04))
ZONE_B_PICK_POSE_POINT_COLOR = tuple(float(v) for v in _POINT["color_rgba"])


def _make_sphere_marker(
    marker_id: int,
    ns: str,
    position: tuple[float, float, float],
    diameter: float,
    color: tuple[float, float, float, float],
    frame_id: str = FRAME_ID,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = float(position[0])
    marker.pose.position.y = float(position[1])
    marker.pose.position.z = float(position[2])
    marker.pose.orientation.w = 1.0

    marker.scale = Vector3(
        x=float(diameter),
        y=float(diameter),
        z=float(diameter),
    )
    marker.color = ColorRGBA(
        r=float(color[0]),
        g=float(color[1]),
        b=float(color[2]),
        a=float(color[3]),
    )
    return marker


def get_zone_b_pick_pose_markers() -> MarkerArray:
    marker_array = get_zone_b_markers()
    marker_array.markers.append(
        _make_sphere_marker(
            len(marker_array.markers),
            "zone_b_pick_pose",
            ZONE_B_PICK_POSE_POINT_POSITION,
            ZONE_B_PICK_POSE_POINT_DIAMETER,
            ZONE_B_PICK_POSE_POINT_COLOR,
        )
    )
    return marker_array


class EnvironmentVisualizer:
    def __init__(self, node):
        qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = node.create_publisher(MarkerArray, "/competition_markers", qos)

    def publish_zone(self, zone: str = "B") -> None:
        if zone not in ("B", "ALL"):
            return
        self._pub.publish(get_zone_b_pick_pose_markers())

    def clear(self) -> None:
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self._pub.publish(marker_array)
