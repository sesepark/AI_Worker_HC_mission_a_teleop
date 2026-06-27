from __future__ import annotations

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from manipulation.robot_interface.moveit_client import MoveItClient

from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)


FRAME_ID = "odom"

_TF_LOOKUP_TIMEOUT_SEC = 2.0
_TF_POLL_SEC           = 0.05


def _lookup_base_to_odom(node):
    """현재 odom→base_link TF를 조회. 타임아웃 시 None 반환."""
    import time as _time
    import tf2_ros
    from rclpy.time import Time as _Time
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)
    deadline = _time.time() + _TF_LOOKUP_TIMEOUT_SEC
    while _time.time() < deadline:
        try:
            return buf.lookup_transform('odom', 'base_link', _Time())
        except tf2_ros.TransformException:
            _time.sleep(_TF_POLL_SEC)
    node.get_logger().warn('[planning_scene_b_place] odom→base_link TF lookup 타임아웃, base_link 좌표 그대로 사용')
    return None


def _apply_tf(tf_stamped, pos: tuple) -> tuple:
    """TransformStamped를 적용해 base_link 좌표 → odom 좌표 변환."""
    if tf_stamped is None:
        return pos
    x, y, z = float(pos[0]), float(pos[1]), float(pos[2])
    t = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    rx = (1 - 2*(qy**2 + qz**2))*x + 2*(qx*qy - qz*qw)*y + 2*(qx*qz + qy*qw)*z
    ry = 2*(qx*qy + qz*qw)*x + (1 - 2*(qx**2 + qz**2))*y + 2*(qy*qz - qx*qw)*z
    rz = 2*(qx*qz - qy*qw)*x + 2*(qy*qz + qx*qw)*y + (1 - 2*(qx**2 + qy**2))*z
    return (rx + t.x, ry + t.y, rz + t.z)

PACKAGE_NAME = "manipulation"
PACKAGE_SHARE_DIR = get_package_share_directory(PACKAGE_NAME)
ZONE_B_CONFIG_PATH = os.path.join(PACKAGE_SHARE_DIR, "config", "zone_b_place.yaml")


def _load_zone_b_config() -> dict:
    with open(ZONE_B_CONFIG_PATH, "r") as f:
        return yaml.safe_load(f)["zone_b"]


_ZB = _load_zone_b_config()


# =============================================================================
# Zone B constants
# =============================================================================

# Conveyor
_ZB_CONV = _ZB["conveyor"]
ZONE_B_CONVEYOR_ID = "zone_b_conveyor"
ZONE_B_CONVEYOR_SIZE = (
    _ZB_CONV["depth"],
    _ZB_CONV["length"],
    _ZB_CONV["height"],
)
ZONE_B_CONVEYOR_POSITION = (
    _ZB_CONV["center_x"],
    _ZB_CONV["center_y"],
    _ZB_CONV["height"] / 2.0,
)
_ZB_CONV_SURF_Z = _ZB_CONV["height"]

# Box
_ZB_BOX = _ZB["box"]
ZONE_B_BOX_ID = "zone_b_box"
ZONE_B_BOX_SIZE = (
    _ZB_BOX["depth"],
    _ZB_BOX["width"],
    _ZB_BOX["height"],
)
ZONE_B_BOX_POSITION = (
    _ZB_BOX["center_x"],
    _ZB_BOX["center_y"],
    _ZB_CONV_SURF_Z + _ZB_BOX["height"] / 2.0,
)

# Destination table
_ZB_TBL = _ZB["dest_table"]
_ZB_TBL_TOP = _ZB_TBL["top_thickness"]
_ZB_TBL_LEG_H = _ZB_TBL["height"] - _ZB_TBL_TOP
_ZB_TBL_CX = _ZB_TBL["center_x"]
_ZB_TBL_CY = _ZB_TBL["center_y"]
_ZB_TBL_SURF = _ZB_TBL["height"]

ZONE_B_DEST_TABLE_BODY_ID = "zone_b_dest_table_body"
ZONE_B_DEST_TABLE_BODY_SIZE = (
    _ZB_TBL["depth"],
    _ZB_TBL["width"],
    _ZB_TBL_LEG_H,
)
ZONE_B_DEST_TABLE_BODY_POSITION = (
    _ZB_TBL_CX,
    _ZB_TBL_CY,
    _ZB_TBL_LEG_H / 2.0,
)

ZONE_B_DEST_TABLE_TOP_ID = "zone_b_dest_table_top"
ZONE_B_DEST_TABLE_TOP_SIZE = (
    _ZB_TBL["depth"],
    _ZB_TBL["width"],
    _ZB_TBL_TOP,
)
ZONE_B_DEST_TABLE_TOP_POSITION = (
    _ZB_TBL_CX,
    _ZB_TBL_CY,
    _ZB_TBL["height"] - _ZB_TBL_TOP / 2.0,
)

_ZB_LEG_SZ = _ZB_TBL["leg_size"]
_ZB_LEG_IN = _ZB_TBL["leg_inset"]
ZONE_B_DEST_TABLE_LEG_SIZE = (
    _ZB_LEG_SZ,
    _ZB_LEG_SZ,
    _ZB_TBL_LEG_H,
)
ZONE_B_DEST_TABLE_LEG_POSITIONS = [
    (
        _ZB_TBL_CX - (_ZB_TBL["depth"] / 2.0 - _ZB_LEG_IN),
        +(_ZB_TBL["width"] / 2.0 - _ZB_LEG_IN),
        _ZB_TBL_LEG_H / 2.0,
    ),
    (
        _ZB_TBL_CX - (_ZB_TBL["depth"] / 2.0 - _ZB_LEG_IN),
        -(_ZB_TBL["width"] / 2.0 - _ZB_LEG_IN),
        _ZB_TBL_LEG_H / 2.0,
    ),
    (
        _ZB_TBL_CX + (_ZB_TBL["depth"] / 2.0 - _ZB_LEG_IN),
        +(_ZB_TBL["width"] / 2.0 - _ZB_LEG_IN),
        _ZB_TBL_LEG_H / 2.0,
    ),
    (
        _ZB_TBL_CX + (_ZB_TBL["depth"] / 2.0 - _ZB_LEG_IN),
        -(_ZB_TBL["width"] / 2.0 - _ZB_LEG_IN),
        _ZB_TBL_LEG_H / 2.0,
    ),
]

# Landing marker
_ZB_MRK = _ZB["landing_marker"]
ZONE_B_LANDING_MARKER_ID = "zone_b_landing_marker"
ZONE_B_LANDING_MARKER_SIZE = (
    _ZB_MRK["size_x"],
    _ZB_MRK["size_y"],
    _ZB_MRK["thickness"],
)
ZONE_B_LANDING_MARKER_POSITION = (
    _ZB_MRK["center_x"],
    _ZB_MRK["center_y"],
    _ZB_TBL_SURF + _ZB_MRK["thickness"] / 2.0,
)

# Stop line
_ZB_SL = _ZB["stop_line"]
ZONE_B_STOPLINE_ID = "zone_b_stopline"
ZONE_B_STOPLINE_SIZE = (
    _ZB_SL["thickness"],
    _ZB_SL["length"],
    _ZB_SL["height"],
)
ZONE_B_STOPLINE_POSITION = (
    _ZB_SL["center_x"],
    _ZB_SL["center_y"],
    _ZB_SL["height"] / 2.0,
)


# =============================================================================
# Collision object helpers
# =============================================================================

def _add_box(client: MoveItClient, obj_id: str, size: tuple, position: tuple, frame_id: str = "base_link") -> None:
    client._moveit_r.add_collision_box(
        id=obj_id,
        size=size,
        position=position,
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        frame_id=frame_id,
    )


def _remove_obj(client: MoveItClient, obj_id: str) -> None:
    client._moveit_r.remove_collision_object(id=obj_id)


def clear_all_objects(client: MoveItClient) -> None:
    client._moveit_r.clear_all_collision_objects()
    client._node.get_logger().info(
        "[planning_scene_b] Planning Scene의 모든 collision objects 제거 완료"
    )


def setup_zone_b(client: MoveItClient) -> None:
    _remove_obj(client, ZONE_B_BOX_ID)
    odom_tf = _lookup_base_to_odom(client._node)
    _p = lambda pos: _apply_tf(odom_tf, pos)
    _add_box(client, ZONE_B_CONVEYOR_ID,        ZONE_B_CONVEYOR_SIZE,        _p(ZONE_B_CONVEYOR_POSITION),        "odom")
    _add_box(client, ZONE_B_DEST_TABLE_BODY_ID, ZONE_B_DEST_TABLE_BODY_SIZE, _p(ZONE_B_DEST_TABLE_BODY_POSITION), "odom")
    _add_box(client, ZONE_B_DEST_TABLE_TOP_ID,  ZONE_B_DEST_TABLE_TOP_SIZE,  _p(ZONE_B_DEST_TABLE_TOP_POSITION),  "odom")
    for i, pos in enumerate(ZONE_B_DEST_TABLE_LEG_POSITIONS):
        _add_box(client, f"zone_b_dest_table_leg_{i}", ZONE_B_DEST_TABLE_LEG_SIZE, _p(pos), "odom")
    client._node.get_logger().info(
        "[planning_scene_b_place] Zone B collision objects 등록 완료 "
        "(zone_b_box는 시각화만 하고 collision object에서 제외)"
    )


def remove_zone_b(client: MoveItClient) -> None:
    for obj_id in (
        ZONE_B_CONVEYOR_ID,
        ZONE_B_BOX_ID,
        ZONE_B_DEST_TABLE_BODY_ID,
        ZONE_B_DEST_TABLE_TOP_ID,
    ):
        _remove_obj(client, obj_id)

    for i in range(len(ZONE_B_DEST_TABLE_LEG_POSITIONS)):
        _remove_obj(client, f"zone_b_dest_table_leg_{i}")

    client._node.get_logger().info(
        "[planning_scene_b] Zone B collision objects 제거 완료"
    )


# =============================================================================
# RViz Marker visualization
# =============================================================================

COLOR_CONVEYOR = (0.40, 0.25, 0.10, 0.85)
COLOR_BOX = (0.82, 0.71, 0.55, 0.90)
COLOR_WHITE = (0.90, 0.90, 0.90, 0.85)
COLOR_TABLE = (0.76, 0.60, 0.42, 0.85)
COLOR_GREEN = (0.10, 0.80, 0.20, 1.00)


def _make_box_marker(
    marker_id: int,
    ns: str,
    size: tuple,
    position: tuple,
    color: tuple,
    frame_id: str = FRAME_ID,
) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = ns
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD

    marker.pose.position.x = float(position[0])
    marker.pose.position.y = float(position[1])
    marker.pose.position.z = float(position[2])
    marker.pose.orientation.w = 1.0

    marker.scale = Vector3(
        x=float(size[0]),
        y=float(size[1]),
        z=float(size[2]),
    )

    marker.color = ColorRGBA(
        r=float(color[0]),
        g=float(color[1]),
        b=float(color[2]),
        a=float(color[3]),
    )

    return marker


def get_zone_b_markers() -> MarkerArray:
    marker_array = MarkerArray()
    marker_id = 0
    ns = "zone_b"

    marker_array.markers.append(
        _make_box_marker(
            marker_id,
            ns,
            ZONE_B_CONVEYOR_SIZE,
            ZONE_B_CONVEYOR_POSITION,
            COLOR_CONVEYOR,
        )
    )
    marker_id += 1

    marker_array.markers.append(
        _make_box_marker(
            marker_id,
            ns,
            ZONE_B_BOX_SIZE,
            ZONE_B_BOX_POSITION,
            COLOR_BOX,
        )
    )
    marker_id += 1

    marker_array.markers.append(
        _make_box_marker(
            marker_id,
            ns,
            ZONE_B_DEST_TABLE_BODY_SIZE,
            ZONE_B_DEST_TABLE_BODY_POSITION,
            COLOR_WHITE,
        )
    )
    marker_id += 1

    marker_array.markers.append(
        _make_box_marker(
            marker_id,
            ns,
            ZONE_B_DEST_TABLE_TOP_SIZE,
            ZONE_B_DEST_TABLE_TOP_POSITION,
            COLOR_TABLE,
        )
    )
    marker_id += 1

    for pos in ZONE_B_DEST_TABLE_LEG_POSITIONS:
        marker_array.markers.append(
            _make_box_marker(
                marker_id,
                ns,
                ZONE_B_DEST_TABLE_LEG_SIZE,
                pos,
                COLOR_WHITE,
            )
        )
        marker_id += 1

    marker_array.markers.append(
        _make_box_marker(
            marker_id,
            ns,
            ZONE_B_LANDING_MARKER_SIZE,
            ZONE_B_LANDING_MARKER_POSITION,
            COLOR_GREEN,
        )
    )
    marker_id += 1

    marker_array.markers.append(
        _make_box_marker(
            marker_id,
            ns,
            ZONE_B_STOPLINE_SIZE,
            ZONE_B_STOPLINE_POSITION,
            COLOR_GREEN,
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
        self._pub.publish(get_zone_b_markers())

    def clear(self) -> None:
        marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self._pub.publish(marker_array)
