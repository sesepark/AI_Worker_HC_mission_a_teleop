"""
대회 환경 CollisionObject 설정 모듈.
규정집 제12~15조 기준 (2026 Humanoid Challenge).

좌표 기준:
  - 로봇 base_link = (0, 0, 0)
  - +x : 로봇 정면, +y : 로봇 좌측, +z : 위
  - z 공식: 테이블_상면_z + 물체_높이/2
"""

import os
import yaml
from math import sqrt

from ament_index_python.packages import get_package_share_directory
from manipulation.robot_interface.moveit_client import MoveItClient
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

FRAME_ID = "base_link"

_ALLOW_COLLISIONS_TIMEOUT_SEC = 2.0
_SPIN_TIMEOUT_SEC              = 0.05

PACKAGE_SHARE_DIR = get_package_share_directory("manipulation")
_DESK_CONFIG_PATH = os.path.join(PACKAGE_SHARE_DIR, "config", "desk.yaml")
_ZONE_A_CONFIG_PATH = os.path.join(PACKAGE_SHARE_DIR, "config", "zone_a.yaml")


def _load_zone_a_config() -> dict:
    with open(os.path.normpath(_ZONE_A_CONFIG_PATH), 'r') as f:
        return yaml.safe_load(f)['zone_a']


_ZA = _load_zone_a_config()

# ══════════════════════════════════════════════
# A구간 — 부품 선별 (v4 PDF 기준, config/zone_a.yaml)
# 화이트 테이블: W1600×D800×H800
# 노란 공구함: W760×D480×H200, 멀티피스(바닥+4벽)
# 파란 공구함: W590×D380×H150, 멀티피스(바닥+4벽)
# ══════════════════════════════════════════════

_ZA_TABLE_W    = _ZA['table']['width']
_ZA_TABLE_D    = _ZA['table']['depth']
_ZA_TABLE_H    = _ZA['table']['height']
_ZA_TABLE_TOP  = _ZA['table']['top_thickness']
_ZA_TABLE_CX   = _ZA['table']['center_x']
_ZA_TABLE_SURF = _ZA_TABLE_H

ZONE_A_TABLE_TOP_ID       = "zone_a_table_top"
ZONE_A_TABLE_TOP_SIZE     = (_ZA_TABLE_D, _ZA_TABLE_W, _ZA_TABLE_TOP)
ZONE_A_TABLE_TOP_POSITION = (_ZA_TABLE_CX, 0.0, _ZA_TABLE_H - _ZA_TABLE_TOP / 2.0)

_ZA_LEG_SZ = _ZA['table']['leg_size']
_ZA_LEG_IN = _ZA['table']['leg_inset']
_ZA_LEG_H  = _ZA_TABLE_H - _ZA_TABLE_TOP
ZONE_A_TABLE_LEG_SIZE      = (_ZA_LEG_SZ, _ZA_LEG_SZ, _ZA_LEG_H)
ZONE_A_TABLE_LEG_POSITIONS = [
    (_ZA_TABLE_CX - (_ZA_TABLE_D / 2.0 - _ZA_LEG_IN), +(_ZA_TABLE_W / 2.0 - _ZA_LEG_IN), _ZA_LEG_H / 2.0),
    (_ZA_TABLE_CX - (_ZA_TABLE_D / 2.0 - _ZA_LEG_IN), -(_ZA_TABLE_W / 2.0 - _ZA_LEG_IN), _ZA_LEG_H / 2.0),
    (_ZA_TABLE_CX + (_ZA_TABLE_D / 2.0 - _ZA_LEG_IN), +(_ZA_TABLE_W / 2.0 - _ZA_LEG_IN), _ZA_LEG_H / 2.0),
    (_ZA_TABLE_CX + (_ZA_TABLE_D / 2.0 - _ZA_LEG_IN), -(_ZA_TABLE_W / 2.0 - _ZA_LEG_IN), _ZA_LEG_H / 2.0),
]

# ── 노란 공구함 멀티피스 (W760×D480×H200, 벽두께 10mm) ──
_YB_W  = _ZA['yellow_box']['outer_width']
_YB_D  = _ZA['yellow_box']['outer_depth']
_YB_H  = _ZA['yellow_box']['outer_height']
_YB_T  = _ZA['yellow_box']['wall_thickness']
_YB_CX = _ZA['yellow_box']['center_x']
_YB_CY = _ZA['yellow_box']['center_y']
_YB_FLOOR_Z = _ZA_TABLE_SURF + _YB_T / 2.0
_YB_WALL_CZ = _ZA_TABLE_SURF + _YB_H / 2.0

ZONE_A_YELLOW_BOX_FLOOR_ID      = "zone_a_yellow_box_floor"
ZONE_A_YELLOW_BOX_WALL_FRONT_ID = "zone_a_yellow_box_wall_front"
ZONE_A_YELLOW_BOX_WALL_BACK_ID  = "zone_a_yellow_box_wall_back"
ZONE_A_YELLOW_BOX_WALL_LEFT_ID  = "zone_a_yellow_box_wall_left"
ZONE_A_YELLOW_BOX_WALL_RIGHT_ID = "zone_a_yellow_box_wall_right"

_YELLOW_BOX_PIECES: list[tuple[str, tuple, tuple]] = [
    (ZONE_A_YELLOW_BOX_FLOOR_ID,
     (_YB_D,                        _YB_W, _YB_T),
     (_YB_CX,                       _YB_CY, _YB_FLOOR_Z)),
    (ZONE_A_YELLOW_BOX_WALL_FRONT_ID,
     (_YB_T,                        _YB_W, _YB_H),
     (_YB_CX - (_YB_D / 2.0 - _YB_T / 2.0), _YB_CY, _YB_WALL_CZ)),
    (ZONE_A_YELLOW_BOX_WALL_BACK_ID,
     (_YB_T,                        _YB_W, _YB_H),
     (_YB_CX + (_YB_D / 2.0 - _YB_T / 2.0), _YB_CY, _YB_WALL_CZ)),
    (ZONE_A_YELLOW_BOX_WALL_LEFT_ID,
     (_YB_D - 2.0 * _YB_T,          _YB_T, _YB_H),
     (_YB_CX, _YB_CY + (_YB_W / 2.0 - _YB_T / 2.0), _YB_WALL_CZ)),
    (ZONE_A_YELLOW_BOX_WALL_RIGHT_ID,
     (_YB_D - 2.0 * _YB_T,          _YB_T, _YB_H),
     (_YB_CX, _YB_CY - (_YB_W / 2.0 - _YB_T / 2.0), _YB_WALL_CZ)),
]

# ── 파란 공구함 멀티피스 (W590×D380×H150, 벽두께 10mm) ──
_BB_W  = _ZA['blue_tray']['outer_width']
_BB_D  = _ZA['blue_tray']['outer_depth']
_BB_H  = _ZA['blue_tray']['outer_height']
_BB_T  = _ZA['blue_tray']['wall_thickness']
_BB_CX = _ZA['blue_tray']['center_x']
_BB_CY = _ZA['blue_tray']['center_y']
_BB_FLOOR_Z = _ZA_TABLE_SURF + _BB_T / 2.0
_BB_WALL_CZ = _ZA_TABLE_SURF + _BB_H / 2.0

ZONE_A_BLUE_TRAY_FLOOR_ID      = "zone_a_blue_tray_floor"
ZONE_A_BLUE_TRAY_WALL_FRONT_ID = "zone_a_blue_tray_wall_front"
ZONE_A_BLUE_TRAY_WALL_BACK_ID  = "zone_a_blue_tray_wall_back"
ZONE_A_BLUE_TRAY_WALL_LEFT_ID  = "zone_a_blue_tray_wall_left"
ZONE_A_BLUE_TRAY_WALL_RIGHT_ID = "zone_a_blue_tray_wall_right"

_BLUE_TRAY_PIECES: list[tuple[str, tuple, tuple]] = [
    (ZONE_A_BLUE_TRAY_FLOOR_ID,
     (_BB_D,                        _BB_W, _BB_T),
     (_BB_CX,                       _BB_CY, _BB_FLOOR_Z)),
    (ZONE_A_BLUE_TRAY_WALL_FRONT_ID,
     (_BB_T,                        _BB_W, _BB_H),
     (_BB_CX - (_BB_D / 2.0 - _BB_T / 2.0), _BB_CY, _BB_WALL_CZ)),
    (ZONE_A_BLUE_TRAY_WALL_BACK_ID,
     (_BB_T,                        _BB_W, _BB_H),
     (_BB_CX + (_BB_D / 2.0 - _BB_T / 2.0), _BB_CY, _BB_WALL_CZ)),
    (ZONE_A_BLUE_TRAY_WALL_LEFT_ID,
     (_BB_D - 2.0 * _BB_T,          _BB_T, _BB_H),
     (_BB_CX, _BB_CY + (_BB_W / 2.0 - _BB_T / 2.0), _BB_WALL_CZ)),
    (ZONE_A_BLUE_TRAY_WALL_RIGHT_ID,
     (_BB_D - 2.0 * _BB_T,          _BB_T, _BB_H),
     (_BB_CX, _BB_CY - (_BB_W / 2.0 - _BB_T / 2.0), _BB_WALL_CZ)),
]

ZONE_A_CONTAINER_IDS = [
    piece_id for piece_id, _, _ in (_YELLOW_BOX_PIECES + _BLUE_TRAY_PIECES)
]

# ══════════════════════════════════════════════
# B구간 — 부품 운반 (규정집 제13조)
# ══════════════════════════════════════════════

ZONE_B_CONVEYOR_ID             = "zone_b_conveyor"
ZONE_B_CONVEYOR_SIZE           = (0.6, 0.35, 0.5)
ZONE_B_CONVEYOR_POSITION       = (0.8, 0.0, 0.25)
ZONE_B_CONVEYOR_TOP_ID         = "zone_b_conveyor_top"
ZONE_B_CONVEYOR_TOP_SIZE       = (0.6, 0.35, 0.02)
ZONE_B_CONVEYOR_TOP_POSITION   = (0.8, 0.0, 0.51)
ZONE_B_BOX_ID                  = "zone_b_box"
ZONE_B_BOX_SIZE                = (0.41, 0.31, 0.28)
ZONE_B_BOX_POSITION            = (0.8, 0.0, 0.66)
ZONE_B_STOPLINE_ID             = "zone_b_stopline"
ZONE_B_STOPLINE_SIZE           = (0.6, 0.05, 0.005)
ZONE_B_STOPLINE_POSITION       = (0.5, 0.0, 0.003)
ZONE_B_DEST_TABLE_ID           = "zone_b_dest_table"
ZONE_B_DEST_TABLE_SIZE         = (0.8, 0.6, 0.9)
ZONE_B_DEST_TABLE_POSITION     = (1.8, 0.0, 0.45)
ZONE_B_DROPOFF_MARKER_ID       = "zone_b_dropoff_marker"
ZONE_B_DROPOFF_MARKER_SIZE     = (0.5, 0.4, 0.01)
ZONE_B_DROPOFF_MARKER_POSITION = (1.8, 0.0, 0.905)

# ══════════════════════════════════════════════
# C구간 — 순차 조립 (규정집 제14조)
# ══════════════════════════════════════════════

ZONE_C_BENCH_BODY_ID       = "zone_c_bench_body"
ZONE_C_BENCH_BODY_SIZE     = (1.0, 0.6, 0.78)
ZONE_C_BENCH_BODY_POSITION = (0.85, 0.0, 0.39)
ZONE_C_BENCH_TOP_ID        = "zone_c_bench_top"
ZONE_C_BENCH_TOP_SIZE      = (1.0, 0.6, 0.02)
ZONE_C_BENCH_TOP_POSITION  = (0.85, 0.0, 0.79)
ZONE_C_PEG_RADII           = [0.0095, 0.0125, 0.016, 0.019]
ZONE_C_PEG_HEIGHT          = 0.15
ZONE_C_PEG_X               = 0.60
ZONE_C_PEG_Z               = 0.875
ZONE_C_PEG_Y_POSITIONS     = [-0.225, -0.075, 0.075, 0.225]
ZONE_C_BOLT_RADIUS         = 0.025
ZONE_C_BOLT_HEIGHT         = 0.04
ZONE_C_BOLT_X              = 0.40
ZONE_C_BOLT_Z              = 0.825
ZONE_C_BOLT_Y_POSITIONS    = [-0.225, -0.075, 0.075, 0.225]
ZONE_C_BUTTON_ID           = "zone_c_button"
ZONE_C_BUTTON_RADIUS       = 0.03
ZONE_C_BUTTON_HEIGHT       = 0.06
ZONE_C_BUTTON_POSITION     = (0.75, 0.0, 0.83)
ZONE_C_MONITOR_ID          = "zone_c_monitor"

# ══════════════════════════════════════════════
# D구간 — 휠 장착 체결 (규정집 제15조)
# ══════════════════════════════════════════════

ZONE_D_TIRE_TABLE_ID           = "zone_d_tire_table"
ZONE_D_TIRE_TABLE_SIZE         = (0.5, 0.5, 0.78)
ZONE_D_TIRE_TABLE_POSITION     = (0.7, -0.7, 0.39)
ZONE_D_TIRE_TABLE_TOP_ID       = "zone_d_tire_table_top"
ZONE_D_TIRE_TABLE_TOP_SIZE     = (0.5, 0.5, 0.02)
ZONE_D_TIRE_TABLE_TOP_POSITION = (0.7, -0.7, 0.79)
ZONE_D_TIRE_ID                 = "zone_d_tire"
ZONE_D_TIRE_RADIUS             = 0.15
ZONE_D_TIRE_HEIGHT             = 0.05
ZONE_D_TIRE_POSITION           = (0.7, -0.7, 0.825)
ZONE_D_TOOLBOX_ID              = "zone_d_toolbox"
ZONE_D_TOOLBOX_SIZE            = (0.4, 0.4, 0.78)
ZONE_D_TOOLBOX_POSITION        = (0.7, 0.7, 0.39)
ZONE_D_TOOLBOX_TOP_ID          = "zone_d_toolbox_top"
ZONE_D_TOOLBOX_TOP_SIZE        = (0.4, 0.4, 0.02)
ZONE_D_TOOLBOX_TOP_POSITION    = (0.7, 0.7, 0.79)
ZONE_D_BOLT_ID                 = "zone_d_bolt"
ZONE_D_BOLT_RADIUS             = 0.03
ZONE_D_BOLT_HEIGHT             = 0.06
ZONE_D_BOLT_POSITION           = (0.7, 0.55, 0.833)
ZONE_D_DRILL_ID                = "zone_d_drill"
ZONE_D_DRILL_SIZE              = (0.25, 0.08, 0.12)
ZONE_D_DRILL_POSITION          = (0.7, 0.78, 0.86)
ZONE_D_HOLE_POST_ID            = "zone_d_hole_post"
ZONE_D_HOLE_POST_RADIUS        = 0.016
ZONE_D_HOLE_POST_HEIGHT        = 1.0
ZONE_D_HOLE_POST_POSITION      = (1.2, 0.0, 0.5)
ZONE_D_WHEEL_HUB_ID            = "zone_d_wheel_hub"
ZONE_D_WHEEL_HUB_RADIUS        = 0.019
ZONE_D_WHEEL_HUB_HEIGHT        = 0.05
ZONE_D_WHEEL_HUB_POSITION      = (1.2, 0.0, 1.0)
ZONE_D_HOLE_BASE_ID            = "zone_d_hole_base"
ZONE_D_HOLE_BASE_SIZE          = (0.15, 0.15, 0.05)
ZONE_D_HOLE_BASE_POSITION      = (1.2, 0.0, 0.025)


# ══════════════════════════════════════════════
# 공통 헬퍼 함수 (moveit_r 단일 publisher 사용)
# planning scene은 전역 공유이므로 한쪽으로만 보내면 됨
# ══════════════════════════════════════════════

def _add_box(client: MoveItClient, obj_id: str, size: tuple, position: tuple):
    client._moveit_r.add_collision_box(
        id=obj_id,
        size=size,
        position=position,
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    )


def _add_cylinder(client: MoveItClient, obj_id: str, radius: float, height: float, position: tuple):
    client._moveit_r.add_collision_cylinder(
        id=obj_id,
        radius=radius,
        height=height,
        position=position,
        quat_xyzw=(0.0, 0.0, 0.0, 1.0),
    )


def _remove_obj(client: MoveItClient, obj_id: str):
    client._moveit_r.remove_collision_object(id=obj_id)


# ══════════════════════════════════════════════
# A구간 설정 / 해제
# ══════════════════════════════════════════════

def setup_zone_a(client: MoveItClient) -> None:
    _add_box(client, ZONE_A_TABLE_TOP_ID, ZONE_A_TABLE_TOP_SIZE, ZONE_A_TABLE_TOP_POSITION)
    for i, pos in enumerate(ZONE_A_TABLE_LEG_POSITIONS):
        _add_box(client, f"zone_a_table_leg_{i}", ZONE_A_TABLE_LEG_SIZE, pos)
    for piece_id, size, pos in _YELLOW_BOX_PIECES:
        _add_box(client, piece_id, size, pos)
    for piece_id, size, pos in _BLUE_TRAY_PIECES:
        _add_box(client, piece_id, size, pos)


def remove_zone_a(client: MoveItClient) -> None:
    _remove_obj(client, ZONE_A_TABLE_TOP_ID)
    for i in range(len(ZONE_A_TABLE_LEG_POSITIONS)):
        _remove_obj(client, f"zone_a_table_leg_{i}")
    for piece_id, _, _ in _YELLOW_BOX_PIECES:
        _remove_obj(client, piece_id)
    for piece_id, _, _ in _BLUE_TRAY_PIECES:
        _remove_obj(client, piece_id)


# ══════════════════════════════════════════════
# B구간 설정 / 해제
# ══════════════════════════════════════════════

def setup_zone_b(client: MoveItClient) -> None:
    _add_box(client, ZONE_B_CONVEYOR_ID,       ZONE_B_CONVEYOR_SIZE,       ZONE_B_CONVEYOR_POSITION)
    _add_box(client, ZONE_B_CONVEYOR_TOP_ID,   ZONE_B_CONVEYOR_TOP_SIZE,   ZONE_B_CONVEYOR_TOP_POSITION)
    _add_box(client, ZONE_B_BOX_ID,            ZONE_B_BOX_SIZE,            ZONE_B_BOX_POSITION)
    _add_box(client, ZONE_B_STOPLINE_ID,       ZONE_B_STOPLINE_SIZE,       ZONE_B_STOPLINE_POSITION)
    _add_box(client, ZONE_B_DEST_TABLE_ID,     ZONE_B_DEST_TABLE_SIZE,     ZONE_B_DEST_TABLE_POSITION)
    _add_box(client, ZONE_B_DROPOFF_MARKER_ID, ZONE_B_DROPOFF_MARKER_SIZE, ZONE_B_DROPOFF_MARKER_POSITION)


def remove_zone_b(client: MoveItClient) -> None:
    for obj_id in (
        ZONE_B_CONVEYOR_ID, ZONE_B_CONVEYOR_TOP_ID, ZONE_B_BOX_ID,
        ZONE_B_STOPLINE_ID, ZONE_B_DEST_TABLE_ID, ZONE_B_DROPOFF_MARKER_ID,
    ):
        _remove_obj(client, obj_id)


# ══════════════════════════════════════════════
# C구간 설정 / 해제
# ══════════════════════════════════════════════

def setup_zone_c(client: MoveItClient) -> None:
    _remove_obj(client, ZONE_C_MONITOR_ID)  # 이전 세션 레거시 정리
    _add_box(client, ZONE_C_BENCH_BODY_ID, ZONE_C_BENCH_BODY_SIZE, ZONE_C_BENCH_BODY_POSITION)
    _add_box(client, ZONE_C_BENCH_TOP_ID,  ZONE_C_BENCH_TOP_SIZE,  ZONE_C_BENCH_TOP_POSITION)
    for i, (r, y) in enumerate(zip(ZONE_C_PEG_RADII, ZONE_C_PEG_Y_POSITIONS)):
        _add_cylinder(client, f"zone_c_peg_{i}", r, ZONE_C_PEG_HEIGHT, (ZONE_C_PEG_X, y, ZONE_C_PEG_Z))
    for i, y in enumerate(ZONE_C_BOLT_Y_POSITIONS):
        _add_cylinder(client, f"zone_c_bolt_{i}", ZONE_C_BOLT_RADIUS, ZONE_C_BOLT_HEIGHT, (ZONE_C_BOLT_X, y, ZONE_C_BOLT_Z))
    _add_cylinder(client, ZONE_C_BUTTON_ID, ZONE_C_BUTTON_RADIUS, ZONE_C_BUTTON_HEIGHT, ZONE_C_BUTTON_POSITION)


def remove_zone_c(client: MoveItClient) -> None:
    for obj_id in (ZONE_C_BENCH_BODY_ID, ZONE_C_BENCH_TOP_ID, ZONE_C_BUTTON_ID, ZONE_C_MONITOR_ID):
        _remove_obj(client, obj_id)
    for i in range(len(ZONE_C_PEG_Y_POSITIONS)):
        _remove_obj(client, f"zone_c_peg_{i}")
    for i in range(len(ZONE_C_BOLT_Y_POSITIONS)):
        _remove_obj(client, f"zone_c_bolt_{i}")


# ══════════════════════════════════════════════
# D구간 설정 / 해제
# ══════════════════════════════════════════════

def setup_zone_d(client: MoveItClient) -> None:
    _add_box(client, ZONE_D_TIRE_TABLE_ID,     ZONE_D_TIRE_TABLE_SIZE,     ZONE_D_TIRE_TABLE_POSITION)
    _add_box(client, ZONE_D_TIRE_TABLE_TOP_ID, ZONE_D_TIRE_TABLE_TOP_SIZE, ZONE_D_TIRE_TABLE_TOP_POSITION)
    _add_cylinder(client, ZONE_D_TIRE_ID,      ZONE_D_TIRE_RADIUS,         ZONE_D_TIRE_HEIGHT,      ZONE_D_TIRE_POSITION)
    _add_box(client, ZONE_D_TOOLBOX_ID,        ZONE_D_TOOLBOX_SIZE,        ZONE_D_TOOLBOX_POSITION)
    _add_box(client, ZONE_D_TOOLBOX_TOP_ID,    ZONE_D_TOOLBOX_TOP_SIZE,    ZONE_D_TOOLBOX_TOP_POSITION)
    _add_cylinder(client, ZONE_D_BOLT_ID,      ZONE_D_BOLT_RADIUS,         ZONE_D_BOLT_HEIGHT,      ZONE_D_BOLT_POSITION)
    _add_box(client, ZONE_D_DRILL_ID,          ZONE_D_DRILL_SIZE,          ZONE_D_DRILL_POSITION)
    _add_cylinder(client, ZONE_D_HOLE_POST_ID, ZONE_D_HOLE_POST_RADIUS,    ZONE_D_HOLE_POST_HEIGHT, ZONE_D_HOLE_POST_POSITION)
    _add_cylinder(client, ZONE_D_WHEEL_HUB_ID, ZONE_D_WHEEL_HUB_RADIUS,   ZONE_D_WHEEL_HUB_HEIGHT, ZONE_D_WHEEL_HUB_POSITION)
    _add_box(client, ZONE_D_HOLE_BASE_ID,      ZONE_D_HOLE_BASE_SIZE,      ZONE_D_HOLE_BASE_POSITION)


def remove_zone_d(client: MoveItClient) -> None:
    for obj_id in (
        ZONE_D_TIRE_TABLE_ID, ZONE_D_TIRE_TABLE_TOP_ID, ZONE_D_TIRE_ID,
        ZONE_D_TOOLBOX_ID, ZONE_D_TOOLBOX_TOP_ID, ZONE_D_BOLT_ID,
        ZONE_D_DRILL_ID, ZONE_D_HOLE_POST_ID, ZONE_D_WHEEL_HUB_ID, ZONE_D_HOLE_BASE_ID,
    ):
        _remove_obj(client, obj_id)


# ══════════════════════════════════════════════
# 전체 환경 설정 / 해제
# ══════════════════════════════════════════════

def setup_environment(client: MoveItClient) -> None:
    setup_zone_a(client)
    setup_zone_b(client)
    setup_zone_c(client)
    setup_zone_d(client)
    client._node.get_logger().info('환경 설정 완료: A/B/C/D 구간 collision objects 등록됨')


def remove_environment(client: MoveItClient) -> None:
    remove_zone_a(client)
    remove_zone_b(client)
    remove_zone_c(client)
    remove_zone_d(client)
    client._node.get_logger().info('환경 해제 완료: 모든 collision objects 제거됨')


def clear_all_objects(client: MoveItClient) -> None:
    """Planning Scene의 모든 collision object 제거. 구간 전환 전 또는 완전 초기화 시 호출."""
    client._moveit_r.clear_all_collision_objects()
    client._node.get_logger().info('씬 초기화 완료: 모든 collision objects 제거됨')


# ══════════════════════════════════════════════
# 시각화 (MarkerArray → /competition_markers)
# ══════════════════════════════════════════════

COLOR_TABLE      = (0.76, 0.60, 0.42, 0.85)
COLOR_WHITE      = (0.90, 0.90, 0.90, 0.85)
COLOR_YELLOW     = (1.00, 0.85, 0.00, 0.90)
COLOR_BLUE       = (0.10, 0.40, 0.90, 0.90)
COLOR_CONVEYOR   = (0.40, 0.25, 0.10, 0.85)
COLOR_BOX        = (0.82, 0.71, 0.55, 0.90)
COLOR_STOPLINE   = (1.00, 1.00, 0.00, 1.00)
COLOR_DEST_TABLE = (0.50, 0.50, 0.50, 0.85)
COLOR_STEEL      = (0.65, 0.65, 0.70, 0.90)
COLOR_GREEN      = (0.10, 0.80, 0.20, 1.00)
COLOR_TIRE       = (0.15, 0.15, 0.15, 0.90)
COLOR_TOOLBOX    = (0.30, 0.30, 0.35, 0.85)


def _make_box_marker(marker_id, ns, size, position, color) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.ns = ns
    m.id = marker_id
    m.type = Marker.CUBE
    m.action = Marker.ADD
    m.pose.position.x = float(position[0])
    m.pose.position.y = float(position[1])
    m.pose.position.z = float(position[2])
    m.pose.orientation.w = 1.0
    m.scale = Vector3(x=float(size[0]), y=float(size[1]), z=float(size[2]))
    m.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(color[3]))
    return m


def _make_cylinder_marker(marker_id, ns, radius, height, position, color) -> Marker:
    m = Marker()
    m.header.frame_id = FRAME_ID
    m.ns = ns
    m.id = marker_id
    m.type = Marker.CYLINDER
    m.action = Marker.ADD
    m.pose.position.x = float(position[0])
    m.pose.position.y = float(position[1])
    m.pose.position.z = float(position[2])
    m.pose.orientation.w = 1.0
    m.scale = Vector3(x=float(radius * 2), y=float(radius * 2), z=float(height))
    m.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(color[3]))
    return m


def _zone_a_marker_entries() -> list[tuple[str, Marker]]:
    entries: list[tuple[str, Marker]] = []
    mid = 0
    ns = "zone_a"
    entries.append((ZONE_A_TABLE_TOP_ID,
                    _make_box_marker(mid, ns, ZONE_A_TABLE_TOP_SIZE, ZONE_A_TABLE_TOP_POSITION, COLOR_TABLE))); mid += 1
    for i, pos in enumerate(ZONE_A_TABLE_LEG_POSITIONS):
        entries.append((f"zone_a_table_leg_{i}",
                        _make_box_marker(mid, ns, ZONE_A_TABLE_LEG_SIZE, pos, COLOR_WHITE))); mid += 1
    for piece_id, size, pos in _YELLOW_BOX_PIECES:
        entries.append((piece_id, _make_box_marker(mid, ns, size, pos, COLOR_YELLOW))); mid += 1
    for piece_id, size, pos in _BLUE_TRAY_PIECES:
        entries.append((piece_id, _make_box_marker(mid, ns, size, pos, COLOR_BLUE))); mid += 1
    return entries


def _zone_b_marker_entries() -> list[tuple[str, Marker]]:
    entries: list[tuple[str, Marker]] = []
    mid = 0
    ns = "zone_b"
    entries.append((ZONE_B_CONVEYOR_ID,       _make_box_marker(mid, ns, ZONE_B_CONVEYOR_SIZE,       ZONE_B_CONVEYOR_POSITION,       COLOR_CONVEYOR)));   mid += 1
    entries.append((ZONE_B_CONVEYOR_TOP_ID,   _make_box_marker(mid, ns, ZONE_B_CONVEYOR_TOP_SIZE,   ZONE_B_CONVEYOR_TOP_POSITION,   COLOR_CONVEYOR)));   mid += 1
    entries.append((ZONE_B_BOX_ID,            _make_box_marker(mid, ns, ZONE_B_BOX_SIZE,            ZONE_B_BOX_POSITION,            COLOR_BOX)));        mid += 1
    entries.append((ZONE_B_STOPLINE_ID,       _make_box_marker(mid, ns, ZONE_B_STOPLINE_SIZE,       ZONE_B_STOPLINE_POSITION,       COLOR_STOPLINE)));   mid += 1
    entries.append((ZONE_B_DEST_TABLE_ID,     _make_box_marker(mid, ns, ZONE_B_DEST_TABLE_SIZE,     ZONE_B_DEST_TABLE_POSITION,     COLOR_DEST_TABLE))); mid += 1
    entries.append((ZONE_B_DROPOFF_MARKER_ID, _make_box_marker(mid, ns, ZONE_B_DROPOFF_MARKER_SIZE, ZONE_B_DROPOFF_MARKER_POSITION, COLOR_STOPLINE)));   mid += 1
    return entries


def _zone_c_marker_entries() -> list[tuple[str, Marker]]:
    entries: list[tuple[str, Marker]] = []
    mid = 0
    ns = "zone_c"
    entries.append((ZONE_C_BENCH_BODY_ID, _make_box_marker(mid, ns, ZONE_C_BENCH_BODY_SIZE, ZONE_C_BENCH_BODY_POSITION, COLOR_WHITE))); mid += 1
    entries.append((ZONE_C_BENCH_TOP_ID,  _make_box_marker(mid, ns, ZONE_C_BENCH_TOP_SIZE,  ZONE_C_BENCH_TOP_POSITION,  COLOR_TABLE)));  mid += 1
    for i, (r, y) in enumerate(zip(ZONE_C_PEG_RADII, ZONE_C_PEG_Y_POSITIONS)):
        entries.append((f"zone_c_peg_{i}", _make_cylinder_marker(mid, ns, r, ZONE_C_PEG_HEIGHT, (ZONE_C_PEG_X, y, ZONE_C_PEG_Z), COLOR_STEEL))); mid += 1
    for i, y in enumerate(ZONE_C_BOLT_Y_POSITIONS):
        entries.append((f"zone_c_bolt_{i}", _make_cylinder_marker(mid, ns, ZONE_C_BOLT_RADIUS, ZONE_C_BOLT_HEIGHT, (ZONE_C_BOLT_X, y, ZONE_C_BOLT_Z), COLOR_STEEL))); mid += 1
    entries.append((ZONE_C_BUTTON_ID, _make_cylinder_marker(mid, ns, ZONE_C_BUTTON_RADIUS, ZONE_C_BUTTON_HEIGHT, ZONE_C_BUTTON_POSITION, COLOR_GREEN))); mid += 1
    return entries


def _zone_d_marker_entries() -> list[tuple[str, Marker]]:
    entries: list[tuple[str, Marker]] = []
    mid = 0
    ns = "zone_d"
    entries.append((ZONE_D_TIRE_TABLE_ID,     _make_box_marker(mid, ns, ZONE_D_TIRE_TABLE_SIZE,     ZONE_D_TIRE_TABLE_POSITION,     COLOR_WHITE)));  mid += 1
    entries.append((ZONE_D_TIRE_TABLE_TOP_ID, _make_box_marker(mid, ns, ZONE_D_TIRE_TABLE_TOP_SIZE, ZONE_D_TIRE_TABLE_TOP_POSITION, COLOR_TABLE)));  mid += 1
    entries.append((ZONE_D_TIRE_ID,           _make_cylinder_marker(mid, ns, ZONE_D_TIRE_RADIUS, ZONE_D_TIRE_HEIGHT, ZONE_D_TIRE_POSITION, COLOR_TIRE)));   mid += 1
    entries.append((ZONE_D_TOOLBOX_ID,        _make_box_marker(mid, ns, ZONE_D_TOOLBOX_SIZE,        ZONE_D_TOOLBOX_POSITION,        COLOR_TOOLBOX))); mid += 1
    entries.append((ZONE_D_TOOLBOX_TOP_ID,    _make_box_marker(mid, ns, ZONE_D_TOOLBOX_TOP_SIZE,    ZONE_D_TOOLBOX_TOP_POSITION,    COLOR_TABLE)));   mid += 1
    entries.append((ZONE_D_BOLT_ID,           _make_cylinder_marker(mid, ns, ZONE_D_BOLT_RADIUS, ZONE_D_BOLT_HEIGHT, ZONE_D_BOLT_POSITION, COLOR_STEEL)));  mid += 1
    entries.append((ZONE_D_DRILL_ID,          _make_box_marker(mid, ns, ZONE_D_DRILL_SIZE,          ZONE_D_DRILL_POSITION,          COLOR_STEEL)));   mid += 1
    entries.append((ZONE_D_HOLE_POST_ID,      _make_cylinder_marker(mid, ns, ZONE_D_HOLE_POST_RADIUS, ZONE_D_HOLE_POST_HEIGHT, ZONE_D_HOLE_POST_POSITION, COLOR_STEEL))); mid += 1
    entries.append((ZONE_D_WHEEL_HUB_ID,      _make_cylinder_marker(mid, ns, ZONE_D_WHEEL_HUB_RADIUS, ZONE_D_WHEEL_HUB_HEIGHT, ZONE_D_WHEEL_HUB_POSITION, COLOR_STEEL))); mid += 1
    entries.append((ZONE_D_HOLE_BASE_ID,      _make_box_marker(mid, ns, ZONE_D_HOLE_BASE_SIZE,      ZONE_D_HOLE_BASE_POSITION,      COLOR_STEEL)));   mid += 1
    return entries


def get_zone_a_markers() -> MarkerArray:
    ma = MarkerArray()
    for _, m in _zone_a_marker_entries():
        ma.markers.append(m)
    return ma


def get_zone_b_markers() -> MarkerArray:
    ma = MarkerArray()
    for _, m in _zone_b_marker_entries():
        ma.markers.append(m)
    return ma


def get_zone_c_markers() -> MarkerArray:
    ma = MarkerArray()
    for _, m in _zone_c_marker_entries():
        ma.markers.append(m)
    return ma


def get_zone_d_markers() -> MarkerArray:
    ma = MarkerArray()
    for _, m in _zone_d_marker_entries():
        ma.markers.append(m)
    return ma


def _build_object_marker_map() -> dict[str, Marker]:
    mapping: dict[str, Marker] = {}
    for zone_fn in (_zone_a_marker_entries, _zone_b_marker_entries,
                    _zone_c_marker_entries, _zone_d_marker_entries):
        for obj_id, m in zone_fn():
            mapping[obj_id] = m
    return mapping


_OBJECT_MARKER_MAP: dict[str, Marker] = _build_object_marker_map()
_NS_ID_TO_OBJ: dict[tuple[str, int], str] = {
    (m.ns, m.id): obj_id for obj_id, m in _OBJECT_MARKER_MAP.items()
}


class EnvironmentVisualizer:
    """
    /competition_markers 토픽으로 MarkerArray를 퍼블리시하는 시각화 클래스.
    """

    _ZONE_FN = {
        'A': get_zone_a_markers,
        'B': get_zone_b_markers,
        'C': get_zone_c_markers,
        'D': get_zone_d_markers,
    }

    def __init__(self, node):
        qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pub = node.create_publisher(MarkerArray, '/competition_markers', qos)
        self._highlights: dict = {}
        self._floating: set = set()
        self._overrides: dict = {}

    def publish_zone(self, zone: str):
        if zone == 'ALL':
            ma = MarkerArray()
            for fn in self._ZONE_FN.values():
                ma.markers.extend(fn().markers)
        elif zone in self._ZONE_FN:
            ma = self._ZONE_FN[zone]()
        else:
            return
        filtered = []
        for m in ma.markers:
            key = _NS_ID_TO_OBJ.get((m.ns, m.id))
            if key and key in self._floating:
                continue
            if key and key in self._overrides:
                ox, oy, oz = self._overrides[key]
                m.pose.position.x = float(ox)
                m.pose.position.y = float(oy)
                m.pose.position.z = float(oz)
            if key and key in self._highlights:
                c = self._highlights[key]
                m.color = ColorRGBA(r=float(c[0]), g=float(c[1]), b=float(c[2]), a=float(c[3]))
            filtered.append(m)
        ma.markers = filtered
        self._pub.publish(ma)

    def clear(self):
        self._highlights.clear()
        self._floating.clear()
        self._overrides.clear()
        ma = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self._pub.publish(ma)

    def highlight_object(self, object_id: str, color: tuple) -> None:
        self._highlights[object_id] = color
        orig = _OBJECT_MARKER_MAP.get(object_id)
        if orig is None:
            return
        m = Marker()
        m.header.frame_id = orig.header.frame_id
        m.ns = orig.ns
        m.id = orig.id
        m.type = orig.type
        m.action = Marker.ADD
        if object_id in self._overrides:
            ox, oy, oz = self._overrides[object_id]
            m.pose = Pose(position=Point(x=float(ox), y=float(oy), z=float(oz)),
                          orientation=orig.pose.orientation)
        else:
            m.pose = orig.pose
        m.scale = orig.scale
        m.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(color[3]))
        ma = MarkerArray()
        ma.markers.append(m)
        self._pub.publish(ma)

    def set_floating(self, object_id: str) -> None:
        self._floating.add(object_id)
        self._overrides.pop(object_id, None)

    def restore_object(self, object_id: str) -> None:
        self._highlights.pop(object_id, None)
        self._floating.discard(object_id)
        self._overrides.pop(object_id, None)
        orig = _OBJECT_MARKER_MAP.get(object_id)
        if orig is None:
            return
        ma = MarkerArray()
        ma.markers.append(orig)
        self._pub.publish(ma)

    def drop_object(self, object_id: str, x: float, y: float, z: float = 0.01) -> None:
        self._highlights.pop(object_id, None)
        self._floating.discard(object_id)
        self._overrides[object_id] = (x, y, z)
        orig = _OBJECT_MARKER_MAP.get(object_id)
        if orig is None:
            return
        m = Marker()
        m.header.frame_id = orig.header.frame_id
        m.ns = orig.ns
        m.id = orig.id
        m.type = orig.type
        m.action = Marker.ADD
        m.pose = Pose(position=Point(x=float(x), y=float(y), z=float(z)),
                      orientation=orig.pose.orientation)
        m.scale = orig.scale
        m.color = orig.color
        ma = MarkerArray()
        ma.markers.append(m)
        self._pub.publish(ma)

    def move_highlight(self, object_id: str, x: float, y: float, z: float) -> None:
        orig = _OBJECT_MARKER_MAP.get(object_id)
        if orig is None:
            return
        color = self._highlights.get(object_id)
        if color is None:
            return
        m = Marker()
        m.header.frame_id = orig.header.frame_id
        m.ns = orig.ns
        m.id = orig.id
        m.type = orig.type
        m.action = Marker.ADD
        m.pose = Pose(position=Point(x=float(x), y=float(y), z=float(z)),
                      orientation=orig.pose.orientation)
        m.scale = orig.scale
        m.color = ColorRGBA(r=float(color[0]), g=float(color[1]), b=float(color[2]), a=float(color[3]))
        ma = MarkerArray()
        ma.markers.append(m)
        self._pub.publish(ma)


# ══════════════════════════════════════════════
# 파지 가능 물체 목록
# ══════════════════════════════════════════════

GRASPABLE_OBJECTS = {
    'A': [],  # 실물 부품은 perception이 실시간으로 탐지 — collision object로 등록하지 않음
    'B': ['zone_b_box'],
    'C': ['zone_c_bolt_0', 'zone_c_bolt_1', 'zone_c_bolt_2', 'zone_c_bolt_3'],
    'D': ['zone_d_tire', 'zone_d_bolt'],
}

OBJECT_LUT_NAME: dict[str, str] = {
    'zone_b_box':    'ETC',
    'zone_c_bolt_0': 'ETC',
    'zone_c_bolt_1': 'ETC',
    'zone_c_bolt_2': 'ETC',
    'zone_c_bolt_3': 'ETC',
    'zone_d_tire':   'ETC',
    'zone_d_bolt':   'ETC',
    'zone_d_drill':  'ETC',
}

SURFACE_OBJECTS = {
    'A': [ZONE_A_TABLE_TOP_ID],
    'B': [ZONE_B_CONVEYOR_TOP_ID, ZONE_B_DEST_TABLE_ID],
    'C': [ZONE_C_BENCH_TOP_ID],
    'D': [ZONE_D_TIRE_TABLE_TOP_ID, ZONE_D_TOOLBOX_TOP_ID],
}


# ══════════════════════════════════════════════
# Attach / Detach
# ══════════════════════════════════════════════

_TOUCH_LINKS_R = [
    'gripper_r_rh_p12_rn_base',
    'gripper_r_rh_p12_rn_l1', 'gripper_r_rh_p12_rn_l2',
    'gripper_r_rh_p12_rn_r1', 'gripper_r_rh_p12_rn_r2',
]
_TOUCH_LINKS_L = [
    'gripper_l_rh_p12_rn_base',
    'gripper_l_rh_p12_rn_l1', 'gripper_l_rh_p12_rn_l2',
    'gripper_l_rh_p12_rn_r1', 'gripper_l_rh_p12_rn_r2',
]


def attach_object(client: MoveItClient, object_id: str,
                  link_name: str = 'end_effector_r_link',
                  viz: EnvironmentVisualizer | None = None) -> None:
    left = 'end_effector_l' in link_name
    moveit2     = client._moveit_l if left else client._moveit_r
    touch_links = _TOUCH_LINKS_L   if left else _TOUCH_LINKS_R
    moveit2.attach_collision_object(id=object_id, link_name=link_name, touch_links=touch_links)
    if viz is not None:
        viz.highlight_object(object_id, (1.0, 0.0, 0.0, 1.0))
        viz.set_floating(object_id)


def detach_object(client: MoveItClient, object_id: str,
                  link_name: str = 'end_effector_r_link',
                  viz: EnvironmentVisualizer | None = None,
                  drop_pos: tuple | None = None) -> None:
    left = 'end_effector_l' in link_name
    moveit2 = client._moveit_l if left else client._moveit_r
    moveit2.detach_collision_object(id=object_id)
    if viz is not None:
        if drop_pos is not None:
            viz.drop_object(object_id, drop_pos[0], drop_pos[1], drop_pos[2])
        else:
            viz.restore_object(object_id)


# ══════════════════════════════════════════════
# allow_collisions
# ══════════════════════════════════════════════

def allow_zone_objects(client: MoveItClient, zone: str) -> None:
    """zone의 파지 대상 물체와의 충돌 허용. setup_zone_X 호출 후 실행.
    컨테이너(박스/트레이) 벽은 위가 열려 있으므로 충돌 허용 불필요 — 제외."""
    import time as _t
    objs = list(GRASPABLE_OBJECTS.get(zone, []))
    for obj_id in objs:
        for mv in (client._moveit_r, client._moveit_l):
            f = mv.allow_collisions(obj_id, True)
            if f is None:
                continue
            deadline = _t.time() + _ALLOW_COLLISIONS_TIMEOUT_SEC
            while not f.done() and _t.time() < deadline:
                _t.sleep(_SPIN_TIMEOUT_SEC)


# ══════════════════════════════════════════════
# 책상 Collision Object (테스트 씬용)
# ══════════════════════════════════════════════

DESK_TOP_ID  = "test_desk_top"
DESK_BODY_ID = "test_desk_body"


def _load_desk_config() -> dict:
    with open(os.path.normpath(_DESK_CONFIG_PATH), 'r') as f:
        return yaml.safe_load(f)['desk']


def setup_desk(client: MoveItClient) -> None:
    """config/desk.yaml 파라미터로 책상 collision object를 Planning Scene에 등록."""
    cfg = _load_desk_config()
    top_thickness: float = float(cfg['top_thickness'])
    width:         float = float(cfg['width'])
    depth:         float = float(cfg['depth'])
    height:        float = float(cfg['height'])
    dist:          float = float(cfg['distance_from_robot'])

    top_z:       float = height - top_thickness / 2.0
    center_x:    float = dist + depth / 2.0
    body_height: float = height - top_thickness
    body_z:      float = body_height / 2.0

    _add_box(client, DESK_TOP_ID,  (depth, width, top_thickness), (center_x, 0.0, top_z))
    _add_box(client, DESK_BODY_ID, (depth, width, body_height),   (center_x, 0.0, body_z))
    client._node.get_logger().info(
        f'[setup_desk] 완료 (top_z={top_z:.3f}, center_x={center_x:.3f})'
    )


def remove_desk(client: MoveItClient) -> None:
    _remove_obj(client, DESK_TOP_ID)
    _remove_obj(client, DESK_BODY_ID)
    client._node.get_logger().info('[remove_desk] 책상 제거 완료')
