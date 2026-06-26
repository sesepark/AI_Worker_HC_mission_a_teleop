"""Mission A grasp pose adapter: orientation + offset rules by object position.

기본 로직 (위치 기반):
  x >= X_FAR_THRESHOLD (3열)       → yaw90 + x오프셋
  x <  X_FAR + y >= Y_RIGHT_WALL  → standard + y -= GRASP_Y_OFFSET
  x <  X_FAR + y <  Y_RIGHT_WALL  → standard + y += GRASP_Y_OFFSET

주변 너트 근접 로직 (y방향):
  neighbors 중 dist < NEIGHBOR_CLOSE_THRESH 있으면:
    - 좌(+y)에만 붙음  → 우(-y) 오프셋, 단 우벽이면 yaw90
    - 우(-y)에만 붙음  → 좌(+y) 오프셋, 단 좌벽이면 yaw90
    - 양쪽 다 붙음     → yaw90

알려진 제약:
  3열(x >= X_FAR) + 좌벽(y >= Y_LEFT_WALL) 구역은 yaw90 접근이 카메라로 인해
  불안정할 수 있음. yaw180 등 대안 탐색 중 — 현재는 yaw90 그대로 시도.
"""

from __future__ import annotations

import logging
from typing import Sequence

from geometry_msgs.msg import Pose

_log = logging.getLogger(__name__)



# ── 위치 기반 상수 ────────────────────────────────────────────────────
X_FAR_THRESHOLD = 0.40     # m — 3열 경계
Y_RIGHT_WALL    = -0.30    # m — 우벽 경계 (실측 후 수정)
Y_LEFT_WALL     =  0.30    # m — 좌벽 경계 (실측 후 수정)

# standard approach (yaw0) 오프셋
GRASP_STD_X_OFFSET = 0.0   # m — x 미세 조정 (실측 후 수정, 예: -0.043)
GRASP_STD_Y_OFFSET = 0.0   # m — y 좌/우 회피 (실측 후 수정, 예: 0.043 / 0.045)

# yaw90 approach 오프셋
GRASP_YAW_X_OFFSET = 0.0   # m — x 진입 조정 (실측 후 수정, 예: -0.043 / -0.045)
GRASP_YAW_Y_OFFSET = 0.0   # m — y 미세 조정 (실측 후 수정, 예: 0.043)

GRASP_Z         = 0.83     # m — 고정 파지 높이

# ── 근접 판단 상수 ────────────────────────────────────────────────────
NEIGHBOR_CLOSE_THRESH = 0.120   # m — 이 이내면 붙어있다고 판단 (너트 중앙 간격 기준)
NEIGHBOR_Y_MARGIN     = 0.010   # m — 좌/우 방향 판단 최소 y 차이

# ── 오리엔테이션 쿼터니언 ─────────────────────────────────────────────
_QUAT_STANDARD = (0.0, 0.0, 0.0,    1.0)      # 표준 top-down
_QUAT_YAW90    = (0.0, 0.0, 0.7071, 0.7071)   # Z축 90° 회전
_QUAT_YAW180   = (0.0, 0.0, 1.0,    0.0)      # Z축 180° 회전 (탐색 중)


def build_mission_a_grasp_pose(
    center_pose: Pose,
    neighbors: Sequence[Pose] | None = None,
) -> Pose:
    """Return final grasp pose with orientation, offset, and z applied.

    neighbors: 주변 너트 중앙좌표 리스트 (없으면 위치 기반 로직만 적용).
    """
    pose = _copy_pose(center_pose)
    pose.position.z = GRASP_Z

    cx = center_pose.position.x
    cy = center_pose.position.y

    _log.info(f'[grasp_adapter] target=({cx:.3f},{cy:.3f}) neighbors={len(neighbors) if neighbors else 0}개')

    # ── 1. 근접 너트 분석 ──────────────────────────────────────────────
    left_close  = False
    right_close = False

    if neighbors:
        for nb in neighbors:
            dist = ((nb.position.x - cx) ** 2 + (nb.position.y - cy) ** 2) ** 0.5
            if dist < NEIGHBOR_CLOSE_THRESH:
                dy = nb.position.y - cy
                if dy > NEIGHBOR_Y_MARGIN:
                    left_close = True
                elif dy < -NEIGHBOR_Y_MARGIN:
                    right_close = True

    # ── 2. 근접 너트 있으면 회피 로직 ─────────────────────────────────
    if left_close or right_close:
        if left_close and right_close:
            case = '양쪽 근접 → yaw90'
            qx, qy, qz, qw = _QUAT_YAW90
            pose.position.x += GRASP_YAW_X_OFFSET
            pose.position.y += GRASP_YAW_Y_OFFSET
        elif left_close:
            if cy < Y_RIGHT_WALL:
                case = '좌 근접 + 우벽 → yaw90 [제약]'
                qx, qy, qz, qw = _QUAT_YAW90
                pose.position.x += GRASP_YAW_X_OFFSET
                pose.position.y += GRASP_YAW_Y_OFFSET
            else:
                case = '좌 근접 → 우(-y) 회피'
                qx, qy, qz, qw = _QUAT_STANDARD
                pose.position.x += GRASP_STD_X_OFFSET
                pose.position.y -= GRASP_STD_Y_OFFSET
        else:
            if cy >= Y_LEFT_WALL:
                case = '우 근접 + 좌벽 → yaw90 [제약: 카메라 충돌 위험]'
                qx, qy, qz, qw = _QUAT_YAW90
                pose.position.x += GRASP_YAW_X_OFFSET
                pose.position.y += GRASP_YAW_Y_OFFSET
            else:
                case = '우 근접 → 좌(+y) 회피'
                qx, qy, qz, qw = _QUAT_STANDARD
                pose.position.x += GRASP_STD_X_OFFSET
                pose.position.y += GRASP_STD_Y_OFFSET

        p = pose.position
        _log.info(f'[grasp_adapter] {case} → pose=({p.x:.3f},{p.y:.3f},{p.z:.3f})')
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    # ── 3. 기존 위치 기반 로직 ────────────────────────────────────────
    if cx >= X_FAR_THRESHOLD:
        case = f'3열(x={cx:.3f}) → yaw90'
        qx, qy, qz, qw = _QUAT_YAW90
        pose.position.x += GRASP_YAW_X_OFFSET
        pose.position.y += GRASP_YAW_Y_OFFSET
    else:
        qx, qy, qz, qw = _QUAT_STANDARD
        pose.position.x += GRASP_STD_X_OFFSET
        if cy >= Y_RIGHT_WALL:
            case = f'standard 일반(y={cy:.3f}) → y-=STD_Y'
            pose.position.y -= GRASP_STD_Y_OFFSET
        else:
            case = f'standard 우벽(y={cy:.3f}) → y+=STD_Y'
            pose.position.y += GRASP_STD_Y_OFFSET

    p = pose.position
    _log.info(f'[grasp_adapter] {case} → pose=({p.x:.3f},{p.y:.3f},{p.z:.3f})')
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def _copy_pose(pose: Pose) -> Pose:
    p = Pose()
    p.position.x = pose.position.x
    p.position.y = pose.position.y
    p.position.z = pose.position.z
    p.orientation.x = pose.orientation.x
    p.orientation.y = pose.orientation.y
    p.orientation.z = pose.orientation.z
    p.orientation.w = pose.orientation.w
    return p
