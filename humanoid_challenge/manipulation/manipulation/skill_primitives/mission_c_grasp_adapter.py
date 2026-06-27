"""Mission C grasp adapter — 부품 중심 pose → grasp pose 변환.

Mission A(`mission_a_grasp_adapter.build_mission_a_grasp_pose`)와 동일 역할의 C 버전.
C 는 모든 부품을 **top-down(identity) 자세**로 집고, pick y 에 고정 오프셋을 더한다.
(test_pick_C.build_c_grasp_pose 이식 + R1/R3 명명 정정.)

정정(분석 R1/R3):
  - R1: 원본 `_QUAT_YAW90 = (0,0,0,1)` 은 변수명·주석과 달리 **항등(top-down)** 이었다 →
        `_QUAT_TOPDOWN` 로 정명(값 동일, 의미 명확화).
  - R3: `GRASP_Y_OFFSET` 은 **y** 축에 적용된다(원본 주석은 "x 오프셋" 오기였음).
"""
from __future__ import annotations

from geometry_msgs.msg import Pose

GRASP_Z        = 0.83     # 그리퍼 목표 높이 [m] (고정)
GRASP_Y_OFFSET = -0.045   # pick y 축 오프셋 [m] (mission_a adapter 의 y 오프셋과 동일 크기)
CARRY_Z        = 1.150    # pick 완료 후 carry 높이 [m]

_QUAT_TOPDOWN = (0.0, 0.0, 0.0, 1.0)   # top-down(identity) — 그리퍼 수직 하향


def build_c_grasp_pose(center: Pose) -> Pose:
    """부품 중심 Pose → Mission C grasp Pose (top-down, y 오프셋, 고정 z)."""
    pose = Pose()
    pose.position.x = center.position.x
    pose.position.y = center.position.y + GRASP_Y_OFFSET
    pose.position.z = GRASP_Z
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = _QUAT_TOPDOWN
    return pose
