"""Mission C grasp adapter — 부품 중심 pose → grasp pose 변환.

Mission A standard 오프셋과 동일하게 적용 (벽/열 조건 없이 무조건 standard).
"""
from __future__ import annotations

from geometry_msgs.msg import Pose

GRASP_STD_X_OFFSET = -0.005   # m — Mission A standard와 동일
GRASP_STD_Y_OFFSET =  0.038   # m — Mission A standard와 동일
GRASP_Z            =  0.850   # 그리퍼 목표 높이 [m]
CARRY_Z            =  1.150   # pick 완료 후 carry 높이 [m]

_QUAT_STANDARD = (0.0, 0.0, 0.0, 1.0)   # top-down(identity)


def build_c_grasp_pose(center: Pose) -> Pose:
    """부품 중심 Pose → Mission C grasp Pose (top-down, standard 오프셋, 고정 z)."""
    pose = Pose()
    pose.position.x = center.position.x + GRASP_STD_X_OFFSET
    pose.position.y = center.position.y - GRASP_STD_Y_OFFSET
    pose.position.z = GRASP_Z
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = _QUAT_STANDARD
    return pose
