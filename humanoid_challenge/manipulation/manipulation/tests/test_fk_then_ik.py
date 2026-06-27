"""
특정 joint 값 → FK → z 오프셋 적용 → IK → 새 joint 값 출력.

실행:
  ros2 run manipulation test_fk_then_ik
"""

import time

import rclpy
from rclpy.node import Node

from manipulation.robot_interface.moveit_client import Arm, MoveItClient, MoveResult

ARM = Arm.RIGHT

# ── 여기를 수정 ──────────────────────────────────────────────────────
START_JOINTS = [-2.707296, -0.299926, 2.883739, -2.112988, -1.433787, 0.369488, 0.659124]
Z_OFFSET     = -0.15   # 낮출 높이 [m]  (음수 = 아래)
# ─────────────────────────────────────────────────────────────────────


def main():
    rclpy.init()
    node   = Node('test_fk_then_ik')
    log    = node.get_logger()
    client = MoveItClient(node)

    # 1. 해당 joint로 이동
    log.info(f'[fk_ik] joints로 이동: {START_JOINTS}')
    r = client.move_to_joints(
        START_JOINTS, arm=ARM,
        velocity=0.2, acceleration=0.2,
        pipeline='pilz_industrial_motion_planner', planner='PTP',
    )
    if r != MoveResult.SUCCEEDED:
        log.error(f'[fk_ik] 이동 실패: {r.value}')
        node.destroy_node()
        rclpy.shutdown()
        return

    time.sleep(0.5)

    # 2. FK로 현재 pose 취득
    pose = client.get_pose(arm=ARM)
    if pose is None:
        log.error('[fk_ik] FK 실패')
        node.destroy_node()
        rclpy.shutdown()
        return

    p = pose.position
    log.info(f'[fk_ik] FK 결과: pos=({p.x:.4f},{p.y:.4f},{p.z:.4f})')

    # 3. z 오프셋 적용
    pose.position.z += Z_OFFSET
    log.info(f'[fk_ik] 목표 pos: ({p.x:.4f},{p.y:.4f},{p.z:.4f})')

    # 4. IK
    joints = client.solve_ik(pose, arm=ARM)
    if joints is None:
        log.error('[fk_ik] IK 실패 — 해당 위치 도달 불가')
        node.destroy_node()
        rclpy.shutdown()
        return

    joints_str = ', '.join(f'{v:.6f}' for v in joints)
    log.info(f'[fk_ik] IK 결과 joint 값:')
    log.info(f'  [{joints_str}]')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
