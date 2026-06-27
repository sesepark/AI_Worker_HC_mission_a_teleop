"""
Mission C place 테스트.
부품을 파이프 위에 순서대로 (pipe1 → pipe4) 삽입.

파이프 배치 (4개 중앙 = y=0, 간격 150mm):
  pipe1  pipe2  pipe3  pipe4
  y=+0.225  y=+0.075  y=-0.075  y=-0.225
  (왼팔)    (왼팔)     (오른팔)   (오른팔)

실행:
  ros2 run manipulation test_place_c
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import MoveItClient, Arm, MoveResult
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.planning_scene import setup_zone_c_table, remove_zone_c_table
from manipulation.skill_primitives.mission_c_arm_selector import select_arm
from manipulation.tests.MissionC.test_pick_C import CENTER_Y

# ── 파이프 좌표 (base_link 기준, m) ──────────────────────────────────
# TODO: 실물 계측 후 x, z 갱신
PIPE_X     = 0.40   # 테이블까지의 거리
PIPE_Z_TOP = 0.90   # 테이블 높이 + 파이프 높이(0.08m) + 여유

# y 좌표: 도면 중심간 거리 63 / 172 / 179 / 185 / 72 (mm) 기준
# 파이프 중심 위치(왼쪽 기준): 63 / 235 / 414 / 599mm
# y=0 기준: 테이블 전체 폭 670mm의 절반(335mm), 로봇 좌측 = +y
#   pipe1: (335 - 63)mm  = +0.272m
#   pipe2: (335 - 235)mm = +0.100m
#   pipe3: (414 - 335)mm = -0.079m
#   pipe4: (599 - 335)mm = -0.264m
PIPE_POSITIONS = {
    'pipe1': (PIPE_X,  0.272, PIPE_Z_TOP),
    'pipe2': (PIPE_X,  0.100, PIPE_Z_TOP),
    'pipe3': (PIPE_X, -0.079, PIPE_Z_TOP),
    'pipe4': (PIPE_X, -0.264, PIPE_Z_TOP),
}

APPROACH_HEIGHT = 0.10   # 파이프 위에서 하강 시작 높이 (m)
PLACE_Y_OFFSET  = -0.045  # pick과 동일한 y 오프셋
NAV_DISTANCE    =  0.30   # 이동 거리 가정값 (m)

# 팔-파이프 가동 범위 제약
_LEFT_PIPES  = {'pipe1', 'pipe2'}   # +y, 왼팔 전담
_RIGHT_PIPES = {'pipe3', 'pipe4'}   # -y, 오른팔 전담
# ─────────────────────────────────────────────────────────────────────

_QUAT_TOPDOWN = (0.0, 0.0, 0.0, 1.0)   # top-down (identity)


def _make_pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x, pose.orientation.y, \
        pose.orientation.z, pose.orientation.w = _QUAT_TOPDOWN
    return pose


def place_on_pipe(
    pipe_name: str,
    arm: Arm,
    client: MoveItClient,
    gripper: GripperInterface,
    log,
) -> bool:
    nav_y_offset = 0.0
    if arm == Arm.RIGHT and pipe_name in _LEFT_PIPES:
        log.warn(f'[place_c] move left — 오른팔로 {pipe_name} 도달 불가')
        # ↓ 여기서 navigation으로 왼쪽 30cm 이동
        log.info(f'[place_c] 왼쪽으로 {NAV_DISTANCE*100:.0f}cm 이동 완료 — place 재시도')
        nav_y_offset = -NAV_DISTANCE
    elif arm == Arm.LEFT and pipe_name in _RIGHT_PIPES:
        log.warn(f'[place_c] move right — 왼팔로 {pipe_name} 도달 불가')
        # ↓ 여기서 navigation으로 오른쪽 30cm 이동
        log.info(f'[place_c] 오른쪽으로 {NAV_DISTANCE*100:.0f}cm 이동 완료 — place 재시도')
        nav_y_offset = +NAV_DISTANCE

    x, y, z = PIPE_POSITIONS[pipe_name]
    ey = y + nav_y_offset + PLACE_Y_OFFSET
    log.info(f'[place_c] {pipe_name} → arm={arm.value}  pos=({x:.3f}, {ey:.3f}, {z:.3f})')

    hover  = _make_pose(x, ey, z + APPROACH_HEIGHT)
    target = _make_pose(x, ey, z)

    # hover로 글로벌 이동
    r = client.move_to_pose(hover, arm=arm, velocity=0.2, acceleration=0.2)
    if r != MoveResult.SUCCEEDED:
        log.error(f'[place_c] {pipe_name} hover 실패: {r.value}')
        return False

    # 파이프 위로 Cartesian 하강
    r = client.move_cartesian(target, arm=arm)
    if r != MoveResult.SUCCEEDED:
        log.warn(f'[place_c] {pipe_name} cartesian 하강 실패: {r.value}')
        client.move_to_pose(hover, arm=arm)
        return False

    # 그리퍼 열기 (부품 해제)
    gripper.open_to(arm.value, 0.5)
    gripper.wait_until_executed()
    gripper.wait_motion()
    log.info(f'[place_c] {pipe_name} 부품 해제 완료')

    # hover로 복귀
    client.move_cartesian(hover, arm=arm)
    return True


def main():
    rclpy.init()
    node    = Node('test_place_c')
    log     = node.get_logger()
    client  = MoveItClient(node)
    gripper = GripperInterface(node)

    setup_zone_c_table(client)

    pipe_name = 'pipe4'
    arm = select_arm(CENTER_Y)
    ok = place_on_pipe(pipe_name, arm, client, gripper, log)
    log.info(f'[place_c] {pipe_name} 결과: {"OK" if ok else "FAIL"}')

    remove_zone_c_table(client)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
