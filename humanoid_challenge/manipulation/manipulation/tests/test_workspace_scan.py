"""
로봇 테이블 진입 깊이별 yellow_box 커버리지 스캔 (IK만, 이동 없음).

테이블 하부가 뚫려 있으므로 로봇이 테이블 안으로 inset만큼 진입 가능.
inset이 커질수록 박스가 로봇에 가까워져 팔 도달 범위가 넓어짐.

yellow_box (로봇 끝단=테이블 앞 엣지 기준):
  center_x = 0.570, D=0.480 → 내부 x: 0.330 ~ 0.800 (로봇 기준 절대 좌표)
  center_y = -0.295, W=0.760 → 내부 y: -0.655 ~ +0.075

inset 적용 시 박스 x 좌표 = 절대좌표 - inset

실행:
  ros2 run manipulation test_workspace_scan
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from manipulation.robot_interface.moveit_client import MoveItClient, Arm

ARM = Arm.RIGHT

# 로봇 테이블 진입 깊이 후보 (m)
INSETS = [0.00, 0.10, 0.20, 0.30]

# yellow_box 내부 그리드 (절대 좌표 기준, inset으로 보정)
X_ABS = [0.330, 0.420, 0.510, 0.600, 0.690, 0.780]  # 박스 앞→뒤
Y_VALS = [-0.650, -0.550, -0.450, -0.350, -0.250, -0.150, -0.050, +0.050]
Z      = 0.830


def _make_pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0
    return pose


def scan_inset(client: MoveItClient, log, inset: float) -> int:
    hits = 0
    header = f'  inset={inset:.2f}m  ' + ''.join(f' y={y:+.2f}' for y in Y_VALS)
    log.info(header)
    for x_abs in X_ABS:
        x = x_abs - inset
        row = f'    x_abs={x_abs:.2f}(rel={x:.2f}) |'
        for y in Y_VALS:
            ok = client.check_reachable(_make_pose(x, y, Z), arm=ARM)
            row += ' ✅' if ok else ' ❌'
            if ok:
                hits += 1
        log.info(row)
    log.info(f'  → 도달: {hits}/{len(X_ABS)*len(Y_VALS)} pts')
    return hits


def main():
    rclpy.init()
    node   = Node('test_workspace_scan')
    log    = node.get_logger()
    client = MoveItClient(node)

    log.info('=' * 70)
    log.info(f'[workspace_scan] 진입 깊이별 yellow_box 커버리지  z={Z}')
    log.info('=' * 70)

    for inset in INSETS:
        log.info('-' * 50)
        hits = scan_inset(client, log, inset)

    log.info('=' * 70)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
