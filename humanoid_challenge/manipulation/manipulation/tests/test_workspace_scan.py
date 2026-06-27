"""
로봇 테이블 진입 깊이별 yellow_box 커버리지 스캔 (IK만, 이동 없음).

테이블 하부가 뚫려 있으므로 로봇이 테이블 안으로 inset만큼 진입 가능.
inset이 커질수록 박스가 로봇에 가까워져 팔 도달 범위가 넓어짐.

yellow_box (로봇 끝단=테이블 앞 엣지 기준):
  center_x = 0.570, D=0.480 → 내부 x: 0.330 ~ 0.800 (로봇 기준 절대 좌표)
  center_y = -0.295, W=0.760 → 내부 y: -0.655 ~ +0.075

inset 적용 시 박스 x 좌표 = 절대좌표 - inset

실행:
  ros2 run manipulation test_workspace_scan                        # 기본 yaw0 전체 스캔
  ros2 run manipulation test_workspace_scan --ros-args -p mode:=yaw180  # yaw180 좌상단 집중 스캔
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

# yaw180 집중 스캔 범위 — (0.475, -0.203, 0.880) 기준 앞/좌/아래
YAW180_X_VALS = [0.425, 0.475, 0.525, 0.575, 0.625]   # 앞(+x) 방향
YAW180_Y_VALS = [-0.203, -0.150, -0.100, -0.050, 0.000, +0.050]  # 좌(+y) 방향
YAW180_Z_VALS = [0.880, 0.860, 0.850, 0.840, 0.830]               # 아래(-z) 방향 (0.830까지)

# yaw-90: Z축 -90° 회전
_QUAT_YAW_NEG90 = (-0.7071, 0.0, 0.0, 0.7071)  # qx=-0.7071, qw=0.7071... wait
# qz=-0.7071, qw=0.7071 for yaw -90 around z
_QZ_YAW180    = (0.0,     0.0, 1.0,    0.0)    # (qx,qy,qz,qw)
_QZ_YAW_NEG90 = (0.0,     0.0, -0.7071, 0.7071)


def _make_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
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


def _scan_xy(client, log, label, x_vals, y_vals, z, qx, qy, qz, qw):
    log.info(f'\n[X/Y 스캔]  z={z:.3f}  orientation={label}')
    header = '           ' + ''.join(f'  y={y:+.3f}' for y in y_vals)
    log.info(header)
    for x in x_vals:
        row = f'  x={x:.3f} |'
        for y in y_vals:
            ok = client.check_reachable(_make_pose(x, y, z, qx, qy, qz, qw), arm=ARM)
            row += '  ✅ ' if ok else '  ❌ '
        log.info(row)


def _scan_z(client, log, label, x, y, z_vals, qx, qy, qz, qw):
    log.info(f'\n[Z 스캔]  x={x:.3f}  y={y:.3f}  orientation={label}')
    for z in z_vals:
        ok = client.check_reachable(_make_pose(x, y, z, qx, qy, qz, qw), arm=ARM)
        log.info(f'  z={z:.3f} → {"✅ reachable" if ok else "❌ unreachable"}')


def scan_yaw180(client: MoveItClient, log, lift_offset: float = 0.0) -> None:
    """yaw180 orientation으로 좌상단 집중 스캔.
    lift_offset: lift를 내린 경우 z에 더할 오프셋 (e.g. 0.10 for lift -10cm).
    """
    z_fixed = 0.880 + lift_offset
    z_vals  = [z + lift_offset for z in YAW180_Z_VALS]
    label   = f'lift_offset={lift_offset:+.2f}' if lift_offset else 'lift=0'

    log.info('=' * 70)
    log.info(f'[yaw180 스캔]  {label}  기준점=(0.475,-0.203,{z_fixed:.3f})')
    log.info('=' * 70)
    _scan_xy(client, log, f'yaw180 {label}', YAW180_X_VALS, YAW180_Y_VALS,
             z_fixed, 0.0, 0.0, *_QZ_YAW180[2:])
    _scan_z(client, log, f'yaw180 {label}', 0.475, -0.203, z_vals,
            0.0, 0.0, *_QZ_YAW180[2:])


def scan_yaw_neg90(client: MoveItClient, log, lift_offset: float = 0.0) -> None:
    """yaw-90 orientation으로 좌상단 집중 스캔."""
    z_fixed = 0.880 + lift_offset
    z_vals  = [z + lift_offset for z in YAW180_Z_VALS]
    label   = f'lift_offset={lift_offset:+.2f}' if lift_offset else 'lift=0'

    log.info('=' * 70)
    log.info(f'[yaw-90 스캔]  {label}  기준점=(0.475,-0.203,{z_fixed:.3f})')
    log.info('=' * 70)
    _scan_xy(client, log, f'yaw-90 {label}', YAW180_X_VALS, YAW180_Y_VALS,
             z_fixed, *_QZ_YAW_NEG90)
    _scan_z(client, log, f'yaw-90 {label}', 0.475, -0.203, z_vals,
            *_QZ_YAW_NEG90)


def main():
    rclpy.init()
    node   = Node('test_workspace_scan')
    log    = node.get_logger()
    client = MoveItClient(node)

    node.declare_parameter('mode', 'default')
    node.declare_parameter('lift_offset', 0.0)
    mode        = node.get_parameter('mode').get_parameter_value().string_value
    lift_offset = node.get_parameter('lift_offset').get_parameter_value().double_value

    if mode == 'yaw180':
        scan_yaw180(client, log, lift_offset)
    elif mode == 'yaw_neg90':
        scan_yaw_neg90(client, log, lift_offset)
    else:
        log.info('=' * 70)
        log.info(f'[workspace_scan] 진입 깊이별 yellow_box 커버리지  z={Z}  orientation=yaw0')
        log.info('=' * 70)
        for inset in INSETS:
            log.info('-' * 50)
            scan_inset(client, log, inset)
        log.info('=' * 70)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
