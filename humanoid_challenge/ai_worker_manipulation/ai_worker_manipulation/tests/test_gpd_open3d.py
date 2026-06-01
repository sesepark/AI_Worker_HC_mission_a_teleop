#!/usr/bin/env python3
import os
import subprocess

import numpy as np
import open3d as o3d
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from scipy.spatial.transform import Rotation


# ---------------------------------------------------------------------------
# 오른팔 depth cam → base_link 더미 transform
#
# 실제 로봇에서는 TF에서 lookup_transform으로 가져옴.
# 데모용 고정값: 카메라가 base_link 기준으로 어디에 달려있는지 정의.
#
# ROS 카메라 optical frame 관례:
#   x = 이미지 오른쪽, y = 이미지 아래쪽, z = 깊이(전방)
# ---------------------------------------------------------------------------

# 오른팔 카메라 위치 (base_link 기준, 단위: m)
_CAM_POS = np.array([0.35, -0.2, 0.5])

# 카메라가 바라보는 방향: base_link 기준 전방+약간 아래
_cam_z = np.array([0.8,  0.2, -0.6])
_cam_z = _cam_z / np.linalg.norm(_cam_z)

# 카메라 x(이미지 우측) ≈ base_link -y 방향, cam_z와 직교화
_cam_x = np.array([0.0, -1.0, 0.0])
_cam_x -= np.dot(_cam_x, _cam_z) * _cam_z
_cam_x = _cam_x / np.linalg.norm(_cam_x)

# 카메라 y(이미지 하단) = z × x
_cam_y = np.cross(_cam_z, _cam_x)

# 4×4 homogeneous transform: camera_optical_frame → base_link
T_CAM_TO_BASE = np.eye(4)
T_CAM_TO_BASE[:3, :3] = np.column_stack([_cam_x, _cam_y, _cam_z])
T_CAM_TO_BASE[:3,  3] = _CAM_POS

# 카메라에서 본 물체 위치 (카메라 z방향 30cm 앞)
OBJECT_IN_CAM = np.array([0.0, 0.0, 0.3])


'''
현재 더미:


# 고정값 (팔 자세 무관)
T_CAM_TO_BASE = np.eye(4)
T_CAM_TO_BASE[:3, :3] = R_fixed
T_CAM_TO_BASE[:3, 3]  = [0.35, -0.2, 0.5]
실제 로봇 연결 시:


tf_buffer = tf2_ros.Buffer()
t = tf_buffer.lookup_transform('base_link', 'camera_r_depth_optical_frame', rclpy.time.Time())

# 번역
trans = t.transform.translation
T_CAM_TO_BASE[:3, 3] = [trans.x, trans.y, trans.z]

# 회전 (quaternion → 회전행렬)
rot = t.transform.rotation
R = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
T_CAM_TO_BASE[:3, :3] = R
'''


# ---------------------------------------------------------------------------
# 좌표 변환 유틸
# ---------------------------------------------------------------------------

def transform_points(pts: np.ndarray, T: np.ndarray) -> np.ndarray:
    """(N,3) 포인트 배열에 4×4 homogeneous transform 적용."""
    ones = np.ones((len(pts), 1))
    pts_h = np.hstack([pts, ones])          # (N, 4)
    return (T @ pts_h.T).T[:, :3]          # (N, 3)


# ---------------------------------------------------------------------------
# Dummy PCD 생성 (카메라 좌표계 → base_link 변환 포함)
# ---------------------------------------------------------------------------

def capture_from_viewpoint(pcd_full, camera_location):
    _, pt_map = pcd_full.hidden_point_removal(camera_location, radius=100)
    return pcd_full.select_by_index(pt_map)


def make_dummy_pcd_in_base_link() -> tuple[o3d.geometry.PointCloud, np.ndarray]:
    """
    1. Bunny를 카메라 좌표계에서 OBJECT_IN_CAM 위치에 생성
    2. T_CAM_TO_BASE로 base_link 좌표계로 변환
    3. 변환된 PCD와 base_link 기준 물체 중심 반환
    """
    # --- 카메라 좌표계에서 Bunny 생성 ---
    bunny = o3d.data.BunnyMesh()
    mesh  = o3d.io.read_triangle_mesh(bunny.path)
    mesh.compute_vertex_normals()
    mesh.scale(0.7, center=mesh.get_center())
    mesh.translate(OBJECT_IN_CAM - mesh.get_center())

    pcd_cam = mesh.sample_points_uniformly(number_of_points=20000)

    # 카메라 시점에서 보이는 포인트만 선택 (카메라는 원점에서 바라봄)
    cam_origin_in_cam = np.array([0.0, 0.0, 0.0])
    pcd_visible = capture_from_viewpoint(pcd_cam, cam_origin_in_cam)

    pts_cam = np.asarray(pcd_visible.points, dtype=np.float64)
    print(f"카메라 좌표계 포인트 수: {len(pts_cam)}")

    # --- base_link 좌표계로 변환 ---
    pts_base = transform_points(pts_cam, T_CAM_TO_BASE)

    pcd_base = o3d.geometry.PointCloud()
    pcd_base.points = o3d.utility.Vector3dVector(pts_base)
    pcd_base, _ = pcd_base.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd_base = pcd_base.voxel_down_sample(voxel_size=0.003)

    # base_link 기준 물체 중심
    object_center_base = transform_points(OBJECT_IN_CAM.reshape(1, 3), T_CAM_TO_BASE)[0]

    return pcd_base, object_center_base


# ---------------------------------------------------------------------------
# GPD 실행 및 파싱
# ---------------------------------------------------------------------------

def run_gpd(pcd_path: str) -> str:
    gpd_dir = "/root/ros2_ws/src/ai_worker/gpd"
    result  = subprocess.run(
        ["./build/detect_grasps", "cfg/eigen_params.cfg", pcd_path],
        cwd=gpd_dir,
        capture_output=True,
        text=True,
        env={**os.environ, "LIBGL_ALWAYS_SOFTWARE": "1"},
    )
    if result.returncode != 0:
        print(f"[GPD 오류] returncode={result.returncode}")
        print(result.stderr[-500:])
    return result.stdout


def parse_grasp_poses(stdout: str) -> list[dict]:
    grasps: list[dict] = []
    cur: dict = {}

    for line in stdout.splitlines():
        if 'Grasp' in line and 'score:' in line:
            if cur:
                grasps.append(cur)
            cur = {'score': float(line.split('score:')[1].replace(')', '').strip())}
        elif cur:
            for key in ('position', 'approach', 'binormal', 'axis'):
                if f'{key}:' in line:
                    vals = line.split(f'{key}:')[1].strip().split()
                    cur[key] = np.array([float(v) for v in vals[:3]])
                    break

    if cur:
        grasps.append(cur)
    return grasps


def filter_grasps_by_score(grasps: list[dict], min_score: float = 0.0) -> list[dict]:
    """score가 min_score 이상인 grasp만 통과."""
    return [g for g in grasps if g.get('score', -999) >= min_score]


# ---------------------------------------------------------------------------
# MoveIt 좌표계 변환 (base_link frame)
# ---------------------------------------------------------------------------

def grasp_to_pose_moveit(grasp: dict) -> Pose:
    """
    GPD hand frame → MoveIt EEF frame 변환.
    MoveIt 관례: EEF z축 = 그리퍼 접근 방향(approach)
    R = [binormal | axis | approach]  (x, y, z 컬럼)
    """
    def _n(v):
        n = np.linalg.norm(v)
        return v / n if n > 1e-8 else v

    approach = _n(grasp.get('approach', np.array([1., 0., 0.])))
    binormal = _n(grasp.get('binormal', np.array([0., 1., 0.])))
    axis     = _n(grasp.get('axis',     np.array([0., 0., 1.])))

    R = np.column_stack((binormal, axis, approach))
    R, _ = np.linalg.qr(R)
    if np.linalg.det(R) < 0:
        R[:, 2] *= -1

    q = Rotation.from_matrix(R).as_quat()  # [x, y, z, w]

    pose = Pose()
    pos = grasp['position']
    pose.position.x = float(pos[0])
    pose.position.y = float(pos[1])
    pose.position.z = float(pos[2])
    pose.orientation.x = float(q[0])
    pose.orientation.y = float(q[1])
    pose.orientation.z = float(q[2])
    pose.orientation.w = float(q[3])
    return pose


# ---------------------------------------------------------------------------
# ROS 2 노드
# ---------------------------------------------------------------------------

class GraspResultPublisher(Node):
    def __init__(self):
        super().__init__('gpd_grasp_result_publisher')

        # transient_local: 나중에 subscribe해도 마지막 메시지를 받음 (ROS 1 latch 동작)
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(PoseArray, '/grasp_poses', qos)

    def publish_grasps(self, grasps: list[dict]):
        if not grasps:
            self.get_logger().warn('No valid grasps to publish.')
            return

        sorted_grasps = sorted(grasps, key=lambda g: g['score'], reverse=True)

        msg = PoseArray()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()

        for g in sorted_grasps:
            if 'position' not in g or 'approach' not in g:
                continue
            msg.poses.append(grasp_to_pose_moveit(g))

        if not msg.poses:
            self.get_logger().warn('No poses built.')
            return

        self.pub.publish(msg)
        self.get_logger().info(
            f'Published {len(msg.poses)} poses → /gpd/grasp_poses [base_link]')
        self.get_logger().info(
            f'Top grasp score: {sorted_grasps[0]["score"]:.4f}')


# ---------------------------------------------------------------------------
# 메인
# ---------------------------------------------------------------------------

def main():
    print("오른팔 depth cam 더미 PCD 생성 중...")
    print(f"  카메라 위치 (base_link): {_CAM_POS}")
    print(f"  물체 위치 (카메라 기준): {OBJECT_IN_CAM}")

    pcd_base, object_center_base = make_dummy_pcd_in_base_link()
    print(f"  물체 위치 (base_link):   {object_center_base.round(3)}")
    print(f"  변환 후 포인트 수: {len(pcd_base.points)}")

    pcd_path = "/tmp/test_bunny_base_link.pcd"
    o3d.io.write_point_cloud(pcd_path, pcd_base)

    print("\nGPD 실행 중...")
    stdout = run_gpd(pcd_path)

    printing = False
    for line in stdout.splitlines():
        if 'Selected grasps' in line:
            printing = True
        if printing:
            print(line)

    grasps = parse_grasp_poses(stdout)
    valid  = filter_grasps_by_score(grasps, min_score=0.0)

    print(f"\n필터링 후 유효한 grasp: {len(valid)}/{len(grasps)}개")
    for i, g in enumerate(valid):
        print(f"  Grasp {i}  score={g['score']:.4f}  pos={g['position'].round(3)}")

    rclpy.init()
    node = GraspResultPublisher()
    node.publish_grasps(valid)

    print("\n토픽 발행 중... Ctrl+C로 종료")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()