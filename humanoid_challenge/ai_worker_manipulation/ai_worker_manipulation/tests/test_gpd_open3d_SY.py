import open3d as o3d
import numpy as np
import subprocess
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose

def capture_from_viewpoint(pcd_full, camera_location):
    _, pt_map = pcd_full.hidden_point_removal(camera_location, radius=100)
    return pcd_full.select_by_index(pt_map)

def parse_grasp_poses(output):
    grasps = []
    lines = output.split('\n')
    current_grasp = {}

    for line in lines:
        if 'Grasp' in line and 'score:' in line:
            if current_grasp:
                grasps.append(current_grasp)

            score_str = line.split('score:')[1].replace(')', '').strip()
            current_grasp = {'score': float(score_str)}

        elif 'position:' in line and current_grasp:
            vals = line.split('position:')[1].strip().split()
            current_grasp['position'] = np.array([float(v) for v in vals[:3]])

        elif 'approach:' in line and current_grasp:
            vals = line.split('approach:')[1].strip().split()
            current_grasp['approach'] = np.array([float(v) for v in vals[:3]])

        elif 'binormal:' in line and current_grasp:
            vals = line.split('binormal:')[1].strip().split()
            current_grasp['binormal'] = np.array([float(v) for v in vals[:3]])

        elif 'axis:' in line and current_grasp:
            vals = line.split('axis:')[1].strip().split()
            current_grasp['axis'] = np.array([float(v) for v in vals[:3]])

    if current_grasp:
        grasps.append(current_grasp)

    return grasps


def filter_grasps_by_approach(grasps, camera_positions, object_center=np.array([0,0,0])):
    valid_grasps = []
    for grasp in grasps:
        if 'approach' not in grasp:
            continue
        approach_norm = grasp['approach'] / np.linalg.norm(grasp['approach'])
        is_valid = False
        for cam_pos in camera_positions:
            cam_dir = object_center - cam_pos
            cam_dir = cam_dir / np.linalg.norm(cam_dir)
            if np.dot(approach_norm, cam_dir) > 0.3:
                is_valid = True
                break
        if is_valid:
            valid_grasps.append(grasp)
    return valid_grasps


def normalize(v):
    norm = np.linalg.norm(v)

    if norm < 1e-8:
        return v

    return v / norm


def rotation_matrix_to_quaternion(R):
    q = np.empty(4)
    trace = np.trace(R)

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[2, 1] - R[1, 2]) * s
        q[1] = (R[0, 2] - R[2, 0]) * s
        q[2] = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s

    q = q / np.linalg.norm(q)

    return q


def quaternion_from_grasp(grasp):
    if "approach" in grasp and "binormal" in grasp and "axis" in grasp:
        approach = normalize(grasp["approach"])
        binormal = normalize(grasp["binormal"])
        axis = normalize(grasp["axis"])

        R = np.column_stack((approach, binormal, axis))

        return rotation_matrix_to_quaternion(R)

    approach = normalize(grasp["approach"])

    up = np.array([0.0, 0.0, 1.0])

    if abs(np.dot(approach, up)) > 0.95:
        up = np.array([0.0, 1.0, 0.0])

    y_axis = normalize(np.cross(up, approach))
    z_axis = normalize(np.cross(approach, y_axis))

    R = np.column_stack((approach, y_axis, z_axis))

    return rotation_matrix_to_quaternion(R)



class GraspResultPublisher(Node):
    def __init__(self):
        super().__init__("gpd_grasp_result_publisher")

        self.publisher = self.create_publisher(
            PoseArray,
            "/grasp_poses",
            10
        )

    def publish_grasps(self, grasps, frame_id="world"):
        if len(grasps) == 0:
            self.get_logger().warn("No valid grasps to publish.")
            return

        sorted_grasps = sorted(
            grasps,
            key=lambda g: g["score"],
            reverse=True
        )

        msg = PoseArray()
        msg.header.frame_id = frame_id

        for grasp in sorted_grasps:
            if "position" not in grasp:
                continue

            if "approach" not in grasp:
                continue

            pose = Pose()

            position = grasp["position"]
            qx, qy, qz, qw = quaternion_from_grasp(grasp)

            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2])

            pose.orientation.x = float(qx)
            pose.orientation.y = float(qy)
            pose.orientation.z = float(qz)
            pose.orientation.w = float(qw)

            msg.poses.append(pose)

        if len(msg.poses) == 0:
            self.get_logger().warn("No poses were added to PoseArray.")
            return

        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info(
            f"Published {len(msg.poses)} grasp poses to /grasp_poses"
        )

        self.get_logger().info(
            f"Top grasp score: {sorted_grasps[0]['score']:.4f}"
        )


def run_gpd(pcd_path, scene_name, camera_positions, grasp_publisher):
    print(f"\n{'='*50}")
    print(f"Scene: {scene_name}")
    print(f"{'='*50}")
    gpd_dir = "/root/ros2_ws/src/ai_worker/gpd"
    result = subprocess.run(
        ["./build/detect_grasps", "cfg/eigen_params.cfg", pcd_path],
        cwd=gpd_dir,
        capture_output=True,
        text=True,
        env={**os.environ, "LIBGL_ALWAYS_SOFTWARE": "1"}
    )
    lines = result.stdout.split('\n')
    printing = False
    for line in lines:
        if 'GRASP POSES' in line or 'Selected grasps' in line:
            printing = True
        if printing:
            print(line)

    grasps = parse_grasp_poses(result.stdout)
    valid = filter_grasps_by_approach(grasps, camera_positions)

    print(f"\n필터링 후 유효한 grasp: {len(valid)}/{len(grasps)}개")

    for i, g in enumerate(valid):
        print(f"  Valid Grasp {i} (score: {g['score']:.2f})")
        print(f"    position: {g['position']}")
        print(f"    approach: {g['approach']}")

    valid_sorted = sorted(
        valid,
        key=lambda g: g["score"],
        reverse=True
    )

    if len(valid_sorted) == 0:
        print("\n유효한 grasp가 없습니다.")
        return None

    print("\nScore 기준 정렬된 valid grasp")
    for i, g in enumerate(valid_sorted):
        print(f"  Grasp {i} (score: {g['score']:.4f})")
        print(f"    position: {g['position']}")
        print(f"    approach: {g['approach']}")

    print("\nTop Grasp")
    print(f"  score: {valid_sorted[0]['score']:.4f}")
    print(f"  position: {valid_sorted[0]['position']}")
    print(f"  approach: {valid_sorted[0]['approach']}")

    grasp_publisher.publish_grasps(
        valid,
        frame_id="world"
    )

    return valid_sorted[0]

        

# =============================================
# AI Worker 양팔 카메라 시점 (위에서 비스듬히)
# =============================================
camera_positions = [
    np.array([0.0, -0.35, 0.3]),   # 오른팔 측면
    np.array([0.0,  0.35, 0.3]),   # 왼팔 측면
]

# =============================================
# Stanford Bunny - 실제 스캔 데이터 (사각지대 있음)
# =============================================
print("Bunny 포인트 클라우드 생성 중...")
bunny = o3d.data.BunnyMesh()
mesh = o3d.io.read_triangle_mesh(bunny.path)
mesh.compute_vertex_normals()
mesh.scale(0.7, center=mesh.get_center())
pcd_full = mesh.sample_points_uniformly(number_of_points=20000)

# 양팔 카메라 시점 합성
all_points = []
for cam_pos in camera_positions:
    pcd_view = capture_from_viewpoint(pcd_full, cam_pos)
    all_points.append(np.asarray(pcd_view.points))

pcd_combined = o3d.geometry.PointCloud()
pcd_combined.points = o3d.utility.Vector3dVector(np.vstack(all_points))
pcd_combined, _ = pcd_combined.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
pcd_combined = pcd_combined.voxel_down_sample(voxel_size=0.003)
print(f"포인트 수: {len(pcd_combined.points)}")

o3d.io.write_point_cloud("/tmp/test_bunny_dual.pcd", pcd_combined)

rclpy.init()

grasp_publisher = GraspResultPublisher()

run_gpd(
    "/tmp/test_bunny_dual.pcd",
    "Bunny 양팔 카메라 시점",
    camera_positions,
    grasp_publisher
)

grasp_publisher.destroy_node()
rclpy.shutdown()