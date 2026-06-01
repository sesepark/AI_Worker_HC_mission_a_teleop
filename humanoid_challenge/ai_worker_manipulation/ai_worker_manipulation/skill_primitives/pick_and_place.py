from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, MoveResult
from ai_worker_manipulation.robot_interface.gripper_controller import GripperController
from ai_worker_manipulation.skill_primitives.grasp_assesment import GraspAssessment
from geometry_msgs.msg import PoseArray, Pose
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import rclpy
import time
from scipy.spatial.transform import Rotation

GRASP_TOPIC = "/grasp_poses" # 나중에 이름 바뀌면 여기서 바꾸자

# test_gpd_open3d.py publisher와 QoS 맞춰야 latched message 수신 가능
_GRASP_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

def wait_for_grasp(client: MoveItClient, timeout: float = 30.0) -> Pose | None:
    "GPD에서 grasp pose publish된거 기다리고, 그중 가장 높은 점수 + kinematically reachable한 하나의 Pose 반환"
    log = client.node.get_logger()
    log.info("Waiting for grasp result...")
    all_poses = []

    def callback(msg: PoseArray):
        if not all_poses:  # take only the first message, ignore duplicate publishes
            log.info(f"Received {len(msg.poses)} grasp candidates.")
            all_poses.extend(msg.poses)

    sub = client.node.create_subscription(PoseArray, GRASP_TOPIC, callback, _GRASP_QOS)

    deadline = time.time() + timeout
    while not all_poses and time.time() < deadline:
        rclpy.spin_once(client.node, timeout_sec=0.1)

    client.node.destroy_subscription(sub)

    if not all_poses:
        log.error("No grasp candidates received within timeout")
        return None

    for i, pose in enumerate(all_poses):
        if client.check_reachable(pose):
            log.info(f"Selected grasp candidate {i} (best reachable)")
            return pose
        log.warn(f"Grasp candidate {i} is not reachable, trying next")

    log.error("No reachable grasp candidates found")
    return None


def pre_grasp_of(pose: Pose, offset: float = 0.15) -> Pose:
    # offset: trial and error 필요. 너무 멀면 경로 생성 실패, 너무 가까우면 충돌 위험. 15cm 정도가 적당할 것으로 예상
    q = pose.orientation
    r = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    approach_vector = r[:, 2]  # test_gpd_open3d: R = [binormal|axis|approach] → approach = col 2

    pre = Pose()
    pre.position.x = pose.position.x - offset * approach_vector[0]
    pre.position.y = pose.position.y - offset * approach_vector[1]
    pre.position.z = pose.position.z - offset * approach_vector[2]
    pre.orientation = pose.orientation
    return pre


def pick(client: MoveItClient, gripper: GripperController, grasp_pose: Pose) -> bool:
    log = client.node.get_logger()
    pre = pre_grasp_of(grasp_pose)

    log.info("Moving to pre-grasp")
    if client.move_to_pose(pre) != MoveResult.SUCCEEDED:
        return False
    log.info("Cartesian move to grasp")
    if client.cartesian_move(grasp_pose) != MoveResult.SUCCEEDED:
        return False
    log.info("Closing gripper")
    gripper.close('right')
    ga = GraspAssessment(client.node)
    if not ga.assess_on_close('right', 'ETC'):
        log.error("Grasp assessment failed — aborting pick")
        return False
    log.info("Retracting to pre-grasp")
    if client.cartesian_move(pre) != MoveResult.SUCCEEDED:
        return False

    return True


def place(client: MoveItClient, gripper: GripperController, place_pose: Pose) -> bool:
    log = client.node.get_logger()
    pre = pre_grasp_of(place_pose)

    log.info("Moving to pre-place")
    if client.move_to_pose(pre) != MoveResult.SUCCEEDED:
        return False
    # TODO: z floor check — cartesian_move이 테이블에 닿지 않도록
    log.info("Cartesian move to place")
    if client.cartesian_move(place_pose) != MoveResult.SUCCEEDED:
        return False
    log.info("Opening gripper")
    gripper.Open('right')
    log.info("Retracting from place")
    if client.cartesian_move(pre) != MoveResult.SUCCEEDED:
        return False

    return True
