from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient
from ai_worker_manipulation.skill_primitives.environment import setup_environment
from ai_worker_manipulation.robot_interface.gripper_controller import GripperController
from geometry_msgs.msg import Pose


def _pose(x, y, z):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.w = 1.0
    return p


# demo_0513에서 IK 해 확인된 좌표
PRE_GRASP = _pose(0.35, -0.25, 0.80)
GRASP     = _pose(0.35, -0.25, 0.75)
PRE_PLACE = _pose(0.35,  0.0, 0.80)
PLACE     = _pose(0.35,  0.0, 0.75)


def main():
    client  = MoveItClient()
    log     = client.node.get_logger()
    gripper = GripperController(node=client.node)
    setup_environment(client)

    gripper.open('right')
    client.move_to_home()

    try:
        # ── Pick ──────────────────────────────────────────────
        log.info("=== PICK ===")

        log.info("pre-grasp 이동")
        if client.move_to_pose(PRE_GRASP).value != "succeeded":
            log.error("pre-grasp 이동 실패")
            return

        log.info("grasp 위치로 cartesian 이동")
        if client.cartesian_move(GRASP).value != "succeeded":
            log.error("grasp cartesian 이동 실패")
            return

        log.info("그리퍼 닫기")
        gripper.close('right')

        log.info("pre-grasp 복귀")
        client.cartesian_move(PRE_GRASP)

        # ── Place ─────────────────────────────────────────────
        log.info("=== PLACE ===")

        log.info("pre-place 이동")
        if client.move_to_pose(PRE_PLACE).value != "succeeded":
            log.error("pre-place 이동 실패")
            return

        log.info("place 위치로 cartesian 이동")
        if client.cartesian_move(PLACE).value != "succeeded":
            log.error("place cartesian 이동 실패")
            return

        log.info("그리퍼 열기")
        gripper.open('right')

        log.info("pre-place 복귀")
        client.cartesian_move(PRE_PLACE)

        log.info("=== DONE ===")

    finally:
        gripper.open('right')
        client.move_to_home()
        gripper.shutdown()
        client.shutdown()


if __name__ == '__main__':
    main()