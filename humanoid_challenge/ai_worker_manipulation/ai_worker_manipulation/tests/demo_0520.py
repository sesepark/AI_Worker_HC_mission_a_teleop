from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient
from ai_worker_manipulation.skill_primitives.environment import setup_environment
from ai_worker_manipulation.robot_interface.gripper_controller import GripperController
from ai_worker_manipulation.skill_primitives.pick_and_place import wait_for_grasp, pick, place
from geometry_msgs.msg import Pose

# Set True to skip GPD and use hardcoded poses for testing in RViz
DUMMY_MODE = True

# Top-down grasp: 180° around X → end-effector points down
# Adjust positions to match your object in RViz
DUMMY_GRASP = Pose()
DUMMY_GRASP.position.x = 0.45
DUMMY_GRASP.position.y = -0.20
DUMMY_GRASP.position.z = 0.55
DUMMY_GRASP.orientation.x = 1.0
DUMMY_GRASP.orientation.w = 0.0

DUMMY_PLACE = Pose()
DUMMY_PLACE.position.x = 0.45
DUMMY_PLACE.position.y = 0.20
DUMMY_PLACE.position.z = 0.55
DUMMY_PLACE.orientation.x = 1.0
DUMMY_PLACE.orientation.w = 0.0


def main():
    client = MoveItClient()
    log = client.node.get_logger()
    gripper = GripperController(node=client.node)
    setup_environment(client)

    gripper.open('right')
    client.move_to_home()

    if DUMMY_MODE:
        log.info("DUMMY_MODE: using hardcoded grasp and place poses")
        grasp_pose = DUMMY_GRASP
        place_pose = DUMMY_PLACE
    else:
        grasp_pose = wait_for_grasp(client)
        if grasp_pose is None:
            log.error("Failed to receive grasp result.")
            gripper.shutdown()
            client.shutdown()
            return
        place_pose = DUMMY_PLACE

    if not pick(client, gripper, grasp_pose):
        log.error("Pick failed — aborting")
        gripper.shutdown()
        client.shutdown()
        return

    if not place(client, gripper, place_pose):
        log.error("Place failed — aborting")
        gripper.shutdown()
        client.shutdown()
        return

    client.move_to_home()
    gripper.shutdown()
    client.shutdown()


if __name__ == '__main__':
    main()
