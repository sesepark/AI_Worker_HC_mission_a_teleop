from ai_worker_manipulation.robot_interface.moveit_client import MoveItClient, MoveResult
from ai_worker_manipulation.skill_primitives.environment import setup_environment
from ai_worker_manipulation.robot_interface.gripper_controller import GripperController
from geometry_msgs.msg import Pose
import time

INTERMEDIATE_JOINTS = [0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0]

# def run(log, label, result):
#     log.info(f'{label}: {result.value}')
#     if result != MoveResult.SUCCEEDED:
#         log.error(f'Stopping at: {label}')
#         return False
#     return True

def main():
    client = MoveItClient()
    log = client.node.get_logger()

    setup_environment(client)
    log.info('Environment Ready')

    log.info(f'Initial joints: {client.get_joint_positions()}')
    log.info(f'Initial pose:   {client.get_current_pose()}')
    gripper = GripperController(node=client.node)
    gripper.open('right')
    dummy_pose = Pose()
    dummy_pose.position.x = 0.35
    dummy_pose.position.y = -0.25
    dummy_pose.position.z = 0.8
    dummy_pose.orientation.w = 1.0

    # if not run(log, 'move_to_home', client.move_to_home()):
    #     client.shutdown(); return
    # if not run(log, 'move_to_joints', client.move_to_joints(INTERMEDIATE_JOINTS)):
    #     client.shutdown(); return
    # if not run(log, 'move_to_pose',   client.move_to_pose(dummy_pose)):
    #     client.shutdown(); return
    # dummy_pose.position.z = 0.50
    # if not run(log, 'cartesian_move', client.cartesian_move(dummy_pose)):
    #     client.shutdown(); return
    # if not run(log, 'move_to_home',   client.move_to_home()):
    #     client.shutdown(); return

    log.info(f'move_to_home:   {client.move_to_home().value}')
    #log.info(f'move_to_joints: {client.move_to_joints(INTERMEDIATE_JOINTS).value}')
    log.info(f'move_to_pose:   {client.move_to_pose(dummy_pose).value}')
    dummy_pose.position.z = 0.75
    log.info(f'cartesian_move: {client.cartesian_move(dummy_pose).value}')
    
    
    log.info('Opening gripper')
    gripper.open('right')
    time.sleep(1.5)
    log.info('Closing gripper')
    gripper.close('right')
    time.sleep(1.5)
    
    log.info(f'move_to_home:   {client.move_to_home().value}')

    log.info('YAY')
    client.shutdown()

if __name__ == '__main__':
    main()
