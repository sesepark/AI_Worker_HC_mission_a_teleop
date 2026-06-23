#!/usr/bin/env python3
"""Mission A manipulation 실 서버 — mock_manipulation_a 의 drop-in 대체.

mock_manipulation_a 의 **외부 계약(이름·타입·시맨틱)을 그대로** 노출하고, 내부는 검증된
MoveIt primitive(test_pick_with_perception / test_place)로 구동한다. 새 픽/플레이스 로직 없음.

외부 계약(= mock 과 동일):
  - action  `move_to_scan_pose` (mission_interfaces/MoveToScanPose) → move_to_joints(CAPTURE_JOINTS, RIGHT).
  - sub     `/attach_cmd`  (std_msgs/String, "pick")  → PickSkill 픽.
  - sub     `/detach_cmd`  (std_msgs/String, class)   → PlaceSkill 플레이스.
  - pub     `/attached_object` (std_msgs/String)      → 파지 class / "".
  - pub     `/manipulator_state` (std_msgs/String)    → "IDLE"/"BUSY".
  - sub     `/perception/task_list` (GetTaskList.Response) → class 미러(mock 과 동일 로직).
  - sub     `/perception/wrist/target_one_pose` (PoseStamped) → 픽 중심 pose.

그래스프 래치 보존(불변식): `/attached_object`=class 는 **PickResult.SUCCESS(=GraspSkill.assess_stable
확정) 시점에만** 발행한다. 파지 실패 시 미발행 → FSM A3_PICK timeout→RECOVERY(C2, 오선언 0).

실행 환경: `ai_worker` 컨테이너(ffw bringup + MoveIt + pymoveit2). `mission`·`mission_interfaces`
패키지도 소싱되어야 한다(task_list 미러·인터페이스). mock 과 토글 공존(둘 중 하나만 기동).
"""
from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

from mission_interfaces.action import MoveToScanPose
from mission_interfaces.srv import GetTaskList
from mission.task_list import TaskList   # class 미러(mock_manipulation_a 와 동일 로직, 차감 키 정합)

from manipulation.robot_interface.moveit_client import MoveItClient, Arm, MoveResult
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.planning_scene import setup_zone_a, clear_all_objects
from manipulation.skill_primitives.grasp_assessment import GraspAssessment
from manipulation.skill_primitives.grasp_skill import GraspSkill
from manipulation.skill_primitives.pick_skill import PickSkill, PickResult
from manipulation.skill_primitives.place_skill import PlaceSkill, PlaceResult
from manipulation.skill_primitives.planning_filter import PlanningFilter
from manipulation.skill_primitives.mission_a_grasp_adapter import build_mission_a_grasp_pose


# 검증된 상수 (test_pick_with_perception / test_place)
CAPTURE_JOINTS = [-0.196033, -1.002742, 0.545092, -2.026014, -2.491690, 0.901389, -1.553559]
CARRY_Z = 1.150
PLACE_X, PLACE_Y, PLACE_Z = 0.270, -0.10, 0.880


class MissionAManipulationServer(Node):
    def __init__(self) -> None:
        super().__init__('mission_a_manipulation_server')

        self._cbg = ReentrantCallbackGroup()

        # --- 외부 계약(mock_manipulation_a 와 동일 이름·타입) ---
        self.pub_attached = self.create_publisher(String, '/attached_object', 10)
        self.pub_manip = self.create_publisher(String, '/manipulator_state', 10)
        self.sub_attach = self.create_subscription(
            String, '/attach_cmd', self._on_attach, 10, callback_group=self._cbg)
        self.sub_detach = self.create_subscription(
            String, '/detach_cmd', self._on_detach, 10, callback_group=self._cbg)
        self.sub_task = self.create_subscription(
            GetTaskList.Response, '/perception/task_list', self._on_task, 10,
            callback_group=self._cbg)
        self.sub_target = self.create_subscription(
            PoseStamped, '/perception/wrist/target_one_pose', self._on_target, 10,
            callback_group=self._cbg)
        self.srv_scan = ActionServer(
            self, MoveToScanPose, 'move_to_scan_pose', self._exec_scan,
            callback_group=self._cbg)

        # --- 검증된 manip primitive (test 구성 재사용) ---
        # manage_executor=False: 이 서버 main()이 단일 executor 로 노드를 spin 한다.
        # (MoveItClient 가 자체 executor 를 또 만들면 한 노드를 두 executor 가 spin → action_client.c:659.)
        self.client = MoveItClient(self, manage_executor=False)
        self.gripper = GripperInterface(self)
        self.assess = GraspAssessment(self)
        self.grasp = GraspSkill(self, self.gripper, self.assess)
        self.pfilter = PlanningFilter(self.client, log=self.get_logger().info)
        self.pick = PickSkill(self, self.client, self.gripper, self.grasp, self.pfilter)
        self.place = PlaceSkill(self, self.client, self.gripper, self.pfilter)

        # --- 상태 ---
        self._mirror = TaskList()            # task_list class 미러(픽 class 결정)
        self._current: str | None = None     # 현재 파지 class(래치)
        self._latest_target: Pose | None = None
        self._pending_attach = False         # class/target 미준비 시 보류(mock _pending_attach 시맨틱)
        self._busy = threading.Lock()        # scan/pick/place MoveIt 동작 직렬화
        self._ready = False                  # startup() 완료 전엔 IDLE 미발행(FSM INIT 보류)

        # planning scene 초기화는 spin 이 돌아야 가능 → startup() 으로 이동.
        self.create_timer(0.2, self._pub_manip, callback_group=self._cbg)
        self.create_timer(0.1, self._tick_pending, callback_group=self._cbg)

    # ------------------------------------------------------------------ #
    def startup(self) -> None:
        """단일 executor 가 spin 중일 때 호출 — MoveIt 준비 대기 + planning scene 초기화 후 IDLE 발행 개시."""
        self.client.wait_until_ready()       # move_group 서버 + joint_states 준비(외부 executor 가 spin)
        clear_all_objects(self.client)
        setup_zone_a(self.client)
        self._ready = True
        self.get_logger().info('mission_a_manipulation_server ready (real MoveIt)')

    def _pub_manip(self) -> None:
        # 준비 전 또는 동작 중이면 BUSY, 준비+유휴면 IDLE(FSM INIT 통과 조건). mock은 항상 IDLE.
        busy = (not self._ready) or self._busy.locked()
        self.pub_manip.publish(String(data='BUSY' if busy else 'IDLE'))

    def _on_task(self, msg: GetTaskList.Response) -> None:
        if self._mirror.is_empty():
            parts = [{'name': it.name, 'count': it.count} for it in msg.parts]
            self._mirror.build_from_ocr_parts(parts)
            if not self._mirror.is_empty():
                self.get_logger().info(f'[manip] task 미러: {self._mirror}')

    def _on_target(self, msg: PoseStamped) -> None:
        self._latest_target = msg.pose

    # --- A2_SCAN_POSE: scan/capture 자세 ---
    def _exec_scan(self, goal_handle):
        with self._busy:
            r = self.client.move_to_joints(
                CAPTURE_JOINTS, arm=Arm.RIGHT, velocity=0.2, acceleration=0.2)
        ok = (r == MoveResult.SUCCEEDED)
        goal_handle.succeed()
        result = MoveToScanPose.Result()
        result.success = ok
        result.message = 'scan pose reached' if ok else f'move_to_joints={r}'
        self.get_logger().info(f'[manip] move_to_scan_pose -> success={ok}')
        return result

    # --- A3_PICK: /attach_cmd 수신 → 픽 ---
    def _on_attach(self, msg: String) -> None:
        cls = self._mirror.next_target_class()
        if not cls or self._latest_target is None:
            self._pending_attach = True
            self.get_logger().warn('[manip] /attach_cmd — class/target 미준비, 보류')
            return
        self._do_pick(cls)

    def _tick_pending(self) -> None:
        if self._pending_attach:
            cls = self._mirror.next_target_class()
            if cls and self._latest_target is not None:
                self._pending_attach = False
                self.get_logger().info('[manip] 보류 /attach_cmd 처리')
                self._do_pick(cls)

    def _do_pick(self, cls: str) -> None:
        with self._busy:
            center = self._latest_target
            grasp_pose = build_mission_a_grasp_pose(center)
            res = self.pick.pick(grasp_pose, arm=Arm.RIGHT, object_name=cls)
            if res == PickResult.SUCCESS:
                # 래치: grasp 확정(assess_stable) 후에만 attached 발행 → C2 게이트 성립.
                self._current = cls
                self.pub_attached.publish(String(data=cls))
                self.get_logger().info(f'[manip] 파지 성공 → /attached_object={cls}')
                # carry 상승(검증 흐름: x,y 유지 + CARRY_Z, grasp orientation 유지)
                carry = Pose()
                carry.position.x = center.position.x
                carry.position.y = center.position.y
                carry.position.z = CARRY_Z
                carry.orientation = grasp_pose.orientation
                self.client.move_to_pose(carry, arm=Arm.RIGHT, velocity=0.3, acceleration=0.3)
            else:
                # 파지 실패 → attached 미발행(빈 상태 유지) → FSM A3_PICK timeout→RECOVERY(오선언 0).
                self.get_logger().warn(f'[manip] 파지 실패({res}) — /attached_object 미발행(C2)')

    # --- A3_PLACE: /detach_cmd 수신 → 플레이스 ---
    def _on_detach(self, msg: String) -> None:
        with self._busy:
            if not self._current:
                self.get_logger().warn('[manip] /detach_cmd — 파지 객체 없음(무시)')
                return
            place_pose = Pose()
            place_pose.position.x = PLACE_X
            place_pose.position.y = PLACE_Y
            place_pose.position.z = PLACE_Z
            place_pose.orientation.w = 1.0
            res = self.place.place(place_pose, arm=Arm.RIGHT)
            if res == PlaceResult.SUCCESS:
                # PlaceSkill 이 gripper.open()으로 release. 계약상 해제 발행 + 미러 차감.
                self.pub_attached.publish(String(data=''))
                self._mirror.decrement(self._current)
                self.get_logger().info(
                    f'[manip] 적재 완료 → /attached_object="" ({self._current}), '
                    f'미러 잔여 {self._mirror.total_remaining()}')
                self._current = None
            else:
                # place 실패 → 해제 미발행 → FSM A3_PLACE timeout→RECOVERY(무차감).
                self.get_logger().warn(f'[manip] place 실패({res}) — 해제 미발행')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionAManipulationServer()

    # 단일 executor 로 노드를 spin. 모든 엔티티는 노드 __init__ 에서 이미 생성됨
    # (MoveItClient 의 move_group action client 포함) → spin 이전 생성 보장.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.startup()          # executor 가 spin 중 → MoveIt 준비 대기 + 씬 초기화 + IDLE 개시
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.client.destroy()   # MoveItClient: manage_executor=False 라 내부 executor 없음(no-op 안전)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
