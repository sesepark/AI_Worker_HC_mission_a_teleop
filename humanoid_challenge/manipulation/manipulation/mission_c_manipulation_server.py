#!/usr/bin/env python3
"""Mission C manipulation 실 서버 — mission_a_manipulation_server 의 C(peg 삽입·양팔) 변형.

A 서버의 외부 계약·구조를 그대로 따르되, **이미 구현된 dual-arm primitive를 그대로 사용**해
(1) 양팔 선택, (2) peg 타깃 삽입(hover→Cartesian 하강→gripper open)을 배선한다. 새 모션 로직 없음.
  - dual-arm: MoveItClient(Arm.LEFT/RIGHT) + select_arm(y) + PickSkill/PlaceSkill(arm=...) — 기구현.
  - 삽입 모션: PlaceSkill.place(..., local_mode='hover') = "hover 위 → Cartesian 하강 → open → retreat"
    = peg 삽입. (기구현, A 도 동일 호출 — A 는 단일팔·고정 place 좌표로만 사용.)

외부 계약(= mock/A 와 동일):
  - action  `move_to_scan_pose` → 양팔 capture 자세(CAPTURE_JOINTS_R/L).
  - sub `/attach_cmd`("pick") → PickSkill 픽(arm=select_arm(target.y), build_c_grasp_pose).
  - sub `/detach_cmd`(class) → PlaceSkill 삽입(peg=insert_target, arm=pick arm, hover).
  - pub `/attached_object`(class/"") , `/manipulator_state`("IDLE"/"BUSY").
  - sub `/perception/task_list`, `/perception/wrist/target_one_pose`.
C 신규 입력(FSM mission_c 가 통지):
  - sub `/mission_c/insert_target`(PoseStamped) — 선택된 peg 상단 중심.
  - sub `/mission_c/insert_arm`(String) — 참고(cross-check). 서버는 pick arm 으로 일관 동작.

그래스프 래치 보존(A 동일): `/attached_object`=class 는 PickResult.SUCCESS(=assess_stable) 시점에만.

좌표 정합(분석 R4): peg 좌표는 FSM 이 perception(`/perception/head/pipe_top_centers`, 학습/preset)
  에서 받아 insert_target 으로 전달 → 서버는 그 좌표에 삽입(planning_scene 의 옛 ZONE_C_PEG 상수에
  의존하지 않음). 단 충돌 씬은 setup_zone_c 로 등록.

⚠️ 코디네이션 노트: pick arm 은 pick 타깃 y(select_arm)로, peg 는 FSM 이 순차 배정한다. 실 로봇에선
  배정 peg 가 pick arm 의 가동범위 밖일 수 있다 → FSM peg 배정을 pick arm 에 맞추거나(권장) base
  측방이동(MoveBaseLateral)로 보정. 본 서버는 pick arm 으로 일관 삽입하고 불일치 시 경고만 낸다.

실행 환경: ai_worker(ffw bringup + MoveIt + pymoveit2) + mission/mission_interfaces 소싱. 실 모션은
  사용자 감독(저속·E-stop). 본 파일은 코드/빌드/계약 검증까지(실 MoveIt 모션은 로봇에서).
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
from mission.task_list import TaskList

from manipulation.robot_interface.moveit_client import MoveItClient, Arm, MoveResult
from manipulation.robot_interface.gripper_controller import GripperInterface
from manipulation.robot_interface.planning_scene import (
    setup_zone_c, setup_zone_c_table, clear_all_objects)
from manipulation.skill_primitives.grasp_assessment import GraspAssessment
from manipulation.skill_primitives.grasp_skill import GraspSkill
from manipulation.skill_primitives.pick_skill import PickSkill, PickResult
from manipulation.skill_primitives.place_skill import PlaceSkill, PlaceResult
from manipulation.skill_primitives.planning_filter import PlanningFilter
from manipulation.skill_primitives.mission_c_arm_selector import select_arm
from manipulation.skill_primitives.mission_c_grasp_adapter import (
    build_c_grasp_pose, CARRY_Z)


# 양팔 capture 자세(test_base_C / test_capture_to_pick_C 와 동일값).
CAPTURE_JOINTS_R = [-0.514537, -1.079939,  0.611448, -2.036518, -2.695534,  1.082374, -1.580207]
CAPTURE_JOINTS_L = [-0.514537,  1.079939, -0.611448, -2.036518,  2.695534,  1.082374,  1.580207]


class MissionCManipulationServer(Node):
    def __init__(self) -> None:
        super().__init__('mission_c_manipulation_server')

        self._cbg = ReentrantCallbackGroup()

        # arm_mode: 'right'(현 단계 기본 — A 처럼 우완 단일팔) | 'left' | 'auto'(select_arm 양팔).
        #   right/left = pick·insert·scan 모두 해당 팔만 사용. auto = pick 타깃 y 로 양팔 자동 선택.
        self.arm_mode = str(self.declare_parameter('arm_mode', 'right').value).strip().lower()
        if self.arm_mode not in ('right', 'left', 'auto'):
            self.get_logger().warn(f"arm_mode={self.arm_mode!r} 미지원 → 'right' 사용")
            self.arm_mode = 'right'

        # insert_dry_run: /detach_cmd 시 정밀 peg 삽입을 생략하고 제자리 gripper open(항상 성공)
        #   으로 release → 가동범위/계획 실패 stall 없이 FSM 전 사이클 다회 시험. 기본 OFF(무회귀).
        #   실제 삽입 검증 아님(로그에 [DRY-RUN] 명시). FSM(mission_c) insert_dry_run 과 함께 사용.
        self.insert_dry_run = bool(
            self.declare_parameter('insert_dry_run', False).value)
        # collision_table_only=True: planning scene 에 벤치(body+top)만 등록하고
        #   볼트/peg 충돌 실린더는 제외. 너트가 볼트/peg 위에 있어 grasp 직하강이
        #   실린더와 충돌(Pilz LIN INVALID_MOTION_PLAN)하는 모순을 회피(픽 가능).
        #   False 면 기존 setup_zone_c(볼트/peg 포함).
        self.collision_table_only = bool(
            self.declare_parameter('collision_table_only', True).value)
        # pick_dry_run=True: 실제 파지(접근/하강/grasp) 생략, 즉시 성공 처리(/attached_object 발행).
        #   베이스 이동 시퀀스 검증용 — grasp/reach 문제와 분리해 FSM 을 끝까지 진행시킨다.
        self.pick_dry_run = bool(
            self.declare_parameter('pick_dry_run', False).value)

        # --- 외부 계약(A/mock 과 동일 이름·타입) ---
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
        # C 신규: FSM mission_c 의 peg 타깃·팔 통지.
        self.sub_insert_target = self.create_subscription(
            PoseStamped, '/mission_c/insert_target', self._on_insert_target, 10,
            callback_group=self._cbg)
        self.sub_insert_arm = self.create_subscription(
            String, '/mission_c/insert_arm', self._on_insert_arm, 10,
            callback_group=self._cbg)
        self.srv_scan = ActionServer(
            self, MoveToScanPose, 'move_to_scan_pose', self._exec_scan,
            callback_group=self._cbg)

        # --- 검증된 dual-arm primitive (기구현 재사용) ---
        self.client = MoveItClient(self, manage_executor=False)
        self.gripper = GripperInterface(self)
        self.assess = GraspAssessment(self)
        self.grasp = GraspSkill(self, self.gripper, self.assess)
        self.pfilter = PlanningFilter(self.client, log=self.get_logger())
        self.pick = PickSkill(self, self.client, self.gripper, self.grasp, self.pfilter)
        self.place = PlaceSkill(self, self.client, self.gripper, self.pfilter)

        # --- 상태 ---
        self._mirror = TaskList()
        self._current: str | None = None      # 현재 파지 class(래치)
        self._current_arm: Arm = Arm.RIGHT     # 현재 사이클 팔(pick→insert 일관)
        self._latest_target: Pose | None = None
        self._latest_insert_target: Pose | None = None
        self._latest_insert_arm: str | None = None
        self._pending_attach = False
        self._busy = threading.Lock()
        self._ready = False
        self._done_logged = False              # 미러 0 DONE 로그 1회 보장

        self.create_timer(0.2, self._pub_manip, callback_group=self._cbg)
        self.create_timer(0.1, self._tick_pending, callback_group=self._cbg)

    # ------------------------------------------------------------------ #
    def startup(self) -> None:
        self.client.wait_until_ready()
        clear_all_objects(self.client)
        if self.collision_table_only:
            setup_zone_c_table(self.client)
            self.get_logger().info(
                '[manip-c] planning scene: 벤치만 등록(볼트/peg 충돌 제외) '
                '— grasp 직하강 충돌 회피. (collision_table_only=True)')
        else:
            setup_zone_c(self.client)
        # 사용 팔 그리퍼만 open(파지 준비). right/left=해당 팔만, auto=양팔.
        try:
            if self.arm_mode in ('right', 'auto'):
                self.gripper.open('right')
            if self.arm_mode in ('left', 'auto'):
                self.gripper.open('left')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'[manip-c] 그리퍼 초기 open 경고: {exc}')
        self._ready = True
        self.get_logger().info(
            f'mission_c_manipulation_server ready (real MoveIt, arm_mode={self.arm_mode}, '
            f'insert_dry_run={self.insert_dry_run})')

    def _resolve_arm(self, y: float) -> Arm:
        """arm_mode 에 따라 팔 결정: 'right'/'left' 고정, 'auto'=select_arm(y) 양팔."""
        if self.arm_mode == 'right':
            return Arm.RIGHT
        if self.arm_mode == 'left':
            return Arm.LEFT
        return select_arm(y)

    def _pub_manip(self) -> None:
        busy = (not self._ready) or self._busy.locked()
        self.pub_manip.publish(String(data='BUSY' if busy else 'IDLE'))

    def _on_task(self, msg: GetTaskList.Response) -> None:
        if self._mirror.is_empty():
            parts = [{'name': it.name, 'count': it.count} for it in msg.parts]
            self._mirror.build_from_ocr_parts(parts)
            if not self._mirror.is_empty():
                self.get_logger().info(f'[manip-c] task 미러: {self._mirror}')

    def _on_target(self, msg: PoseStamped) -> None:
        self._latest_target = msg.pose

    def _on_insert_target(self, msg: PoseStamped) -> None:
        self._latest_insert_target = msg.pose

    def _on_insert_arm(self, msg: String) -> None:
        self._latest_insert_arm = msg.data

    # --- C2_SCAN_POSE: capture 자세(사용 팔만; auto 면 양팔) ---
    def _exec_scan(self, goal_handle):
        results = []
        with self._busy:
            if self.arm_mode in ('right', 'auto'):
                results.append(('R', self.client.move_to_joints(
                    CAPTURE_JOINTS_R, arm=Arm.RIGHT, velocity=0.2, acceleration=0.2)))
            if self.arm_mode in ('left', 'auto'):
                results.append(('L', self.client.move_to_joints(
                    CAPTURE_JOINTS_L, arm=Arm.LEFT, velocity=0.2, acceleration=0.2)))
        ok = all(r == MoveResult.SUCCEEDED for _, r in results)
        goal_handle.succeed()
        result = MoveToScanPose.Result()
        result.success = ok
        result.message = ('capture pose reached' if ok
                          else ' '.join(f'{s}={r}' for s, r in results))
        self.get_logger().info(
            f'[manip-c] move_to_scan_pose(arm_mode={self.arm_mode}) -> success={ok}')
        return result

    # --- C3_PICK: /attach_cmd → 픽(arm=select_arm(target.y)) ---
    def _on_attach(self, msg: String) -> None:
        cls = self._mirror.next_target_class()
        if not cls or self._latest_target is None:
            self._pending_attach = True
            self.get_logger().warn('[manip-c] /attach_cmd — class/target 미준비, 보류')
            return
        self._do_pick(cls)

    def _tick_pending(self) -> None:
        if self._pending_attach:
            cls = self._mirror.next_target_class()
            if cls and self._latest_target is not None:
                self._pending_attach = False
                self.get_logger().info('[manip-c] 보류 /attach_cmd 처리')
                self._do_pick(cls)

    def _do_pick(self, cls: str) -> None:
        with self._busy:
            if self.pick_dry_run:
                # 실제 파지 생략 → 성공 처리(베이스 이동 시퀀스 검증용). 팔/grasp 미수행.
                self._current = cls
                self.pub_attached.publish(String(data=cls))
                self.get_logger().info(
                    f'[manip-c][DRY-PICK] 실제 파지 생략 → /attached_object={cls} '
                    '(베이스 이동 검증용)')
                return
            center = self._latest_target
            arm = self._resolve_arm(center.position.y)   # arm_mode(기본 right) 또는 select_arm
            self._current_arm = arm
            grasp_pose = build_c_grasp_pose(center)
            res = self.pick.pick(grasp_pose, arm=arm, object_name=cls)
            if res == PickResult.SUCCESS:
                self._current = cls
                self.pub_attached.publish(String(data=cls))
                self.get_logger().info(
                    f'[manip-c] 파지 성공(arm={arm.value}) → /attached_object={cls}')
                # carry 상승(x,y 유지 + CARRY_Z, grasp orientation 유지) — A 동일.
                carry = Pose()
                carry.position.x = center.position.x
                carry.position.y = center.position.y
                carry.position.z = CARRY_Z
                carry.orientation = grasp_pose.orientation
                self.client.move_to_pose(carry, arm=arm, velocity=0.3, acceleration=0.3)
            else:
                self.get_logger().warn(
                    f'[manip-c] 파지 실패({res}) — /attached_object 미발행(C2)')

    def _emit_done_if_complete(self) -> None:
        """미러 잔여 0(전 부품 적재 완료) 도달 시 DONE 성공 로그 1회. 실/dry 공통."""
        if self._done_logged:
            return
        if self._mirror.is_complete():   # built AND total_remaining()==0
            self._done_logged = True
            bar = '=' * 52
            self.get_logger().info(bar)
            self.get_logger().info('[DONE] 미션 수행 성공 — 미러 잔여 0 (전 부품 적재 완료)')
            self.get_logger().info(bar)

    # --- C3_INSERT: /detach_cmd → peg 삽입(hover) / dry-run release ---
    def _on_detach(self, msg: String) -> None:
        with self._busy:
            if not self._current:
                self.get_logger().warn('[manip-c] /detach_cmd — 파지 객체 없음(무시)')
                return
            arm = self._current_arm

            # dry-run: 정밀 삽입 생략, 제자리 gripper open(MoveIt 無, 항상 성공) → release.
            #   insert_target 미수신이어도 진행(stall 차단). 실제 삽입 검증 아님.
            if self.insert_dry_run:
                try:
                    self.gripper.open(arm.value)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f'[manip-c] dry-run gripper open 경고: {exc}')
                self.pub_attached.publish(String(data=''))
                self._mirror.decrement(self._current)
                self.get_logger().info(
                    f'[INSERT][DRY-RUN] 정밀 삽입 생략, 제자리 release(arm={arm.value}) '
                    f'→ /attached_object="" ({self._current}), '
                    f'미러 잔여 {self._mirror.total_remaining()}')
                self._current = None
                self._latest_insert_target = None
                self._emit_done_if_complete()
                return

            # --- 실 경로(정밀 peg 삽입) ---
            if self._latest_insert_target is None:
                self.get_logger().warn(
                    '[manip-c] /detach_cmd — insert_target 미수신 → 삽입 보류(FSM timeout→RECOVERY)')
                return
            # 코디네이션 cross-check: FSM 이 통지한 insert_arm 과 pick arm 불일치 경고.
            if self._latest_insert_arm and self._latest_insert_arm != arm.value:
                self.get_logger().warn(
                    f'[manip-c] insert_arm({self._latest_insert_arm}) != pick arm({arm.value}) '
                    f'— pick arm 으로 삽입(가동범위 확인 필요)')
            peg = self._latest_insert_target
            insert_pose = Pose()
            insert_pose.position.x = peg.position.x
            insert_pose.position.y = peg.position.y
            insert_pose.position.z = peg.position.z
            # top-down 삽입 자세(grasp 와 동일 규약).
            insert_pose.orientation.w = 1.0
            # 기구현 PlaceSkill hover: hover 위 → Cartesian 하강 → gripper open(삽입) → retreat.
            res = self.place.place(insert_pose, arm=arm, local_mode='hover')
            if res == PlaceResult.SUCCESS:
                self.pub_attached.publish(String(data=''))
                self._mirror.decrement(self._current)
                self.get_logger().info(
                    f'[manip-c] 삽입 완료(arm={arm.value}) → /attached_object="" ({self._current}), '
                    f'미러 잔여 {self._mirror.total_remaining()}')
                self._current = None
                self._latest_insert_target = None
                self._emit_done_if_complete()
            else:
                self.get_logger().warn(f'[manip-c] 삽입 실패({res}) — 해제 미발행')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionCManipulationServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.startup()
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.client.destroy()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
