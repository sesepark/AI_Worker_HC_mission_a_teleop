#!/usr/bin/env python3
"""Real Navigation (Mission A) — MoveBaseLateral Service 서버.

mock_navigation_a 와 동일한 외부 계약(서비스 이름/타입/시맨틱)을 노출하되, 실제로 SG2 swerve
베이스를 odom 폐루프로 측방(좌/우) strafe 이동시킨다. mock 만 끄고 본 노드를 기동하면 드롭인 교체.

  - Request{direction:"left"|"right", distance_mm} → cmd_vel(Twist.linear.y) 발행, /odom 로봇프레임
    측방 델타가 목표(distance_mm/1000)에 도달하면 정지 → Response{arrived, lateral_error_mm, message}.
  - SG2 베이스는 swerve(홀로노믹)라 Twist.linear.y(측방)를 네이티브 지원.
  - 콜백 내에서 이동 완료까지 블로킹(동기). MTE+ReentrantCallbackGroup 이라 블록 중에도 /odom 콜백은
    다른 스레드에서 계속 self.current_pose 를 갱신한다(콜백 내 spin 금지).

이동 폐루프 로직(odom 로봇프레임 투영, sign 매핑, wrong-direction/타임아웃 가드)은
ffw_manual_tools/sg2_lateral_jog.py (Apache-2.0, Copyright 2026 ROBOTIS CO., LTD.) 에서
가져와 단발 노드 → 서비스 콜백으로 재구현한 것이다.

안전(보조 AI 검증 경계):
  - odom 신선도 가드: wait_for_odom_sec 내 /odom 미수신이면 절대 무이동(arrived=False, "no odom").
  - 모든 종료 경로(정상/예외/타임아웃/잘못된 방향/shutdown)에서 zero-Twist 발행(베이스 속도 latch 방지).
  - distance_mm <= 0 은 즉시 무이동 성공(콜드 디스커버리 첫 호출 테스트 안전 경로).
  - fail_inject 파라미터로 강제 arrived=False(=FSM RECOVERY) 주입 가능(mock fail_arrive 대응).
"""
from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from mission_interfaces.srv import MoveBaseLateral


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class MoveBaseLateralServer(Node):
    def __init__(self) -> None:
        super().__init__('move_base_lateral')

        # --- Parameters (sg2_lateral_jog 기본값 보존) ---
        self.service_name = str(
            self.declare_parameter('service_name', 'move_base_lateral').value)
        self.speed = abs(float(self.declare_parameter('speed', 0.12).value))
        self.cmd_vel_topic = str(
            self.declare_parameter('cmd_vel_topic', '/cmd_vel').value)
        self.odom_topic = str(self.declare_parameter('odom_topic', '/odom').value)
        self.rate_hz = float(self.declare_parameter('rate_hz', 20.0).value)
        # max_duration_sec 는 FSM base_move_timeout_sec(기본 30) 보다 작게 유지할 것.
        self.max_duration_sec = float(
            self.declare_parameter('max_duration_sec', 12.0).value)
        self.wrong_direction_tolerance = float(
            self.declare_parameter('wrong_direction_tolerance', 0.05).value)
        self.use_odom_stop = bool(
            self.declare_parameter('use_odom_stop', True).value)
        self.wait_for_odom_sec = float(
            self.declare_parameter('wait_for_odom_sec', 3.0).value)
        self.stop_brake_cycles = int(
            self.declare_parameter(
                'stop_brake_cycles', max(5, int(self.rate_hz * 0.5))).value)
        self.fail_inject = bool(
            self.declare_parameter('fail_inject', False).value)

        if self.rate_hz <= 0.0:
            self.rate_hz = 20.0
        if self.speed <= 0.0:
            self.speed = 0.12

        self._period = 1.0 / self.rate_hz

        # --- 상태 ---
        self.current_pose: tuple[float, float, float] | None = None  # (x, y, yaw)
        self._pose_lock = threading.Lock()
        # 단일 in-flight 이동 보장(방어): FSM 은 상태당 1회만 호출하지만 동시 호출 시 직렬화.
        self._move_lock = threading.Lock()

        # --- 콜백 그룹: service 와 odom 을 분리해, 블로킹 콜백 중에도 odom 갱신 ---
        self._srv_cbg = ReentrantCallbackGroup()
        self._odom_cbg = ReentrantCallbackGroup()

        # cmd_vel/odom QoS(BEST_EFFORT) — sg2_lateral_jog 와 동일. RELIABLE 퍼블리셔와도 호환.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, qos)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos,
            callback_group=self._odom_cbg)

        # 서비스는 __init__ 에서(무거운 init 없이) 생성 → DDS 그래프에 조기 광고(콜드 디스커버리 완화).
        self.srv = self.create_service(
            MoveBaseLateral, self.service_name, self._on_request,
            callback_group=self._srv_cbg)

        self.get_logger().info(
            f'move_base_lateral ready (Service "{self.service_name}"; '
            f'speed={self.speed:.2f}m/s, rate={self.rate_hz:.0f}Hz, '
            f'use_odom_stop={self.use_odom_stop}, fail_inject={self.fail_inject}, '
            f'cmd_vel={self.cmd_vel_topic}, odom={self.odom_topic})')

    # ------------------------------------------------------------------ #
    # Odom
    # ------------------------------------------------------------------ #
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        with self._pose_lock:
            self.current_pose = (p.x, p.y, yaw)

    def _get_pose(self) -> tuple[float, float, float] | None:
        with self._pose_lock:
            return self.current_pose

    @staticmethod
    def _robot_frame_delta(
            start: tuple[float, float, float],
            cur: tuple[float, float, float]) -> tuple[float, float]:
        """start 자세 기준 로봇프레임 (전진, 좌측) 델타. (sg2_lateral_jog 와 동일 투영)"""
        sx, sy, syaw = start
        cx, cy, _ = cur
        dx, dy = cx - sx, cy - sy
        forward = math.cos(syaw) * dx + math.sin(syaw) * dy
        left = -math.sin(syaw) * dx + math.cos(syaw) * dy
        return forward, left

    def _publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _wait_for_odom(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._get_pose() is not None:
                return True
            time.sleep(0.02)
        return self._get_pose() is not None

    # ------------------------------------------------------------------ #
    # Service callback (blocking full move)
    # ------------------------------------------------------------------ #
    def _on_request(self, request, response):
        direction = str(request.direction).strip().lower()
        distance_mm = float(request.distance_mm)

        # 1) 방향 검증
        if direction not in ('left', 'right'):
            self._publish_stop()
            response.arrived = False
            response.lateral_error_mm = 0.0
            response.message = f'invalid direction: {request.direction!r}'
            self.get_logger().warning(f'[nav] {response.message}')
            return response

        sign = 1.0 if direction == 'left' else -1.0
        distance_m = abs(distance_mm) / 1000.0

        # 2) 강제 실패 주입(테스트) — 무이동
        if self.fail_inject:
            self._publish_stop()
            response.arrived = False
            response.lateral_error_mm = 0.0
            response.message = 'navigate injected-failure'
            self.get_logger().warning(f'[nav] {response.message}')
            return response

        # 3) distance<=0 → 즉시 무이동 성공(콜드 디스커버리 첫 호출 안전 경로)
        if distance_m <= 1e-6:
            self._publish_stop()
            response.arrived = True
            response.lateral_error_mm = 0.0
            response.message = f'no-op dir={direction} 0mm (cold-call ok)'
            self.get_logger().info(f'[nav] {response.message}')
            return response

        # 4) odom 신선도 가드(폐루프 시) — 미수신이면 절대 무이동
        if self.use_odom_stop and not self._wait_for_odom(self.wait_for_odom_sec):
            self._publish_stop()
            response.arrived = False
            response.lateral_error_mm = 0.0
            response.message = f'no odom on {self.odom_topic} (refuse blind move)'
            self.get_logger().error(f'[nav] {response.message}')
            return response

        # 5) 이동 직렬화(방어) — 동시 호출 시 한 번에 하나만.
        if not self._move_lock.acquire(blocking=False):
            self._publish_stop()
            response.arrived = False
            response.lateral_error_mm = 0.0
            response.message = 'busy: another move in progress'
            self.get_logger().warning(f'[nav] {response.message}')
            return response

        start_pose = self._get_pose()
        t0 = time.monotonic()
        stop_reason = 'max_duration_reached'  # 비관적 기본값
        try:
            self.get_logger().info(
                f'[nav] MoveBaseLateral move ({direction} {distance_mm:.0f}mm) start')
            while rclpy.ok():
                elapsed = time.monotonic() - t0
                cur = self._get_pose()
                if self.use_odom_stop and cur is not None and start_pose is not None:
                    _, left = self._robot_frame_delta(start_pose, cur)
                    signed = sign * left
                    if signed >= distance_m:
                        stop_reason = 'odom_distance_reached'
                        break
                    if signed <= -self.wrong_direction_tolerance:
                        stop_reason = 'wrong_direction_detected'
                        break
                elif not self.use_odom_stop:
                    if elapsed >= distance_m / self.speed:
                        stop_reason = 'open_loop_duration_reached'
                        break
                if elapsed >= self.max_duration_sec:
                    stop_reason = 'max_duration_reached'
                    break

                cmd = Twist()
                cmd.linear.y = sign * self.speed
                self.cmd_pub.publish(cmd)
                time.sleep(self._period)  # NOT spin_once — odom 콜백은 다른 MTE 스레드

            # 제동: zero-Twist 를 몇 사이클 발행
            for _ in range(self.stop_brake_cycles):
                self._publish_stop()
                time.sleep(self._period)
        except Exception as exc:  # noqa: BLE001 — 어떤 경우든 베이스를 세운다
            stop_reason = f'exception: {exc}'
            self.get_logger().error(f'[nav] move 예외: {exc}')
        finally:
            self._publish_stop()  # 모든 종료 경로에서 베이스 정지
            self._move_lock.release()

        # 6) 결과 계산
        cur = self._get_pose()
        left_final = 0.0
        if start_pose is not None and cur is not None:
            _, left_final = self._robot_frame_delta(start_pose, cur)
        achieved = sign * left_final
        arrived = (stop_reason == 'odom_distance_reached') or (
            stop_reason == 'open_loop_duration_reached')
        # 부호 있는 잔차(도착 오차) mm: 달성 - 목표.
        err_mm = (achieved - distance_m) * 1000.0

        response.arrived = arrived
        response.lateral_error_mm = err_mm
        response.message = (
            f'{stop_reason} dir={direction} {distance_mm:.0f}mm '
            f'left_delta={left_final:+.3f}m err={err_mm:+.1f}mm')
        log = self.get_logger().info if arrived else self.get_logger().warning
        log(f'[nav] {response.message} arrived={arrived}')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveBaseLateralServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시에도 베이스 정지를 보장.
        try:
            node._publish_stop()
        except Exception:  # noqa: BLE001
            pass
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
