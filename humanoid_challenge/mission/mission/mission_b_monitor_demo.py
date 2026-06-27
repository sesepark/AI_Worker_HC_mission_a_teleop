"""Mission B 모니터 검증용 데모 퍼블리셔 (실로봇 불필요).

`/mission_b/monitor`(std_msgs/String, JSON)에 실제 mission_b FSM 과 동일한 스키마로
신호를 5초 간격으로 순서대로 진행시켜, 모니터 창(mission_b_monitor)의 동작만 확인한다.

진행 시퀀스(박스 1개당):
  대기(INIT) → B-1 출발신호 → B-2 정지선 → B-3 안착(box_count++) → 다음 박스…
  → 마지막 박스 후 DONE_B 에서 정지(전부 점등 유지).

각 단계는 step_sec(기본 5.0)초 머문다. payload 는 republish_sec(기본 0.5)초마다 재발행해
모니터의 staleness(2초) 판정을 피한다.

실행:
  ros2 run mission mission_b_monitor          # 터미널 1: 모니터 창
  ros2 run mission mission_b_monitor_demo     # 터미널 2: 데모 신호
  # 파라미터: max_boxes(기본 2), step_sec(기본 5.0), loop(끝나면 처음부터 반복)
  ros2 run mission mission_b_monitor_demo --ros-args -p max_boxes:=4 -p loop:=true
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _build_phases(max_boxes: int):
    """(state, departure, stopline, delivery, box_count) 단계 리스트 생성."""
    phases = [('INIT', False, False, False, 0)]
    for b in range(1, max_boxes + 1):
        # 새 박스 시작 시 정지선/안착 플래그는 리셋(출발은 유지되다 갱신).
        phases.append((f'B1_DEPART#{b}', True, False, False, b - 1))
        phases.append((f'B2_STOPLINE#{b}', True, True, False, b - 1))
        phases.append((f'B3_COMPLETE#{b}', True, True, True, b))
    phases.append(('DONE_B', True, True, True, max_boxes))
    return phases


class MonitorDemo(Node):
    def __init__(self):
        super().__init__('mission_b_monitor_demo')
        self.declare_parameter('max_boxes', 2)
        self.declare_parameter('step_sec', 5.0)
        self.declare_parameter('republish_sec', 0.5)
        self.declare_parameter('loop', False)

        self.max_boxes = int(self.get_parameter('max_boxes').value)
        self.step_sec = float(self.get_parameter('step_sec').value)
        self.loop = bool(self.get_parameter('loop').value)
        republish = float(self.get_parameter('republish_sec').value)

        self.phases = _build_phases(self.max_boxes)
        self.idx = 0
        self.t0 = self._now()

        self.pub = self.create_publisher(String, '/mission_b/monitor', 10)
        self.create_timer(republish, self._republish)        # 재발행(staleness 회피)
        self.create_timer(self.step_sec, self._advance)       # 5초마다 단계 진행
        self._republish()
        self.get_logger().info(
            f'monitor demo 시작 — max_boxes={self.max_boxes}, step={self.step_sec}s, '
            f'단계 {len(self.phases)}개, loop={self.loop}')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _advance(self) -> None:
        if self.idx < len(self.phases) - 1:
            self.idx += 1
        elif self.loop:
            self.idx = 0
        else:
            return  # 마지막(DONE_B) 유지
        state = self.phases[self.idx][0]
        self.get_logger().info(f'[{self.idx}/{len(self.phases) - 1}] -> {state}')
        self._republish()

    def _republish(self) -> None:
        state, dep, stop, deliver, cnt = self.phases[self.idx]
        payload = {
            'state': state,
            'mode': 'demo',
            'stage': 'all',
            'box_count': cnt,
            'max_boxes': self.max_boxes,
            'departure_ready': dep,
            'delivery_complete': deliver,
            'stopline_reached': stop,
            # mission_b 와 동일 스키마(모니터는 플래그로 영문 표기, 텍스트는 참고용).
            'departure_text': '출발 가능' if dep else '',
            'delivery_text': '안착 완료' if deliver else '',
            'stopline_text': '정지선 도착' if stop else '',
            'attempts': 0,
            'ts': round(self._now(), 3),
        }
        self.pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MonitorDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
