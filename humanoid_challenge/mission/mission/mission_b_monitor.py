"""Mission B 조종자 모니터 (시각 대시보드).

`/mission_b/monitor`(std_msgs/String, JSON)를 구독해 PyQt5 창으로 신호를 시각 표시한다.
- Ⓑ-1 출발 신호(파지 완료 포함), Ⓑ-2 정지선 도착, Ⓑ-3 안착 완료를 색상 카드로 표시
- 박스 운반 개수(box_count / max_boxes), 현재 FSM 상태(state) 표시
- 일정 시간 메시지 미수신 시 '수신 대기' 로 흐리게 표시

mission_b.launch.py 에서 show_monitor:=true(기본) 면 자동 기동된다.
독립 실행: ros2 run mission mission_b_monitor   (DISPLAY 필요)

ROS 스핀은 백그라운드 데몬 스레드(SingleThreadedExecutor), Qt 이벤트 루프는 메인 스레드.
QTimer(100ms)는 노드가 보관한 최신 payload 만 읽어 라벨을 갱신한다(GIL 하 dict 읽기).
"""
import json
import signal
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

try:
    from PyQt5 import QtCore, QtGui, QtWidgets
except ImportError as exc:  # pragma: no cover
    print(f'[mission_b_monitor] PyQt5 가 필요합니다: {exc}', file=sys.stderr)
    raise


# 색상 팔레트
COL_BG = '#1e1e24'
COL_CARD_OFF = '#33333d'
COL_CARD_ON = '#2e8b57'      # 활성(녹색)
COL_CARD_BOX = '#3a6ea5'     # 박스 카운트(파랑)
COL_TEXT_DIM = '#888894'
COL_TEXT = '#f0f0f4'
STALE_SEC = 2.0              # 이 시간 이상 미수신 시 stale 표시


class MonitorNode(Node):
    """순수 ROS 구독자. 최신 payload 와 수신 시각만 보관."""

    def __init__(self):
        super().__init__('mission_b_monitor')
        self.latest = {}
        self.last_rx = None
        self.create_subscription(String, '/mission_b/monitor', self._cb, 10)
        self.get_logger().info('mission_b_monitor 시작 — /mission_b/monitor 구독')

    def _cb(self, msg: String) -> None:
        try:
            self.latest = json.loads(msg.data)
            self.last_rx = self.get_clock().now().nanoseconds * 1e-9
        except (ValueError, TypeError):
            pass


class Card(QtWidgets.QFrame):
    """제목(작게) + 값(크게) 한 장. set_state 로 색/텍스트 갱신."""

    def __init__(self, title: str, big: bool = False):
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(16, 12, 16, 12)
        self.title = QtWidgets.QLabel(title)
        self.title.setStyleSheet(f'color:{COL_TEXT_DIM}; font-size:15px;')
        self.value = QtWidgets.QLabel('—')
        vsize = 34 if big else 26
        self.value.setStyleSheet(
            f'color:{COL_TEXT}; font-size:{vsize}px; font-weight:bold;')
        self.value.setAlignment(QtCore.Qt.AlignCenter)
        lay.addWidget(self.title)
        lay.addWidget(self.value)
        self._paint(COL_CARD_OFF)

    def _paint(self, color: str) -> None:
        self.setStyleSheet(f'QFrame{{background:{color}; border-radius:12px;}}')

    def set_state(self, text: str, color: str) -> None:
        self.value.setText(text)
        self._paint(color)


class MonitorWindow(QtWidgets.QWidget):
    def __init__(self, node: MonitorNode):
        super().__init__()
        self.node = node
        self.setWindowTitle('Mission B Operator Monitor')
        self.resize(620, 540)
        self.setStyleSheet(f'background:{COL_BG};')

        root = QtWidgets.QVBoxLayout(self)
        root.setSpacing(12)
        root.setContentsMargins(18, 18, 18, 18)

        # 헤더줄: 제목(좌) + 경과시간(우).
        head_row = QtWidgets.QHBoxLayout()
        header = QtWidgets.QLabel('MISSION B  —  PARTS TRANSPORT')
        header.setStyleSheet(
            f'color:{COL_TEXT}; font-size:20px; font-weight:bold;')
        head_row.addWidget(header)
        head_row.addStretch(1)
        self.elapsed_lbl = QtWidgets.QLabel('00:00')
        self.elapsed_lbl.setStyleSheet(
            f'color:{COL_TEXT}; font-size:22px; font-weight:bold; '
            'font-family:monospace;')
        head_row.addWidget(self.elapsed_lbl)
        root.addLayout(head_row)

        self.state_lbl = QtWidgets.QLabel('Status: waiting')
        self.state_lbl.setStyleSheet(f'color:{COL_TEXT_DIM}; font-size:15px;')
        root.addWidget(self.state_lbl)

        # launch(노드 시작) 이후 경과 시간 측정 기준점.
        self.start_time = time.monotonic()

        self.c_depart = Card('B-1   Departure signal (grasp)')
        self.c_stop = Card('B-2   Stop line reached')
        self.c_deliver = Card('B-3   Placement complete')
        for c in (self.c_depart, self.c_stop, self.c_deliver):
            root.addWidget(c)

        self.c_box = Card('Boxes delivered', big=True)
        root.addWidget(self.c_box)
        root.addStretch(1)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(100)

    def _tick(self) -> None:
        # 경과 시간(launch 이후)은 데이터 수신 여부와 무관하게 항상 갱신.
        elapsed = int(time.monotonic() - self.start_time)
        h, rem = divmod(elapsed, 3600)
        m, s = divmod(rem, 60)
        self.elapsed_lbl.setText(
            f'{h:d}:{m:02d}:{s:02d}' if h else f'{m:02d}:{s:02d}')

        d = self.node.latest
        now = self.node.get_clock().now().nanoseconds * 1e-9
        stale = (self.node.last_rx is None) or (now - self.node.last_rx > STALE_SEC)

        if not d:
            self.state_lbl.setText('Status: waiting for data (mission_b not running?)')
            return

        # 화면 라벨은 영어로 고정한다(컨테이너에 한글 폰트 미설치 → tofu 방지).
        # payload 의 한글 *_text 필드 대신 불리언 플래그에서 영어 라벨을 직접 생성.
        state = d.get('state', '-')
        stage = d.get('stage', '-')
        tag = '  ·  SIGNAL LOST' if stale else ''
        self.state_lbl.setText(f'State: {state}   (stage={stage}){tag}')

        on = not stale
        self.c_depart.set_state(
            'DEPART OK (grasped)' if (on and d.get('departure_ready')) else 'waiting',
            COL_CARD_ON if (on and d.get('departure_ready')) else COL_CARD_OFF)
        self.c_stop.set_state(
            'STOP LINE REACHED' if (on and d.get('stopline_reached')) else 'waiting',
            COL_CARD_ON if (on and d.get('stopline_reached')) else COL_CARD_OFF)
        self.c_deliver.set_state(
            'PLACED OK' if (on and d.get('delivery_complete')) else 'waiting',
            COL_CARD_ON if (on and d.get('delivery_complete')) else COL_CARD_OFF)

        cnt = d.get('box_count', 0)
        mx = d.get('max_boxes', 0)
        self.c_box.set_state(f'{cnt} / {mx}', COL_CARD_BOX)


def _spin(executor: SingleThreadedExecutor) -> None:
    """백그라운드 스핀. SIGINT 로 컨텍스트가 내려가면 조용히 종료."""
    try:
        executor.spin()
    except (rclpy.executors.ExternalShutdownException, RuntimeError):
        pass


def main(args=None) -> None:
    # rclpy 신호 핸들러를 끈다(Qt 와 충돌 방지).
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=args)
    node = MonitorNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=_spin, args=(executor,), daemon=True)
    spin_thread.start()

    app = QtWidgets.QApplication(sys.argv)

    # SIGINT/SIGTERM 을 OS 기본 종료로 둔다. 반드시 QApplication 생성 *뒤*에 설정해야
    # 한다(앞에서 걸면 QApplication/rclpy 가 덮어써 Ctrl+C·launch 종료에 창이 매달림).
    # PyQt exec_() 는 Python 신호 핸들러를 제때 실행하지 않으므로 SIG_DFL 이 가장 확실.
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)

    win = MonitorWindow(node)
    win.show()
    try:
        app.exec_()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
