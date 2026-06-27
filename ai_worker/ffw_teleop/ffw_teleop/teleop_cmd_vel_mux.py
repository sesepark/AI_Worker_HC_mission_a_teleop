import json

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import String


def make_teleop_qos(depth=1):
    qos = QoSProfile(depth=depth)
    qos.reliability = ReliabilityPolicy.BEST_EFFORT
    qos.durability = DurabilityPolicy.VOLATILE
    return qos


class TeleopCmdVelMux(Node):

    def __init__(self):
        super().__init__('teleop_cmd_vel_mux')

        self.declare_parameter('joystick_cmd_vel_topic', '/teleop/joystick_cmd_vel')
        self.declare_parameter('keyboard_cmd_vel_topic', '/teleop/keyboard_cmd_vel')
        self.declare_parameter('keyboard_enabled_topic', '/teleop/keyboard_drive/enabled')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('status_topic', '/teleop/cmd_vel_mux/status')
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('keyboard_stale_timeout_s', 0.20)
        self.declare_parameter('joystick_stale_timeout_s', 0.30)

        self.joystick_cmd_vel_topic = str(
            self.get_parameter('joystick_cmd_vel_topic').value).strip()
        self.keyboard_cmd_vel_topic = str(
            self.get_parameter('keyboard_cmd_vel_topic').value).strip()
        self.keyboard_enabled_topic = str(
            self.get_parameter('keyboard_enabled_topic').value).strip()
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value).strip()
        self.status_topic = str(self.get_parameter('status_topic').value).strip()
        publish_hz = max(float(self.get_parameter('publish_hz').value), 1.0)
        self.keyboard_stale_timeout_s = max(
            float(self.get_parameter('keyboard_stale_timeout_s').value), 0.05)
        self.joystick_stale_timeout_s = max(
            float(self.get_parameter('joystick_stale_timeout_s').value), 0.05)

        self.keyboard_enabled = False
        self.last_joystick_cmd = Twist()
        self.last_keyboard_cmd = Twist()
        self.last_joystick_time_s = 0.0
        self.last_keyboard_time_s = 0.0
        self.last_status_publish_s = 0.0
        self.last_source = 'startup'

        cmd_input_qos = make_teleop_qos(depth=1)
        cmd_output_qos = QoSProfile(depth=1)
        cmd_output_qos.reliability = ReliabilityPolicy.RELIABLE
        cmd_output_qos.durability = DurabilityPolicy.VOLATILE
        enabled_qos = QoSProfile(depth=1)
        enabled_qos.reliability = ReliabilityPolicy.RELIABLE
        enabled_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, cmd_output_qos)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(
            Twist, self.joystick_cmd_vel_topic, self._joystick_callback, cmd_input_qos)
        self.create_subscription(
            Twist, self.keyboard_cmd_vel_topic, self._keyboard_callback, cmd_input_qos)
        self.create_subscription(
            Bool, self.keyboard_enabled_topic, self._keyboard_enabled_callback, enabled_qos)
        self.timer = self.create_timer(1.0 / publish_hz, self._publish)

        self.get_logger().info(
            f'cmd_vel mux active: joystick={self.joystick_cmd_vel_topic}, '
            f'keyboard={self.keyboard_cmd_vel_topic}, enabled={self.keyboard_enabled_topic}, '
            f'out={self.cmd_vel_topic}')

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _joystick_callback(self, msg):
        self.last_joystick_cmd = msg
        self.last_joystick_time_s = self._now_s()

    def _keyboard_callback(self, msg):
        self.last_keyboard_cmd = msg
        self.last_keyboard_time_s = self._now_s()

    def _keyboard_enabled_callback(self, msg):
        self.keyboard_enabled = bool(msg.data)

    def _fresh(self, stamp_s, timeout_s, now_s):
        return stamp_s > 0.0 and (now_s - stamp_s) <= timeout_s

    def _publish(self):
        now_s = self._now_s()
        keyboard_fresh = self._fresh(
            self.last_keyboard_time_s, self.keyboard_stale_timeout_s, now_s)
        joystick_fresh = self._fresh(
            self.last_joystick_time_s, self.joystick_stale_timeout_s, now_s)

        if self.keyboard_enabled:
            if keyboard_fresh:
                output = self.last_keyboard_cmd
                source = 'keyboard'
            else:
                output = Twist()
                source = 'keyboard_stale'
        elif joystick_fresh:
            output = self.last_joystick_cmd
            source = 'joystick'
        else:
            output = Twist()
            source = 'joystick_stale'

        self.cmd_pub.publish(output)
        self.last_source = source
        if now_s - self.last_status_publish_s >= 0.2:
            self.last_status_publish_s = now_s
            self._publish_status(now_s, source, keyboard_fresh, joystick_fresh)

    def _owner_for_source(self, source):
        if source == 'keyboard':
            return 'monitor'
        if source == 'keyboard_stale':
            return 'monitor_stale'
        if source == 'joystick':
            return 'leader'
        if source == 'joystick_stale':
            return 'leader_stale'
        return source

    def _publish_status(self, now_s, source, keyboard_fresh, joystick_fresh):
        msg = String()
        msg.data = json.dumps({
            'stamp_sec': now_s,
            'source': source,
            'owner': self._owner_for_source(source),
            'drive_enabled': bool(self.keyboard_enabled),
            'keyboard_enabled': bool(self.keyboard_enabled),
            'keyboard_fresh': bool(keyboard_fresh),
            'joystick_fresh': bool(joystick_fresh),
            'output_topic': self.cmd_vel_topic,
            'keyboard_age_s': (
                None if self.last_keyboard_time_s <= 0.0
                else max(now_s - self.last_keyboard_time_s, 0.0)
            ),
            'joystick_age_s': (
                None if self.last_joystick_time_s <= 0.0
                else max(now_s - self.last_joystick_time_s, 0.0)
            ),
        }, sort_keys=True)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopCmdVelMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
