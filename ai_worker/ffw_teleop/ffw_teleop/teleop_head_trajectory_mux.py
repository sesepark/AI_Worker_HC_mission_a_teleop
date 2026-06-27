import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import Bool
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


class TeleopHeadTrajectoryMux(Node):

    def __init__(self):
        super().__init__('teleop_head_trajectory_mux')

        self.declare_parameter('leader_head_topic', '/teleop/leader_head_cmd')
        self.declare_parameter('monitor_head_topic', '/teleop/monitor_head_cmd')
        self.declare_parameter('monitor_enabled_topic', '/teleop/head_drive/enabled')
        self.declare_parameter(
            'output_head_topic',
            '/leader/joystick_controller_left/joint_trajectory',
        )
        self.declare_parameter('status_topic', '/teleop/head_mux/status')
        self.declare_parameter('publish_hz', 30.0)
        self.declare_parameter('leader_stale_timeout_s', 0.50)
        self.declare_parameter('monitor_stale_timeout_s', 2.00)

        self.leader_head_topic = str(self.get_parameter('leader_head_topic').value).strip()
        self.monitor_head_topic = str(self.get_parameter('monitor_head_topic').value).strip()
        self.monitor_enabled_topic = str(
            self.get_parameter('monitor_enabled_topic').value).strip()
        self.output_head_topic = str(self.get_parameter('output_head_topic').value).strip()
        self.status_topic = str(self.get_parameter('status_topic').value).strip()
        publish_hz = max(float(self.get_parameter('publish_hz').value), 1.0)
        self.leader_stale_timeout_s = max(
            float(self.get_parameter('leader_stale_timeout_s').value), 0.05)
        self.monitor_stale_timeout_s = max(
            float(self.get_parameter('monitor_stale_timeout_s').value), 0.05)

        self.monitor_enabled = False
        self.last_leader_msg = None
        self.last_monitor_msg = None
        self.last_leader_time_s = 0.0
        self.last_monitor_time_s = 0.0
        self.last_status_publish_s = 0.0

        enabled_qos = QoSProfile(depth=1)
        enabled_qos.reliability = ReliabilityPolicy.RELIABLE
        enabled_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.output_pub = self.create_publisher(JointTrajectory, self.output_head_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(
            JointTrajectory, self.leader_head_topic, self._leader_callback, 10)
        self.create_subscription(
            JointTrajectory, self.monitor_head_topic, self._monitor_callback, 10)
        self.create_subscription(
            Bool, self.monitor_enabled_topic, self._enabled_callback, enabled_qos)
        self.timer = self.create_timer(1.0 / publish_hz, self._publish)

        self.get_logger().info(
            f'head trajectory mux active: leader={self.leader_head_topic}, '
            f'monitor={self.monitor_head_topic}, enabled={self.monitor_enabled_topic}, '
            f'out={self.output_head_topic}')

    def _now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _leader_callback(self, msg):
        self.last_leader_msg = msg
        self.last_leader_time_s = self._now_s()

    def _monitor_callback(self, msg):
        self.last_monitor_msg = msg
        self.last_monitor_time_s = self._now_s()

    def _enabled_callback(self, msg):
        self.monitor_enabled = bool(msg.data)

    def _fresh(self, stamp_s, timeout_s, now_s):
        return stamp_s > 0.0 and (now_s - stamp_s) <= timeout_s

    def _publish(self):
        now_s = self._now_s()
        leader_fresh = self._fresh(
            self.last_leader_time_s, self.leader_stale_timeout_s, now_s)
        monitor_fresh = self._fresh(
            self.last_monitor_time_s, self.monitor_stale_timeout_s, now_s)

        output = None
        if self.monitor_enabled:
            if self.last_monitor_msg is not None:
                output = self.last_monitor_msg
                source = 'monitor' if monitor_fresh else 'monitor_hold'
            elif leader_fresh:
                output = self.last_leader_msg
                source = 'monitor_waiting_leader'
            else:
                source = 'monitor_waiting'
        elif leader_fresh:
            output = self.last_leader_msg
            source = 'leader'
        elif self.last_leader_msg is not None:
            output = self.last_leader_msg
            source = 'leader_hold'
        else:
            source = 'startup'

        if output is not None:
            self.output_pub.publish(output)

        if now_s - self.last_status_publish_s >= 0.2:
            self.last_status_publish_s = now_s
            self._publish_status(now_s, source, leader_fresh, monitor_fresh)

    def _owner_for_source(self, source):
        if source in ('monitor', 'monitor_hold'):
            return 'monitor'
        if source in ('monitor_waiting', 'monitor_waiting_leader'):
            return 'monitor_waiting'
        if source in ('leader', 'leader_hold'):
            return 'leader'
        return source

    def _publish_status(self, now_s, source, leader_fresh, monitor_fresh):
        msg = String()
        msg.data = json.dumps({
            'stamp_sec': now_s,
            'source': source,
            'owner': self._owner_for_source(source),
            'head_enabled': bool(self.monitor_enabled),
            'monitor_enabled': bool(self.monitor_enabled),
            'leader_fresh': bool(leader_fresh),
            'monitor_fresh': bool(monitor_fresh),
            'output_topic': self.output_head_topic,
            'leader_age_s': (
                None if self.last_leader_time_s <= 0.0
                else max(now_s - self.last_leader_time_s, 0.0)
            ),
            'monitor_age_s': (
                None if self.last_monitor_time_s <= 0.0
                else max(now_s - self.last_monitor_time_s, 0.0)
            ),
        }, sort_keys=True)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopHeadTrajectoryMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
