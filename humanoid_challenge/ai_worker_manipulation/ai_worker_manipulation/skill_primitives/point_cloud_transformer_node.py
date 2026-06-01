import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from ai_worker_manipulation.robot_interface.tf_transformer import tf_transformer

class PointCloudTransformerNode(Node):
    def __init__(self):
        super().__init__('point_cloud_transformer')  #inherites the node from parent NODE
        self.tf = tf_transformer(self)

        self.declare_parameter('left_topic',  '/camera_left/points') #/camer_left... this part needs to be confirmed with the perception team
        self.declare_parameter('right_topic', '/camera_right/points')
        self.declare_parameter('base_frame',  'base_link')

        left_topic  = self.get_parameter('left_topic').value
        right_topic = self.get_parameter('right_topic').value

        self.create_subscription(PointCloud2, left_topic,  self._on_left,  10)
        self.create_subscription(PointCloud2, right_topic, self._on_right, 10)

        self.pub_left  = self.create_publisher(PointCloud2, '/camera_left/points_base',  10)
        self.pub_right = self.create_publisher(PointCloud2, '/camera_right/points_base', 10)

    def _on_left(self, msg: PointCloud2):
        base_frame = self.get_parameter('base_frame').value
        transformed = self.tf.transform_cloud(msg, base_frame)
        if transformed is None:
            return
        self.pub_left.publish(transformed)
    
    def _on_right(self, msg: PointCloud2):
        base_frame = self.get_parameter('base_frame').value
        transformed = self.tf.transform_cloud(msg, base_frame)
        if transformed is None:
            return
        self.pub_right.publish(transformed)
    
def main():
    rclpy.init()
    node = PointCloudTransformerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

