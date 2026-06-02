import rclpy
import tf2_ros
import tf2_sensor_msgs
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import PointCloud2

class tf_transformer:
    def __init__(self, node):
        self.node = node
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self.node)

    def transform_cloud(self, cloud_msg: PointCloud2, target_frame: str) -> PointCloud2 | None:
        try:
            return self.buffer.transform(cloud_msg, target_frame)    
        except tf2_ros.TransformException as e:
            self.node.get_logger().warn(f'tranform_cloud failed: {e}')
            return None

        
    def transform_pose(self, pose_msg: PoseStamped, target_frame: str) -> Pose | None:
        try:
            transformed = self.buffer.transform(pose_msg, target_frame)
            return transformed.pose
        except tf2_ros.TransformException as e:
            self.node.get_logger().warn(f'transform_pose failed: {e}')
            return None

    def is_transform_available(self, from_frame: str, to_frame: str) -> bool:
        return self.buffer.can_transform(to_frame, from_frame, rclpy.time.Time())

"""
additional functions for future purposes:
- get_transform(from_frame, to_frame) : returns raw transform matrix
- wait_for_transform(from_frame, to_frame, timeout_sec)
- just a single line transform_poses (pose_array, target_frame)
"""