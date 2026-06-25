"""Mission A grasp pose adapter: orientation + offset rules by object position."""

from geometry_msgs.msg import Pose

X_FAR_THRESHOLD  = 0.40    # m — 3rd row boundary
Y_WALL_THRESHOLD = -0.30   # m — right-wall side boundary
GRASP_Y_OFFSET   = 0   #0.043 0.045 applied as -Y (left side) or +Y (right-wall side)
GRASP_X_OFFSET   = 0 #-0.043 -0.045 applied in X for far row
GRASP_Z          = 0.83    # fixed grasp height (m)

_QUAT_STANDARD = (0.0, 0.0, 0.0,    1.0)     # standard top-down (tested on real robot)
_QUAT_YAW90    = (0.0, 0.0, 0.7071, 0.7071)  # 90° around Z — far row


def _copy_pose(pose: Pose) -> Pose:
    p = Pose()
    p.position.x = pose.position.x
    p.position.y = pose.position.y
    p.position.z = pose.position.z
    p.orientation.x = pose.orientation.x
    p.orientation.y = pose.orientation.y
    p.orientation.z = pose.orientation.z
    p.orientation.w = pose.orientation.w
    return p


def build_mission_a_grasp_pose(center_pose: Pose) -> Pose:
    """Return final grasp pose with orientation, offset, and z applied.

    x < X_FAR  AND y >= Y_WALL : standard quat, y -= GRASP_Y_OFFSET
    x < X_FAR  AND y <  Y_WALL : standard quat, y += GRASP_Y_OFFSET
    x >= X_FAR                 : yaw-90 quat,   x += GRASP_X_OFFSET
    """
    pose = _copy_pose(center_pose)
    pose.position.z = GRASP_Z

    if center_pose.position.x >= X_FAR_THRESHOLD:
        qx, qy, qz, qw = _QUAT_YAW90
        pose.position.x += GRASP_X_OFFSET
    else:
        qx, qy, qz, qw = _QUAT_STANDARD
        if center_pose.position.y >= Y_WALL_THRESHOLD:
            pose.position.y -= GRASP_Y_OFFSET
        else:
            pose.position.y += GRASP_Y_OFFSET

    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose
