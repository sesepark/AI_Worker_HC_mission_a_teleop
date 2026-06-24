"""Mission C arm selector: y 좌표 기준으로 사용할 팔을 결정."""

from manipulation.robot_interface.moveit_client import Arm


def select_arm(y: float) -> Arm:
    """y >= 0 → LEFT, y < 0 → RIGHT."""
    return Arm.LEFT if y >= 0.0 else Arm.RIGHT
