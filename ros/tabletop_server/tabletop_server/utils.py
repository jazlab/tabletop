import math

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from rclpy.node import Node


class ServiceCallTimeoutError(Exception):
    """Custom exception for service call timeout."""

    pass


class ServiceWaitTimeoutError(Exception):
    """Custom exception for wait_for_service timeout."""

    pass


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert roll, pitch, yaw angles (in radians) to a quaternion.
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy

    return q


def pose_stamped_from_params(node: Node, prefix: str):
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = node.get_parameter(
        f"{prefix}.header.frame_id"
    ).value
    pose = Pose()
    pose.position.x = node.get_parameter(f"{prefix}.pose.position.x").value
    pose.position.y = node.get_parameter(f"{prefix}.pose.position.y").value
    pose.position.z = node.get_parameter(f"{prefix}.pose.position.z").value
    pose.orientation = quaternion_from_euler(
        node.get_parameter(f"{prefix}.pose.orientation.roll").value,
        node.get_parameter(f"{prefix}.pose.orientation.pitch").value,
        node.get_parameter(f"{prefix}.pose.orientation.yaw").value,
    )
    pose_stamped.pose = pose
    return pose_stamped
