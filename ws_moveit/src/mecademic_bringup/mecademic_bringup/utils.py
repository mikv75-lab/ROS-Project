# mecademic_bringup/utils.py
import os, math
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from rclpy.qos import QoSProfile
import time
import rclpy
from rclpy.node import Node
from rclpy.topic_endpoint_info import TopicEndpointTypeEnum
import time
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from launch import Action
from tf_transformations import quaternion_from_euler

_PKG = 'mecademic_bringup'
_PKG_SHARE = get_package_share_directory(_PKG)

def resolve_mesh_path(path: str) -> str:
    if not path:
        return ""
    return path if os.path.isabs(path) else os.path.join(_PKG_SHARE, path)

def rpy_deg_to_quat(r, p, y):
    # ROS-konform: Yaw-Pitch-Roll (Z-Y-X)
    q = quaternion_from_euler(
        math.radians(r),  # Roll
        math.radians(p),  # Pitch
        math.radians(y),  # Yaw
        axes="sxyz"       # Standard: Fixed-frame XYZ (ROS erwartet so)
    )
    return q  # (x, y, z, w)