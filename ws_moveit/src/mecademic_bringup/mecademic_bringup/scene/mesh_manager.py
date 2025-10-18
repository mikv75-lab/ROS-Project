from typing import Optional, Tuple, List
from visualization_msgs.msg import Marker
from rclpy.node import Node
from .utils import rpy_deg_to_quat

class MeshManager:
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Marker, "visualization_marker", 10)

    def _publish_marker(self, *, ns: str, mid: int, parent: str,
                        mesh_abs: str, xyz: List[float], rpy_deg=None, quat=None,
                        rgba=(0.8,0.8,0.8,1.0)):
        m = Marker()
        m.header.frame_id = parent
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD if mesh_abs else Marker.DELETE
        if mesh_abs:
            m.mesh_resource = f"file://{mesh_abs}"
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
            if quat is None:
                qx,qy,qz,qw = rpy_deg_to_quat(*(rpy_deg or [0,0,0]))
            else:
                qx,qy,qz,qw = quat
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (qx,qy,qz,qw)
        self._pub.publish(m)

    def show_or_update(self, *, obj_id: str, parent: str, mesh_abs: str,
                       xyz=(0,0,0), rpy_deg=(0,0,0), rgba=(0.8,0.8,0.8,1.0)):
        self._publish_marker(ns=obj_id, mid=1000, parent=parent, mesh_abs=mesh_abs, xyz=list(xyz), rpy_deg=list(rpy_deg), rgba=rgba)

    def clear(self, *, obj_id: str, parent: str):
        self._publish_marker(ns=obj_id, mid=1000, parent=parent, mesh_abs="", xyz=[0,0,0])
