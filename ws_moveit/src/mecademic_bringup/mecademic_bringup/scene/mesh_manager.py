# mecademic_bringup/scene/mesh_manager.py
from typing import Optional
from visualization_msgs.msg import Marker
from rclpy.node import Node
from geometry_msgs.msg import Pose
from mecademic_bringup.common.topics import TOPIC_VISUALIZATION_MARKER

class MeshManager:
    """
    Einfacher Visualizer für ein einzelnes Mesh je Objekt-ID über RViz Marker.
    """
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(Marker, TOPIC_VISUALIZATION_MARKER, 10)

    def publish_mesh(self, *, obj_id: str, frame_id: str, mesh_resource: str,
                     pose: Pose, scale=(1.0, 1.0, 1.0), color=(0.7, 0.7, 0.7, 1.0)):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.ns = "scene_meshes"
        m.id = abs(hash(obj_id)) % (2**31-1)
        m.action = Marker.ADD
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = f"file://{mesh_resource}"
        m.pose = pose
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = color
        self._pub.publish(m)

    def delete_mesh(self, obj_id: str, frame_id: str):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.ns = "scene_meshes"
        m.id = abs(hash(obj_id)) % (2**31-1)
        m.action = Marker.DELETE
        self._pub.publish(m)
