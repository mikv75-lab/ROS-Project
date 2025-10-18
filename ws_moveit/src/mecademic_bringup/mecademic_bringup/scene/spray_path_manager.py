from typing import List, Tuple
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose
from rclpy.node import Node

def _pt(x,y,z):
    P = type("P", (), {})()
    P.x, P.y, P.z = x,y,z
    return P

class SprayPathManager:
    def __init__(self, node: Node, frame: str = "workspace_center"):
        self._node = node
        self._frame = frame
        self._pub = node.create_publisher(MarkerArray, "spray_path/markers", 10)
        self._axes_last = 0

    def clear(self):
        ma = MarkerArray()
        # line + points
        for mid in (3001, 3002):
            m = Marker()
            m.header.frame_id = self._frame
            m.header.stamp = self._node.get_clock().now().to_msg()
            m.ns = "spray_path"
            m.id = mid
            m.action = Marker.DELETE
            ma.markers.append(m)
        # axes
        for i in range(self._axes_last):
            for axis in range(3):
                m = Marker()
                m.header.frame_id = self._frame
                m.header.stamp = self._node.get_clock().now().to_msg()
                m.ns = "spray_axes"
                m.id = 400000 + i*3 + axis
                m.action = Marker.DELETE
                ma.markers.append(m)
        self._pub.publish(ma)
        self._axes_last = 0

    def draw(self, poses: List[Pose], *, line_w=0.003, point_d=0.006, axes=True, axis_len=0.03, axis_th=0.003):
        self.clear()
        if not poses:
            return
        now = self._node.get_clock().now().to_msg()
        ma = MarkerArray()
        # line
        line = Marker()
        line.header.frame_id = self._frame; line.header.stamp = now
        line.ns = "spray_path"; line.id = 3001
        line.type = Marker.LINE_STRIP; line.action = Marker.ADD
        line.scale.x = line_w
        line.points = [_pt(p.position.x, p.position.y, p.position.z) for p in poses]
        ma.markers.append(line)
        # points
        pts = Marker()
        pts.header.frame_id = self._frame; pts.header.stamp = now
        pts.ns = "spray_path"; pts.id = 3002
        pts.type = Marker.SPHERE_LIST; pts.action = Marker.ADD
        pts.scale.x = pts.scale.y = pts.scale.z = point_d
        pts.points = [_pt(p.position.x, p.position.y, p.position.z) for p in poses]
        ma.markers.append(pts)
        # axes
        if axes and axis_len > 0.0 and axis_th > 0.0:
            for i, p in enumerate(poses):
                # Z-Achse anzeigen (einfach)
                tip = _pt(p.position.x, p.position.y, p.position.z + axis_len)
                arr = Marker()
                arr.header.frame_id = self._frame; arr.header.stamp = now
                arr.ns = "spray_axes"; arr.id = 400000 + i*3 + 2
                arr.type = Marker.ARROW; arr.action = Marker.ADD
                arr.points = [_pt(p.position.x, p.position.y, p.position.z), tip]
                arr.scale.x = axis_th; arr.scale.y = axis_th*2.0; arr.scale.z = axis_th*2.5
                ma.markers.append(arr)
            self._axes_last = len(poses)
        self._pub.publish(ma)
