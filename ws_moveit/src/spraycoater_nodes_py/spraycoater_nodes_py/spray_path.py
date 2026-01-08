# -*- coding: utf-8 -*-
# File: spraycoater_nodes_py/spray_path.py
#!/usr/bin/env python3
"""
spraycoater_nodes_py/spray_path.py

SprayPath Cache Node (STRICT, 2026-01):
- Implements ONLY these config_hub topic IDs:

  subscribe:
    clear
    show_compiled
    show_planned
    show_executed
    compiled_poses_in
    compiled_markers_in
    planned_poses_in
    planned_markers_in
    executed_poses_in
    executed_markers_in

  publish:
    compiled_poses
    compiled_markers
    planned_poses
    planned_markers
    executed_poses
    executed_markers

Behavior:
- caches incoming poses/markers per layer
- show_* toggles:
    - ON  -> publish cached if available
    - OFF -> publish empty PoseArray + Marker.DELETEALL
- clear:
    - reset caches + clear all outputs
- 1 Hz republish safety net for RViz/late subscribers

No legacy traj IDs, no additional topics.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray, Marker

from spraycoater_nodes_py.utils.config_hub import topics


@dataclass
class _LayerCache:
    poses: Optional[PoseArray] = None
    markers: Optional[MarkerArray] = None


class SprayPath(Node):
    GROUP = "spray_path"

    L_COMPILED = "compiled"
    L_PLANNED = "planned"
    L_EXECUTED = "executed"

    def __init__(self) -> None:
        super().__init__("spray_path")

        self.declare_parameter("backend", "default")
        self.backend: str = self.get_parameter("backend").get_parameter_value().string_value or "default"

        self.loader = topics()

        # ---------------- SUB (toggles + clear) ----------------
        t_clear = self.loader.subscribe_topic(self.GROUP, "clear")
        q_clear = self.loader.qos_by_id("subscribe", self.GROUP, "clear")

        t_show_compiled = self.loader.subscribe_topic(self.GROUP, "show_compiled")
        q_show_compiled = self.loader.qos_by_id("subscribe", self.GROUP, "show_compiled")

        t_show_planned = self.loader.subscribe_topic(self.GROUP, "show_planned")
        q_show_planned = self.loader.qos_by_id("subscribe", self.GROUP, "show_planned")

        t_show_executed = self.loader.subscribe_topic(self.GROUP, "show_executed")
        q_show_executed = self.loader.qos_by_id("subscribe", self.GROUP, "show_executed")

        # ---------------- SUB (inputs) ----------------
        t_compiled_poses_in = self.loader.subscribe_topic(self.GROUP, "compiled_poses_in")
        q_compiled_poses_in = self.loader.qos_by_id("subscribe", self.GROUP, "compiled_poses_in")

        t_compiled_markers_in = self.loader.subscribe_topic(self.GROUP, "compiled_markers_in")
        q_compiled_markers_in = self.loader.qos_by_id("subscribe", self.GROUP, "compiled_markers_in")

        t_planned_poses_in = self.loader.subscribe_topic(self.GROUP, "planned_poses_in")
        q_planned_poses_in = self.loader.qos_by_id("subscribe", self.GROUP, "planned_poses_in")

        t_planned_markers_in = self.loader.subscribe_topic(self.GROUP, "planned_markers_in")
        q_planned_markers_in = self.loader.qos_by_id("subscribe", self.GROUP, "planned_markers_in")

        t_executed_poses_in = self.loader.subscribe_topic(self.GROUP, "executed_poses_in")
        q_executed_poses_in = self.loader.qos_by_id("subscribe", self.GROUP, "executed_poses_in")

        t_executed_markers_in = self.loader.subscribe_topic(self.GROUP, "executed_markers_in")
        q_executed_markers_in = self.loader.qos_by_id("subscribe", self.GROUP, "executed_markers_in")

        # ---------------- PUB (outputs) ----------------
        t_compiled_poses = self.loader.publish_topic(self.GROUP, "compiled_poses")
        q_compiled_poses = self.loader.qos_by_id("publish", self.GROUP, "compiled_poses")

        t_compiled_markers = self.loader.publish_topic(self.GROUP, "compiled_markers")
        q_compiled_markers = self.loader.qos_by_id("publish", self.GROUP, "compiled_markers")

        t_planned_poses = self.loader.publish_topic(self.GROUP, "planned_poses")
        q_planned_poses = self.loader.qos_by_id("publish", self.GROUP, "planned_poses")

        t_planned_markers = self.loader.publish_topic(self.GROUP, "planned_markers")
        q_planned_markers = self.loader.qos_by_id("publish", self.GROUP, "planned_markers")

        t_executed_poses = self.loader.publish_topic(self.GROUP, "executed_poses")
        q_executed_poses = self.loader.qos_by_id("publish", self.GROUP, "executed_poses")

        t_executed_markers = self.loader.publish_topic(self.GROUP, "executed_markers")
        q_executed_markers = self.loader.qos_by_id("publish", self.GROUP, "executed_markers")

        # ---------------- ROS interfaces ----------------
        self.pub_compiled_poses = self.create_publisher(PoseArray, t_compiled_poses, q_compiled_poses)
        self.pub_compiled_markers = self.create_publisher(MarkerArray, t_compiled_markers, q_compiled_markers)

        self.pub_planned_poses = self.create_publisher(PoseArray, t_planned_poses, q_planned_poses)
        self.pub_planned_markers = self.create_publisher(MarkerArray, t_planned_markers, q_planned_markers)

        self.pub_executed_poses = self.create_publisher(PoseArray, t_executed_poses, q_executed_poses)
        self.pub_executed_markers = self.create_publisher(MarkerArray, t_executed_markers, q_executed_markers)

        self.sub_clear = self.create_subscription(Empty, t_clear, self._on_clear, q_clear)

        self.sub_show_compiled = self.create_subscription(Bool, t_show_compiled, self._on_show_compiled, q_show_compiled)
        self.sub_show_planned = self.create_subscription(Bool, t_show_planned, self._on_show_planned, q_show_planned)
        self.sub_show_executed = self.create_subscription(Bool, t_show_executed, self._on_show_executed, q_show_executed)

        self.sub_compiled_poses = self.create_subscription(
            PoseArray, t_compiled_poses_in, self._on_compiled_poses_in, q_compiled_poses_in
        )
        self.sub_compiled_markers = self.create_subscription(
            MarkerArray, t_compiled_markers_in, self._on_compiled_markers_in, q_compiled_markers_in
        )

        self.sub_planned_poses = self.create_subscription(
            PoseArray, t_planned_poses_in, self._on_planned_poses_in, q_planned_poses_in
        )
        self.sub_planned_markers = self.create_subscription(
            MarkerArray, t_planned_markers_in, self._on_planned_markers_in, q_planned_markers_in
        )

        self.sub_executed_poses = self.create_subscription(
            PoseArray, t_executed_poses_in, self._on_executed_poses_in, q_executed_poses_in
        )
        self.sub_executed_markers = self.create_subscription(
            MarkerArray, t_executed_markers_in, self._on_executed_markers_in, q_executed_markers_in
        )

        # ---------------- State ----------------
        self._cache: Dict[str, _LayerCache] = {
            self.L_COMPILED: _LayerCache(),
            self.L_PLANNED: _LayerCache(),
            self.L_EXECUTED: _LayerCache(),
        }

        # default visible
        self._show: Dict[str, bool] = {
            self.L_COMPILED: True,
            self.L_PLANNED: True,
            self.L_EXECUTED: True,
        }

        # safety net for RViz/late subscribers
        self._timer = self.create_timer(1.0, self._republish_all)

        self.get_logger().info(
            f"✅ SprayPath ready (backend='{self.backend}'): compiled/planned/executed | strict topics.yaml IDs"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _mk_empty_pose_array(frame_id: str = "") -> PoseArray:
        msg = PoseArray()
        msg.header.frame_id = frame_id or ""
        return msg

    @staticmethod
    def _mk_deleteall_marker_array(frame_id: str = "") -> MarkerArray:
        ma = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        m.header.frame_id = frame_id or ""
        ma.markers.append(m)
        return ma

    def _publish_layer_poses(self, key: str, msg: PoseArray) -> None:
        if key == self.L_COMPILED:
            self.pub_compiled_poses.publish(msg)
        elif key == self.L_PLANNED:
            self.pub_planned_poses.publish(msg)
        elif key == self.L_EXECUTED:
            self.pub_executed_poses.publish(msg)

    def _publish_layer_markers(self, key: str, msg: MarkerArray) -> None:
        if key == self.L_COMPILED:
            self.pub_compiled_markers.publish(msg)
        elif key == self.L_PLANNED:
            self.pub_planned_markers.publish(msg)
        elif key == self.L_EXECUTED:
            self.pub_executed_markers.publish(msg)

    def _publish_layer_clear(self, key: str, *, frame_id: str = "") -> None:
        self._publish_layer_poses(key, self._mk_empty_pose_array(frame_id))
        self._publish_layer_markers(key, self._mk_deleteall_marker_array(frame_id))

    def _publish_layer_cached_if_shown(self, key: str) -> None:
        if not self._show.get(key, True):
            return
        c = self._cache.get(key)
        if c is None:
            return
        if c.poses is not None:
            self._publish_layer_poses(key, c.poses)
        if c.markers is not None:
            self._publish_layer_markers(key, c.markers)

    def _clear_cache_layer(self, key: str) -> None:
        c = self._cache.get(key)
        if c is not None:
            c.poses = None
            c.markers = None

    # ------------------------------------------------------------------
    # Clear
    # ------------------------------------------------------------------

    def _on_clear(self, _msg: Empty) -> None:
        # reset caches
        self._clear_cache_layer(self.L_COMPILED)
        self._clear_cache_layer(self.L_PLANNED)
        self._clear_cache_layer(self.L_EXECUTED)

        # clear outputs
        self._publish_layer_clear(self.L_COMPILED, frame_id="")
        self._publish_layer_clear(self.L_PLANNED, frame_id="")
        self._publish_layer_clear(self.L_EXECUTED, frame_id="")

        self.get_logger().info(f"[{self.backend}] clear -> reset caches + outputs")

    # ------------------------------------------------------------------
    # Toggles
    # ------------------------------------------------------------------

    def _set_show(self, key: str, v: bool) -> None:
        v = bool(v)
        prev = bool(self._show.get(key, True))
        self._show[key] = v

        if v and not prev:
            # became visible -> publish cache
            self._publish_layer_cached_if_shown(key)
            return

        if (not v) and prev:
            # became hidden -> clear outputs in that layer
            frame_id = ""
            c = self._cache.get(key)
            if c and c.poses is not None:
                frame_id = (c.poses.header.frame_id or "").strip()
            self._publish_layer_clear(key, frame_id=frame_id)
            return

    def _on_show_compiled(self, msg: Bool) -> None:
        self._set_show(self.L_COMPILED, bool(getattr(msg, "data", False)))

    def _on_show_planned(self, msg: Bool) -> None:
        self._set_show(self.L_PLANNED, bool(getattr(msg, "data", False)))

    def _on_show_executed(self, msg: Bool) -> None:
        self._set_show(self.L_EXECUTED, bool(getattr(msg, "data", False)))

    # ------------------------------------------------------------------
    # Inputs (GUI -> Node)
    # ------------------------------------------------------------------

    def _on_compiled_poses_in(self, msg: PoseArray) -> None:
        if msg is None or len(msg.poses) == 0:
            frame_id = (msg.header.frame_id if msg else "") or ""
            self._cache[self.L_COMPILED].poses = None
            if self._show[self.L_COMPILED]:
                self.pub_compiled_poses.publish(self._mk_empty_pose_array(frame_id))
            return

        self._cache[self.L_COMPILED].poses = msg
        if self._show[self.L_COMPILED]:
            self.pub_compiled_poses.publish(msg)

    def _on_compiled_markers_in(self, msg: MarkerArray) -> None:
        if msg is None or len(msg.markers) == 0:
            self._cache[self.L_COMPILED].markers = None
            if self._show[self.L_COMPILED]:
                self.pub_compiled_markers.publish(self._mk_deleteall_marker_array(""))
            return

        self._cache[self.L_COMPILED].markers = msg
        if self._show[self.L_COMPILED]:
            self.pub_compiled_markers.publish(msg)

    def _on_planned_poses_in(self, msg: PoseArray) -> None:
        if msg is None or len(msg.poses) == 0:
            frame_id = (msg.header.frame_id if msg else "") or ""
            self._cache[self.L_PLANNED].poses = None
            if self._show[self.L_PLANNED]:
                self.pub_planned_poses.publish(self._mk_empty_pose_array(frame_id))
            return

        self._cache[self.L_PLANNED].poses = msg
        if self._show[self.L_PLANNED]:
            self.pub_planned_poses.publish(msg)

    def _on_planned_markers_in(self, msg: MarkerArray) -> None:
        if msg is None or len(msg.markers) == 0:
            self._cache[self.L_PLANNED].markers = None
            if self._show[self.L_PLANNED]:
                self.pub_planned_markers.publish(self._mk_deleteall_marker_array(""))
            return

        self._cache[self.L_PLANNED].markers = msg
        if self._show[self.L_PLANNED]:
            self.pub_planned_markers.publish(msg)

    def _on_executed_poses_in(self, msg: PoseArray) -> None:
        if msg is None or len(msg.poses) == 0:
            frame_id = (msg.header.frame_id if msg else "") or ""
            self._cache[self.L_EXECUTED].poses = None
            if self._show[self.L_EXECUTED]:
                self.pub_executed_poses.publish(self._mk_empty_pose_array(frame_id))
            return

        self._cache[self.L_EXECUTED].poses = msg
        if self._show[self.L_EXECUTED]:
            self.pub_executed_poses.publish(msg)

    def _on_executed_markers_in(self, msg: MarkerArray) -> None:
        if msg is None or len(msg.markers) == 0:
            self._cache[self.L_EXECUTED].markers = None
            if self._show[self.L_EXECUTED]:
                self.pub_executed_markers.publish(self._mk_deleteall_marker_array(""))
            return

        self._cache[self.L_EXECUTED].markers = msg
        if self._show[self.L_EXECUTED]:
            self.pub_executed_markers.publish(msg)

    # ------------------------------------------------------------------
    # 1 Hz republish safety net
    # ------------------------------------------------------------------

    def _republish_all(self) -> None:
        if self._show.get(self.L_COMPILED, True):
            self._publish_layer_cached_if_shown(self.L_COMPILED)
        if self._show.get(self.L_PLANNED, True):
            self._publish_layer_cached_if_shown(self.L_PLANNED)
        if self._show.get(self.L_EXECUTED, True):
            self._publish_layer_cached_if_shown(self.L_EXECUTED)


def main(args=None):
    rclpy.init(args=args)
    node = SprayPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
