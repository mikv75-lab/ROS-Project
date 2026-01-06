# -*- coding: utf-8 -*-
#!/usr/bin/env python3
"""
spraycoater_nodes_py/spray_path.py

SprayPath Cache Node with PARALLEL outputs + BOOL toggles.

- RViz can show all 3 simultaneously (each on its own topic).
- GUI sends:
    - compiled/traj/executed poses_in + markers_in (latched)
    - show_compiled/show_traj/show_executed as Bool (latched)
- Node handles:
    - caching
    - (re)publishing on toggle ON
    - clearing outputs on toggle OFF (PoseArray empty + Marker.DELETEALL)
    - new recipe detection driven by compiled signature change

QoS:
- expected transient-local/latched via config_hub for all path topics
- optional 1 Hz republish timer as safety net for RViz/late subscribers
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray, Marker

from spraycoater_nodes_py.utils.config_hub import topics, frames


@dataclass
class _LayerCache:
    poses: Optional[PoseArray] = None
    markers: Optional[MarkerArray] = None


class SprayPath(Node):
    GROUP = "spray_path"

    L_COMPILED = "compiled"
    L_TRAJ = "traj"
    L_EXECUTED = "executed"

    def __init__(self) -> None:
        super().__init__("spray_path")

        self.declare_parameter("backend", "default")
        self.backend: str = self.get_parameter("backend").get_parameter_value().string_value or "default"

        self.loader = topics()
        self.frames = frames()

        # ---------------- SUB (toggles) ----------------
        t_show_compiled = self.loader.subscribe_topic(self.GROUP, "show_compiled")
        q_show_compiled = self.loader.qos_by_id("subscribe", self.GROUP, "show_compiled")

        t_show_traj = self.loader.subscribe_topic(self.GROUP, "show_traj")
        q_show_traj = self.loader.qos_by_id("subscribe", self.GROUP, "show_traj")

        t_show_executed = self.loader.subscribe_topic(self.GROUP, "show_executed")
        q_show_executed = self.loader.qos_by_id("subscribe", self.GROUP, "show_executed")

        # ---------------- SUB (inputs) ----------------
        t_compiled_poses_in = self.loader.subscribe_topic(self.GROUP, "compiled_poses_in")
        q_compiled_poses_in = self.loader.qos_by_id("subscribe", self.GROUP, "compiled_poses_in")

        t_compiled_markers_in = self.loader.subscribe_topic(self.GROUP, "compiled_markers_in")
        q_compiled_markers_in = self.loader.qos_by_id("subscribe", self.GROUP, "compiled_markers_in")

        t_traj_poses_in = self.loader.subscribe_topic(self.GROUP, "traj_poses_in")
        q_traj_poses_in = self.loader.qos_by_id("subscribe", self.GROUP, "traj_poses_in")

        t_traj_markers_in = self.loader.subscribe_topic(self.GROUP, "traj_markers_in")
        q_traj_markers_in = self.loader.qos_by_id("subscribe", self.GROUP, "traj_markers_in")

        t_executed_poses_in = self.loader.subscribe_topic(self.GROUP, "executed_poses_in")
        q_executed_poses_in = self.loader.qos_by_id("subscribe", self.GROUP, "executed_poses_in")

        t_executed_markers_in = self.loader.subscribe_topic(self.GROUP, "executed_markers_in")
        q_executed_markers_in = self.loader.qos_by_id("subscribe", self.GROUP, "executed_markers_in")

        # ---------------- PUB (outputs) ----------------
        t_compiled_poses = self.loader.publish_topic(self.GROUP, "compiled_poses")
        q_compiled_poses = self.loader.qos_by_id("publish", self.GROUP, "compiled_poses")

        t_compiled_markers = self.loader.publish_topic(self.GROUP, "compiled_markers")
        q_compiled_markers = self.loader.qos_by_id("publish", self.GROUP, "compiled_markers")

        t_traj_poses = self.loader.publish_topic(self.GROUP, "traj_poses")
        q_traj_poses = self.loader.qos_by_id("publish", self.GROUP, "traj_poses")

        t_traj_markers = self.loader.publish_topic(self.GROUP, "traj_markers")
        q_traj_markers = self.loader.qos_by_id("publish", self.GROUP, "traj_markers")

        t_executed_poses = self.loader.publish_topic(self.GROUP, "executed_poses")
        q_executed_poses = self.loader.qos_by_id("publish", self.GROUP, "executed_poses")

        t_executed_markers = self.loader.publish_topic(self.GROUP, "executed_markers")
        q_executed_markers = self.loader.qos_by_id("publish", self.GROUP, "executed_markers")

        # ---------------- ROS interfaces ----------------
        self.pub_compiled_poses = self.create_publisher(PoseArray, t_compiled_poses, q_compiled_poses)
        self.pub_compiled_markers = self.create_publisher(MarkerArray, t_compiled_markers, q_compiled_markers)

        self.pub_traj_poses = self.create_publisher(PoseArray, t_traj_poses, q_traj_poses)
        self.pub_traj_markers = self.create_publisher(MarkerArray, t_traj_markers, q_traj_markers)

        self.pub_executed_poses = self.create_publisher(PoseArray, t_executed_poses, q_executed_poses)
        self.pub_executed_markers = self.create_publisher(MarkerArray, t_executed_markers, q_executed_markers)

        self.sub_show_compiled = self.create_subscription(Bool, t_show_compiled, self._on_show_compiled, q_show_compiled)
        self.sub_show_traj = self.create_subscription(Bool, t_show_traj, self._on_show_traj, q_show_traj)
        self.sub_show_executed = self.create_subscription(Bool, t_show_executed, self._on_show_executed, q_show_executed)

        self.sub_compiled_poses = self.create_subscription(
            PoseArray, t_compiled_poses_in, self._on_compiled_poses_in, q_compiled_poses_in
        )
        self.sub_compiled_markers = self.create_subscription(
            MarkerArray, t_compiled_markers_in, self._on_compiled_markers_in, q_compiled_markers_in
        )

        self.sub_traj_poses = self.create_subscription(
            PoseArray, t_traj_poses_in, self._on_traj_poses_in, q_traj_poses_in
        )
        self.sub_traj_markers = self.create_subscription(
            MarkerArray, t_traj_markers_in, self._on_traj_markers_in, q_traj_markers_in
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
            self.L_TRAJ: _LayerCache(),
            self.L_EXECUTED: _LayerCache(),
        }

        # show flags (default TRUE)
        self._show: Dict[str, bool] = {
            self.L_COMPILED: True,
            self.L_TRAJ: True,
            self.L_EXECUTED: True,
        }

        # NEW-RECIPE detection (cheap signature)
        self._last_compiled_sig: Optional[Tuple[str, int]] = None
        self._last_compiled_marker_sig: Optional[Tuple[str, int]] = None

        # 1 Hz republish safety net
        self._timer = self.create_timer(1.0, self._republish_all)

        self.get_logger().info(
            f"✅ SprayPath ready (backend='{self.backend}'): parallel outputs + bool toggles "
            f"(default show_compiled/show_traj/show_executed = True)."
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _mk_empty_pose_array(self, frame_id: str = "") -> PoseArray:
        msg = PoseArray()
        msg.header.frame_id = frame_id or ""
        return msg

    def _mk_deleteall_marker_array(self, frame_id: str = "") -> MarkerArray:
        ma = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        m.header.frame_id = frame_id or ""
        ma.markers.append(m)
        return ma

    def _clear_cache_layer(self, key: str) -> None:
        c = self._cache.get(key)
        if c is not None:
            c.poses = None
            c.markers = None

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

    def _publish_layer_clear(self, key: str, *, frame_id: str = "") -> None:
        self._publish_layer_poses(key, self._mk_empty_pose_array(frame_id))
        self._publish_layer_markers(key, self._mk_deleteall_marker_array(frame_id))

    def _publish_layer_poses(self, key: str, msg: PoseArray) -> None:
        if key == self.L_COMPILED:
            self.pub_compiled_poses.publish(msg)
        elif key == self.L_TRAJ:
            self.pub_traj_poses.publish(msg)
        elif key == self.L_EXECUTED:
            self.pub_executed_poses.publish(msg)

    def _publish_layer_markers(self, key: str, msg: MarkerArray) -> None:
        if key == self.L_COMPILED:
            self.pub_compiled_markers.publish(msg)
        elif key == self.L_TRAJ:
            self.pub_traj_markers.publish(msg)
        elif key == self.L_EXECUTED:
            self.pub_executed_markers.publish(msg)

    # ------------------------------------------------------------------
    # New recipe detection
    # ------------------------------------------------------------------

    def _on_new_recipe(self, *, frame_id: str = "") -> None:
        # clear caches
        self._clear_cache_layer(self.L_COMPILED)
        self._clear_cache_layer(self.L_TRAJ)
        self._clear_cache_layer(self.L_EXECUTED)

        # hard-clear RViz layers (regardless of show flags)
        self._publish_layer_clear(self.L_COMPILED, frame_id=frame_id)
        self._publish_layer_clear(self.L_TRAJ, frame_id=frame_id)
        self._publish_layer_clear(self.L_EXECUTED, frame_id=frame_id)

        self.get_logger().info(f"[{self.backend}] 🧹 new recipe -> cleared all layers + DELETEALL")

    def _maybe_trigger_new_recipe_from_compiled_poses(self, msg: PoseArray) -> None:
        frame_id = (msg.header.frame_id or "").strip()
        sig = (frame_id, int(len(msg.poses)))
        if self._last_compiled_sig is None:
            self._last_compiled_sig = sig
            self._on_new_recipe(frame_id=frame_id)
            return
        if sig != self._last_compiled_sig:
            self._last_compiled_sig = sig
            self._on_new_recipe(frame_id=frame_id)

    def _maybe_trigger_new_recipe_from_compiled_markers(self, msg: MarkerArray) -> None:
        frame_id = ""
        if msg.markers:
            frame_id = (msg.markers[0].header.frame_id or "").strip()
        sig = (frame_id, int(len(msg.markers)))
        if self._last_compiled_marker_sig is None:
            self._last_compiled_marker_sig = sig
            self._on_new_recipe(frame_id=frame_id)
            return
        if sig != self._last_compiled_marker_sig:
            self._last_compiled_marker_sig = sig
            self._on_new_recipe(frame_id=frame_id)

    # ------------------------------------------------------------------
    # Toggle handlers (GUI -> Node), Bool only
    # ------------------------------------------------------------------

    def _set_show(self, key: str, v: bool) -> None:
        v = bool(v)
        prev = bool(self._show.get(key, True))
        self._show[key] = v

        if v and not prev:
            # turned ON: publish cached if any (otherwise keep empty)
            self._publish_layer_cached_if_shown(key)
            self.get_logger().info(f"[{self.backend}] 👁️ show_{key}=True -> republish cached")
            return

        if (not v) and prev:
            # turned OFF: clear output topics now
            frame_id = ""
            c = self._cache.get(key)
            if c and c.poses is not None:
                frame_id = (c.poses.header.frame_id or "").strip()
            self._publish_layer_clear(key, frame_id=frame_id)
            self.get_logger().info(f"[{self.backend}] 🙈 show_{key}=False -> clear outputs (DELETEALL)")
            return

        self.get_logger().info(f"[{self.backend}] show_{key}={v} (no change)")

    def _on_show_compiled(self, msg: Bool) -> None:
        self._set_show(self.L_COMPILED, bool(getattr(msg, "data", False)))

    def _on_show_traj(self, msg: Bool) -> None:
        self._set_show(self.L_TRAJ, bool(getattr(msg, "data", False)))

    def _on_show_executed(self, msg: Bool) -> None:
        self._set_show(self.L_EXECUTED, bool(getattr(msg, "data", False)))

    # ------------------------------------------------------------------
    # Input handlers (GUI -> Node) : compiled/traj/executed data
    # ------------------------------------------------------------------

    def _on_compiled_poses_in(self, msg: PoseArray) -> None:
        if msg is None or len(msg.poses) == 0:
            frame_id = (msg.header.frame_id if msg else "") or ""
            self._cache[self.L_COMPILED].poses = None
            # if shown: clear poses output (markers handled separately)
            if self._show[self.L_COMPILED]:
                self.pub_compiled_poses.publish(self._mk_empty_pose_array(frame_id))
            self.get_logger().info(f"[{self.backend}] 🧹 compiled poses cleared")
            return

        self._maybe_trigger_new_recipe_from_compiled_poses(msg)
        self._cache[self.L_COMPILED].poses = msg
        if self._show[self.L_COMPILED]:
            self.pub_compiled_poses.publish(msg)
        self.get_logger().info(f"[{self.backend}] 📌 compiled poses cached: {len(msg.poses)} (show={self._show[self.L_COMPILED]})")

    def _on_compiled_markers_in(self, msg: MarkerArray) -> None:
        if msg is None or len(msg.markers) == 0:
            self._cache[self.L_COMPILED].markers = None
            if self._show[self.L_COMPILED]:
                self.pub_compiled_markers.publish(self._mk_deleteall_marker_array(""))
            self.get_logger().info(f"[{self.backend}] 🧹 compiled markers cleared (DELETEALL)")
            return

        self._maybe_trigger_new_recipe_from_compiled_markers(msg)
        self._cache[self.L_COMPILED].markers = msg
        if self._show[self.L_COMPILED]:
            self.pub_compiled_markers.publish(msg)
        self.get_logger().info(f"[{self.backend}] 📌 compiled markers cached: {len(msg.markers)} (show={self._show[self.L_COMPILED]})")

    def _on_traj_poses_in(self, msg: PoseArray) -> None:
        if msg is None or len(msg.poses) == 0:
            frame_id = (msg.header.frame_id if msg else "") or ""
            self._cache[self.L_TRAJ].poses = None
            if self._show[self.L_TRAJ]:
                self.pub_traj_poses.publish(self._mk_empty_pose_array(frame_id))
            self.get_logger().info(f"[{self.backend}] 🧹 traj poses cleared")
            return

        self._cache[self.L_TRAJ].poses = msg
        if self._show[self.L_TRAJ]:
            self.pub_traj_poses.publish(msg)
        self.get_logger().info(f"[{self.backend}] 🧭 traj poses cached: {len(msg.poses)} (show={self._show[self.L_TRAJ]})")

    def _on_traj_markers_in(self, msg: MarkerArray) -> None:
        if msg is None or len(msg.markers) == 0:
            self._cache[self.L_TRAJ].markers = None
            if self._show[self.L_TRAJ]:
                self.pub_traj_markers.publish(self._mk_deleteall_marker_array(""))
            self.get_logger().info(f"[{self.backend}] 🧹 traj markers cleared (DELETEALL)")
            return

        self._cache[self.L_TRAJ].markers = msg
        if self._show[self.L_TRAJ]:
            self.pub_traj_markers.publish(msg)
        self.get_logger().info(f"[{self.backend}] 🧭 traj markers cached: {len(msg.markers)} (show={self._show[self.L_TRAJ]})")

    def _on_executed_poses_in(self, msg: PoseArray) -> None:
        if msg is None or len(msg.poses) == 0:
            frame_id = (msg.header.frame_id if msg else "") or ""
            self._cache[self.L_EXECUTED].poses = None
            if self._show[self.L_EXECUTED]:
                self.pub_executed_poses.publish(self._mk_empty_pose_array(frame_id))
            self.get_logger().info(f"[{self.backend}] 🧹 executed poses cleared")
            return

        self._cache[self.L_EXECUTED].poses = msg
        if self._show[self.L_EXECUTED]:
            self.pub_executed_poses.publish(msg)
        self.get_logger().info(f"[{self.backend}] ▶ executed poses cached: {len(msg.poses)} (show={self._show[self.L_EXECUTED]})")

    def _on_executed_markers_in(self, msg: MarkerArray) -> None:
        if msg is None or len(msg.markers) == 0:
            self._cache[self.L_EXECUTED].markers = None
            if self._show[self.L_EXECUTED]:
                self.pub_executed_markers.publish(self._mk_deleteall_marker_array(""))
            self.get_logger().info(f"[{self.backend}] 🧹 executed markers cleared (DELETEALL)")
            return

        self._cache[self.L_EXECUTED].markers = msg
        if self._show[self.L_EXECUTED]:
            self.pub_executed_markers.publish(msg)
        self.get_logger().info(f"[{self.backend}] ▶ executed markers cached: {len(msg.markers)} (show={self._show[self.L_EXECUTED]})")

    # ------------------------------------------------------------------
    # 1 Hz republish safety net
    # ------------------------------------------------------------------

    def _republish_all(self) -> None:
        # Only republish layers that are currently shown
        if self._show.get(self.L_COMPILED, True):
            self._publish_layer_cached_if_shown(self.L_COMPILED)
        if self._show.get(self.L_TRAJ, True):
            self._publish_layer_cached_if_shown(self.L_TRAJ)
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
