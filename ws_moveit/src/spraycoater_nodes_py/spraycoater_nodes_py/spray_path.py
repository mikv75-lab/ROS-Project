# -*- coding: utf-8 -*-
#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray, Marker

from spraycoater_nodes_py.utils.config_hub import topics, frames


@dataclass
class _ViewCache:
    poses: Optional[PoseArray] = None
    markers: Optional[MarkerArray] = None


class SprayPath(Node):
    """
    SprayPath Router/Cache Node (compiled + traj + executed overlay).

    Inputs (subscribe):
      - compiled_path: compiled_poses_in / compiled_markers_in
      - traj:          traj_poses_in     / traj_markers_in
      - executed:      executed_poses_in / executed_markers_in
      - set_view (String): selects what is published on main poses/markers

    Outputs (publish):
      - current (String, latched-like)
      - poses/markers (latched-like): currently selected main view
      - executed_poses/executed_markers (latched-like): overlay, always updated

    IMPORTANT:
      - On NEW RECIPE (new compiled_path), we CLEAR old traj/executed + send DELETEALL markers.
    """

    GROUP = "spray_path"

    VIEW_COMPILED = "compiled_path"
    VIEW_TRAJ = "traj"
    VIEW_EXECUTED = "executed_traj"

    def __init__(self) -> None:
        super().__init__("spray_path")

        # ---------------- Backend-Parameter ----------------
        self.declare_parameter("backend", "default")
        self.backend: str = self.get_parameter("backend").get_parameter_value().string_value or "default"

        # Config-Hub
        self.loader = topics()
        self.frames = frames()

        # ---------------- Topics & QoS via config_hub ----------------
        # SUB
        t_set_view = self.loader.subscribe_topic(self.GROUP, "set_view")
        q_set_view = self.loader.qos_by_id("subscribe", self.GROUP, "set_view")

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

        # PUB
        t_current = self.loader.publish_topic(self.GROUP, "current")
        q_current = self.loader.qos_by_id("publish", self.GROUP, "current")

        t_poses = self.loader.publish_topic(self.GROUP, "poses")
        q_poses = self.loader.qos_by_id("publish", self.GROUP, "poses")

        t_markers = self.loader.publish_topic(self.GROUP, "markers")
        q_markers = self.loader.qos_by_id("publish", self.GROUP, "markers")

        t_exec_poses = self.loader.publish_topic(self.GROUP, "executed_poses")
        q_exec_poses = self.loader.qos_by_id("publish", self.GROUP, "executed_poses")

        t_exec_markers = self.loader.publish_topic(self.GROUP, "executed_markers")
        q_exec_markers = self.loader.qos_by_id("publish", self.GROUP, "executed_markers")

        # ---------------- Publisher / Subscriber ----------------
        self.pub_current = self.create_publisher(String, t_current, q_current)
        self.pub_poses = self.create_publisher(PoseArray, t_poses, q_poses)
        self.pub_markers = self.create_publisher(MarkerArray, t_markers, q_markers)
        self.pub_exec_poses = self.create_publisher(PoseArray, t_exec_poses, q_exec_poses)
        self.pub_exec_markers = self.create_publisher(MarkerArray, t_exec_markers, q_exec_markers)

        self.sub_set_view = self.create_subscription(String, t_set_view, self._on_set_view, q_set_view)

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

        # ---------------- Internal state ----------------
        self._current_view: str = ""
        self._views: Dict[str, _ViewCache] = {
            self.VIEW_COMPILED: _ViewCache(),
            self.VIEW_TRAJ: _ViewCache(),
            self.VIEW_EXECUTED: _ViewCache(),
        }

        # NEW-RECIPE detection (cheap signature)
        self._last_compiled_sig: Optional[Tuple[str, int]] = None
        self._last_compiled_marker_sig: Optional[Tuple[str, int]] = None

        # 1 Hz Republish Timer (helps late subscribers & keeps "latched-like" behavior consistent)
        self._timer = self.create_timer(1.0, self._republish_all)

        self.get_logger().info(
            f"✅ SprayPath ready (backend='{self.backend}'). "
            f"Inputs: compiled + traj + executed + set_view. "
            f"Outputs: current + poses/markers + executed_* overlay. QoS via config_hub + 1Hz republish."
        )

    # ----------------------------------------------------------------------
    # Clear helpers
    # ----------------------------------------------------------------------

    def _mk_empty_pose_array(self, frame_id: str = "") -> PoseArray:
        msg = PoseArray()
        msg.header.frame_id = frame_id or ""
        # stamp optional; leaving empty is ok for RViz, but you can stamp if you want
        return msg

    def _mk_deleteall_marker_array(self, frame_id: str = "") -> MarkerArray:
        ma = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        # frame_id isn't strictly required for DELETEALL but doesn't hurt
        m.header.frame_id = frame_id or ""
        ma.markers.append(m)
        return ma

    def _clear_view_cache(self, key: str) -> None:
        if key in self._views:
            self._views[key].poses = None
            self._views[key].markers = None

    def _clear_all_outputs_now(self, *, frame_id: str = "") -> None:
        """
        Hard-clear what RViz/consumers currently see.
        - main poses/markers
        - executed overlay poses/markers
        """
        # main outputs
        self.pub_poses.publish(self._mk_empty_pose_array(frame_id))
        self.pub_markers.publish(self._mk_deleteall_marker_array(frame_id))

        # overlay outputs
        self.pub_exec_poses.publish(self._mk_empty_pose_array(frame_id))
        self.pub_exec_markers.publish(self._mk_deleteall_marker_array(frame_id))

    def _on_new_recipe(self, *, frame_id: str = "") -> None:
        """
        Called when compiled_path indicates a new recipe was loaded.
        Must delete old traj/executed from caches AND from RViz.
        """
        # Clear caches that must not persist across recipes
        self._clear_view_cache(self.VIEW_TRAJ)
        self._clear_view_cache(self.VIEW_EXECUTED)

        # Also clear compiled markers/poses first (avoid mixing if only one stream arrives)
        self._clear_view_cache(self.VIEW_COMPILED)

        # Clear visible outputs right away (RViz)
        self._clear_all_outputs_now(frame_id=frame_id)

        # If current view was traj/executed, reset it (compiled will be auto-selected again)
        if self._current_view in (self.VIEW_TRAJ, self.VIEW_EXECUTED):
            self._current_view = ""

        self.get_logger().info(f"[{self.backend}] 🧹 new recipe detected -> cleared old traj/executed + DELETEALL markers")

    # ----------------------------------------------------------------------
    # View routing
    # ----------------------------------------------------------------------

    def _set_current_view(self, key: str) -> None:
        key = (key or "").strip()
        if not key:
            return

        if key not in self._views:
            self.get_logger().warning(f"[{self.backend}] ⚠️ set_view: unknown key '{key}'")
            return

        self._current_view = key
        self.pub_current.publish(String(data=key))
        self._publish_selected_view()

    def _publish_selected_view(self) -> None:
        key = self._current_view
        if not key:
            return
        cache = self._views.get(key)
        if cache is None:
            return
        if cache.poses is not None:
            self.pub_poses.publish(cache.poses)
        if cache.markers is not None:
            self.pub_markers.publish(cache.markers)

    def _auto_select_if_empty(self, key: str) -> None:
        if not self._current_view:
            self._set_current_view(key)

    # ----------------------------------------------------------------------
    # Handlers: set_view
    # ----------------------------------------------------------------------

    def _on_set_view(self, msg: String) -> None:
        key = (msg.data or "").strip()
        if not key:
            self.get_logger().warning(f"[{self.backend}] ⚠️ set_view: empty")
            return
        self._set_current_view(key)
        self.get_logger().info(f"[{self.backend}] 👁️ view selected: {key}")

    # ----------------------------------------------------------------------
    # Handlers: compiled_path (NEW RECIPE trigger)
    # ----------------------------------------------------------------------

    def _maybe_trigger_new_recipe_from_compiled_poses(self, msg: PoseArray) -> None:
        frame_id = (msg.header.frame_id or "").strip()
        sig = (frame_id, int(len(msg.poses)))
        if self._last_compiled_sig is None:
            self._last_compiled_sig = sig
            # first ever: treat as new recipe as well (ensures clean start)
            self._on_new_recipe(frame_id=frame_id)
            return
        if sig != self._last_compiled_sig:
            self._last_compiled_sig = sig
            self._on_new_recipe(frame_id=frame_id)

    def _maybe_trigger_new_recipe_from_compiled_markers(self, msg: MarkerArray) -> None:
        # if markers have frame_id, take first marker's frame as hint
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

    def _on_compiled_poses_in(self, msg: PoseArray) -> None:
        # ✅ accept empty as "clear compiled"
        if msg is None or len(msg.poses) == 0:
            self.get_logger().info(f"[{self.backend}] 🧹 compiled_path/poses_in: empty -> clearing compiled view")
            self._views[self.VIEW_COMPILED].poses = None
            if self._current_view == self.VIEW_COMPILED:
                self.pub_poses.publish(self._mk_empty_pose_array((msg.header.frame_id if msg else "") or ""))
            return

        # NEW RECIPE detection
        self._maybe_trigger_new_recipe_from_compiled_poses(msg)

        # cache + publish
        self._views[self.VIEW_COMPILED].poses = msg
        self._auto_select_if_empty(self.VIEW_COMPILED)

        if self._current_view == self.VIEW_COMPILED:
            self.pub_poses.publish(msg)
            self.pub_current.publish(String(data=self.VIEW_COMPILED))

        self.get_logger().info(f"[{self.backend}] 📌 compiled poses cached: poses={len(msg.poses)}")

    def _on_compiled_markers_in(self, msg: MarkerArray) -> None:
        # ✅ accept empty as "clear compiled markers"
        if msg is None or len(msg.markers) == 0:
            self.get_logger().info(f"[{self.backend}] 🧹 compiled_path/markers_in: empty -> clearing compiled markers")
            self._views[self.VIEW_COMPILED].markers = None
            if self._current_view == self.VIEW_COMPILED:
                self.pub_markers.publish(self._mk_deleteall_marker_array(""))
            return

        # NEW RECIPE detection (in case markers arrive before poses)
        self._maybe_trigger_new_recipe_from_compiled_markers(msg)

        self._views[self.VIEW_COMPILED].markers = msg
        self._auto_select_if_empty(self.VIEW_COMPILED)

        if self._current_view == self.VIEW_COMPILED:
            self.pub_markers.publish(msg)
            self.pub_current.publish(String(data=self.VIEW_COMPILED))

        self.get_logger().info(f"[{self.backend}] 📌 compiled markers cached: markers={len(msg.markers)}")

    # ----------------------------------------------------------------------
    # Handlers: traj (generic slot)
    # ----------------------------------------------------------------------

    def _on_traj_poses_in(self, msg: PoseArray) -> None:
        # accept empty as clear
        if msg is None or len(msg.poses) == 0:
            self.get_logger().info(f"[{self.backend}] 🧹 traj/poses_in: empty -> clearing traj view")
            self._views[self.VIEW_TRAJ].poses = None
            if self._current_view == self.VIEW_TRAJ:
                self.pub_poses.publish(self._mk_empty_pose_array((msg.header.frame_id if msg else "") or ""))
            return

        self._views[self.VIEW_TRAJ].poses = msg
        self._auto_select_if_empty(self.VIEW_TRAJ)

        if self._current_view == self.VIEW_TRAJ:
            self.pub_poses.publish(msg)
            self.pub_current.publish(String(data=self.VIEW_TRAJ))

        self.get_logger().info(f"[{self.backend}] 🧭 traj poses cached: poses={len(msg.poses)}")

    def _on_traj_markers_in(self, msg: MarkerArray) -> None:
        # accept empty as clear
        if msg is None or len(msg.markers) == 0:
            self.get_logger().info(f"[{self.backend}] 🧹 traj/markers_in: empty -> clearing traj markers")
            self._views[self.VIEW_TRAJ].markers = None
            if self._current_view == self.VIEW_TRAJ:
                self.pub_markers.publish(self._mk_deleteall_marker_array(""))
            return

        self._views[self.VIEW_TRAJ].markers = msg
        self._auto_select_if_empty(self.VIEW_TRAJ)

        if self._current_view == self.VIEW_TRAJ:
            self.pub_markers.publish(msg)
            self.pub_current.publish(String(data=self.VIEW_TRAJ))

        self.get_logger().info(f"[{self.backend}] 🧭 traj markers cached: markers={len(msg.markers)}")

    # ----------------------------------------------------------------------
    # Handlers: executed overlay
    # ----------------------------------------------------------------------

    def _on_executed_poses_in(self, msg: PoseArray) -> None:
        # accept empty as clear
        if msg is None or len(msg.poses) == 0:
            self.get_logger().info(f"[{self.backend}] 🧹 executed_traj/poses_in: empty -> clearing executed overlay")
            self._views[self.VIEW_EXECUTED].poses = None
            self.pub_exec_poses.publish(self._mk_empty_pose_array((msg.header.frame_id if msg else "") or ""))
            if self._current_view == self.VIEW_EXECUTED:
                self.pub_poses.publish(self._mk_empty_pose_array((msg.header.frame_id if msg else "") or ""))
            return

        self._views[self.VIEW_EXECUTED].poses = msg

        # overlay always publishes
        self.pub_exec_poses.publish(msg)

        # if selected as main, also publish there
        if self._current_view == self.VIEW_EXECUTED:
            self.pub_poses.publish(msg)
            self.pub_current.publish(String(data=self.VIEW_EXECUTED))

        self.get_logger().info(f"[{self.backend}] ▶ executed poses cached: poses={len(msg.poses)}")

    def _on_executed_markers_in(self, msg: MarkerArray) -> None:
        # accept empty as clear
        if msg is None or len(msg.markers) == 0:
            self.get_logger().info(f"[{self.backend}] 🧹 executed_traj/markers_in: empty -> clearing executed markers")
            self._views[self.VIEW_EXECUTED].markers = None
            self.pub_exec_markers.publish(self._mk_deleteall_marker_array(""))
            if self._current_view == self.VIEW_EXECUTED:
                self.pub_markers.publish(self._mk_deleteall_marker_array(""))
            return

        self._views[self.VIEW_EXECUTED].markers = msg

        # overlay always publishes
        self.pub_exec_markers.publish(msg)

        # if selected as main, also publish there
        if self._current_view == self.VIEW_EXECUTED:
            self.pub_markers.publish(msg)
            self.pub_current.publish(String(data=self.VIEW_EXECUTED))

        self.get_logger().info(f"[{self.backend}] ▶ executed markers cached: markers={len(msg.markers)}")

    # ----------------------------------------------------------------------
    # 1 Hz REPUBLISH
    # ----------------------------------------------------------------------

    def _republish_all(self) -> None:
        # main selected view
        self._publish_selected_view()
        if self._current_view:
            self.pub_current.publish(String(data=self._current_view))

        # executed overlay (independent)
        ex = self._views.get(self.VIEW_EXECUTED)
        if ex and ex.poses is not None:
            self.pub_exec_poses.publish(ex.poses)
        if ex and ex.markers is not None:
            self.pub_exec_markers.publish(ex.markers)


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
