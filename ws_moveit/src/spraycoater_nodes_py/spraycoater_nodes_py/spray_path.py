#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spraycoater_nodes_py/spray_path.py

SprayPath Cache Node (STRICT, 2026-01).
Implements the Latched-Cache pattern with explicit New vs. Stored separation.

Topics (see topics.yaml):
  Subs (Control):
    - clear
    - show_compiled
    - show_planned (MASTER: affects planned_run + planned_stored)
    - show_executed (MASTER: affects executed_run + executed_stored)
    - show_planned_run
    - show_planned_stored
    - show_executed_run
    - show_executed_stored

  Subs (Input - Cache filling):
    - compiled_poses_in, compiled_markers_in
    - planned_new_poses_in, planned_new_markers_in, planned_stored_markers_in
    - executed_new_poses_in, executed_new_markers_in, executed_stored_markers_in

  Pubs (Output - RViz):
    - compiled_poses, compiled_markers
    - planned_new_poses, planned_new_markers, planned_stored_markers
    - executed_new_poses, executed_new_markers, executed_stored_markers

Logic:
  - Inputs overwrite the specific cache slot immediately.
  - Compiled toggle controls compiled outputs.
  - Planned/Executed have split toggles for Run vs Stored.
  - Master toggles (show_planned/show_executed) set both split toggles.
  - 1Hz Timer republishes active layers (Safety Net).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Empty
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray, Marker

from spraycoater_nodes_py.utils.config_hub import topics


@dataclass
class _CacheSlot:
    compiled_poses: Optional[PoseArray] = None
    compiled_markers: Optional[MarkerArray] = None

    planned_new_poses: Optional[PoseArray] = None
    planned_new_markers: Optional[MarkerArray] = None
    planned_stored_markers: Optional[MarkerArray] = None

    executed_new_poses: Optional[PoseArray] = None
    executed_new_markers: Optional[MarkerArray] = None
    executed_stored_markers: Optional[MarkerArray] = None


class SprayPath(Node):
    GROUP = "spray_path"

    def __init__(self) -> None:
        super().__init__("spray_path")

        self.declare_parameter("backend", "default")
        self.backend: str = self.get_parameter("backend").get_parameter_value().string_value or "default"

        self.loader = topics()

        # ------------------------------------------------------------------
        # 1. Control Subscriptions (Toggles & Clear)
        # ------------------------------------------------------------------
        t_clear = self.loader.subscribe_topic(self.GROUP, "clear")
        q_clear = self.loader.qos_by_id("subscribe", self.GROUP, "clear")
        self.sub_clear = self.create_subscription(Empty, t_clear, self._on_clear, q_clear)

        t_show_compiled = self.loader.subscribe_topic(self.GROUP, "show_compiled")
        q_show_compiled = self.loader.qos_by_id("subscribe", self.GROUP, "show_compiled")
        self.sub_show_compiled = self.create_subscription(Bool, t_show_compiled, self._on_show_compiled, q_show_compiled)

        # Master toggles (backward compatible)
        t_show_planned = self.loader.subscribe_topic(self.GROUP, "show_planned")
        q_show_planned = self.loader.qos_by_id("subscribe", self.GROUP, "show_planned")
        self.sub_show_planned = self.create_subscription(Bool, t_show_planned, self._on_show_planned, q_show_planned)

        t_show_executed = self.loader.subscribe_topic(self.GROUP, "show_executed")
        q_show_executed = self.loader.qos_by_id("subscribe", self.GROUP, "show_executed")
        self.sub_show_executed = self.create_subscription(Bool, t_show_executed, self._on_show_executed, q_show_executed)

        # Split toggles (new)
        t_show_planned_run = self.loader.subscribe_topic(self.GROUP, "show_planned_run")
        q_show_planned_run = self.loader.qos_by_id("subscribe", self.GROUP, "show_planned_run")
        self.sub_show_planned_run = self.create_subscription(
            Bool, t_show_planned_run, self._on_show_planned_run, q_show_planned_run
        )

        t_show_planned_stored = self.loader.subscribe_topic(self.GROUP, "show_planned_stored")
        q_show_planned_stored = self.loader.qos_by_id("subscribe", self.GROUP, "show_planned_stored")
        self.sub_show_planned_stored = self.create_subscription(
            Bool, t_show_planned_stored, self._on_show_planned_stored, q_show_planned_stored
        )

        t_show_executed_run = self.loader.subscribe_topic(self.GROUP, "show_executed_run")
        q_show_executed_run = self.loader.qos_by_id("subscribe", self.GROUP, "show_executed_run")
        self.sub_show_executed_run = self.create_subscription(
            Bool, t_show_executed_run, self._on_show_executed_run, q_show_executed_run
        )

        t_show_executed_stored = self.loader.subscribe_topic(self.GROUP, "show_executed_stored")
        q_show_executed_stored = self.loader.qos_by_id("subscribe", self.GROUP, "show_executed_stored")
        self.sub_show_executed_stored = self.create_subscription(
            Bool, t_show_executed_stored, self._on_show_executed_stored, q_show_executed_stored
        )

        # ------------------------------------------------------------------
        # 2. Input Subscriptions (Data -> Cache)
        # ------------------------------------------------------------------

        # --- Compiled ---
        self.sub_cp_in = self.create_subscription(
            PoseArray,
            self.loader.subscribe_topic(self.GROUP, "compiled_poses_in"),
            self._on_compiled_poses_in,
            self.loader.qos_by_id("subscribe", self.GROUP, "compiled_poses_in"),
        )
        self.sub_cm_in = self.create_subscription(
            MarkerArray,
            self.loader.subscribe_topic(self.GROUP, "compiled_markers_in"),
            self._on_compiled_markers_in,
            self.loader.qos_by_id("subscribe", self.GROUP, "compiled_markers_in"),
        )

        # --- Planned (Run & Stored) ---
        self.sub_pnp_in = self.create_subscription(
            PoseArray,
            self.loader.subscribe_topic(self.GROUP, "planned_new_poses_in"),
            self._on_planned_new_poses_in,
            self.loader.qos_by_id("subscribe", self.GROUP, "planned_new_poses_in"),
        )
        self.sub_pnm_in = self.create_subscription(
            MarkerArray,
            self.loader.subscribe_topic(self.GROUP, "planned_new_markers_in"),
            self._on_planned_new_markers_in,
            self.loader.qos_by_id("subscribe", self.GROUP, "planned_new_markers_in"),
        )
        self.sub_psm_in = self.create_subscription(
            MarkerArray,
            self.loader.subscribe_topic(self.GROUP, "planned_stored_markers_in"),
            self._on_planned_stored_markers_in,
            self.loader.qos_by_id("subscribe", self.GROUP, "planned_stored_markers_in"),
        )

        # --- Executed (Run & Stored) ---
        self.sub_enp_in = self.create_subscription(
            PoseArray,
            self.loader.subscribe_topic(self.GROUP, "executed_new_poses_in"),
            self._on_executed_new_poses_in,
            self.loader.qos_by_id("subscribe", self.GROUP, "executed_new_poses_in"),
        )
        self.sub_enm_in = self.create_subscription(
            MarkerArray,
            self.loader.subscribe_topic(self.GROUP, "executed_new_markers_in"),
            self._on_executed_new_markers_in,
            self.loader.qos_by_id("subscribe", self.GROUP, "executed_new_markers_in"),
        )
        self.sub_esm_in = self.create_subscription(
            MarkerArray,
            self.loader.subscribe_topic(self.GROUP, "executed_stored_markers_in"),
            self._on_executed_stored_markers_in,
            self.loader.qos_by_id("subscribe", self.GROUP, "executed_stored_markers_in"),
        )

        # ------------------------------------------------------------------
        # 3. Output Publishers (Cache -> RViz)
        # ------------------------------------------------------------------

        # Compiled
        self.pub_cp = self.create_publisher(
            PoseArray,
            self.loader.publish_topic(self.GROUP, "compiled_poses"),
            self.loader.qos_by_id("publish", self.GROUP, "compiled_poses"),
        )
        self.pub_cm = self.create_publisher(
            MarkerArray,
            self.loader.publish_topic(self.GROUP, "compiled_markers"),
            self.loader.qos_by_id("publish", self.GROUP, "compiled_markers"),
        )

        # Planned
        self.pub_pnp = self.create_publisher(
            PoseArray,
            self.loader.publish_topic(self.GROUP, "planned_new_poses"),
            self.loader.qos_by_id("publish", self.GROUP, "planned_new_poses"),
        )
        self.pub_pnm = self.create_publisher(
            MarkerArray,
            self.loader.publish_topic(self.GROUP, "planned_new_markers"),
            self.loader.qos_by_id("publish", self.GROUP, "planned_new_markers"),
        )
        self.pub_psm = self.create_publisher(
            MarkerArray,
            self.loader.publish_topic(self.GROUP, "planned_stored_markers"),
            self.loader.qos_by_id("publish", self.GROUP, "planned_stored_markers"),
        )

        # Executed
        self.pub_enp = self.create_publisher(
            PoseArray,
            self.loader.publish_topic(self.GROUP, "executed_new_poses"),
            self.loader.qos_by_id("publish", self.GROUP, "executed_new_poses"),
        )
        self.pub_enm = self.create_publisher(
            MarkerArray,
            self.loader.publish_topic(self.GROUP, "executed_new_markers"),
            self.loader.qos_by_id("publish", self.GROUP, "executed_new_markers"),
        )
        self.pub_esm = self.create_publisher(
            MarkerArray,
            self.loader.publish_topic(self.GROUP, "executed_stored_markers"),
            self.loader.qos_by_id("publish", self.GROUP, "executed_stored_markers"),
        )

        # ------------------------------------------------------------------
        # 4. State
        # ------------------------------------------------------------------
        self._cache = _CacheSlot()

        self._show_compiled: bool = True

        # Split visibility (planned/executed)
        self._show_planned_run: bool = True
        self._show_planned_stored: bool = True
        self._show_executed_run: bool = True
        self._show_executed_stored: bool = True

        # Safety net: Republish active layers every 1.0s
        self._timer = self.create_timer(1.0, self._republish_all)

        self.get_logger().info(f"✅ SprayPath ready (backend='{self.backend}'). Planned/Executed split Run/Stored.")

    # ------------------------------------------------------------------
    # Logic: Publishing Helpers
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

    def _update_and_publish_pose(self, cache_attr: str, msg: PoseArray, is_shown: bool, publisher) -> None:
        """Generic handler for PoseArray inputs."""
        if msg is None or len(msg.poses) == 0:
            setattr(self._cache, cache_attr, None)
            if is_shown:
                fid = msg.header.frame_id if msg else ""
                publisher.publish(self._mk_empty_pose_array(fid))
            return

        setattr(self._cache, cache_attr, msg)
        if is_shown:
            publisher.publish(msg)

    def _update_and_publish_marker(self, cache_attr: str, msg: MarkerArray, is_shown: bool, publisher) -> None:
        """Generic handler for MarkerArray inputs."""
        if msg is None or len(msg.markers) == 0:
            setattr(self._cache, cache_attr, None)
            if is_shown:
                publisher.publish(self._mk_deleteall_marker_array(""))
            return

        setattr(self._cache, cache_attr, msg)
        if is_shown:
            publisher.publish(msg)

    # ------------------------------------------------------------------
    # Logic: Input Callbacks
    # ------------------------------------------------------------------

    # Compiled
    def _on_compiled_poses_in(self, msg: PoseArray) -> None:
        self._update_and_publish_pose("compiled_poses", msg, self._show_compiled, self.pub_cp)

    def _on_compiled_markers_in(self, msg: MarkerArray) -> None:
        self._update_and_publish_marker("compiled_markers", msg, self._show_compiled, self.pub_cm)

    # Planned (Run)
    def _on_planned_new_poses_in(self, msg: PoseArray) -> None:
        self._update_and_publish_pose("planned_new_poses", msg, self._show_planned_run, self.pub_pnp)

    def _on_planned_new_markers_in(self, msg: MarkerArray) -> None:
        self._update_and_publish_marker("planned_new_markers", msg, self._show_planned_run, self.pub_pnm)

    # Planned (Stored)
    def _on_planned_stored_markers_in(self, msg: MarkerArray) -> None:
        self._update_and_publish_marker("planned_stored_markers", msg, self._show_planned_stored, self.pub_psm)

    # Executed (Run)
    def _on_executed_new_poses_in(self, msg: PoseArray) -> None:
        self._update_and_publish_pose("executed_new_poses", msg, self._show_executed_run, self.pub_enp)

    def _on_executed_new_markers_in(self, msg: MarkerArray) -> None:
        self._update_and_publish_marker("executed_new_markers", msg, self._show_executed_run, self.pub_enm)

    # Executed (Stored)
    def _on_executed_stored_markers_in(self, msg: MarkerArray) -> None:
        self._update_and_publish_marker("executed_stored_markers", msg, self._show_executed_stored, self.pub_esm)

    # ------------------------------------------------------------------
    # Logic: Clear helpers (granular)
    # ------------------------------------------------------------------

    def _clear_all_caches(self) -> None:
        self._cache = _CacheSlot()

    def _clear_output_compiled(self) -> None:
        ea = self._mk_empty_pose_array("")
        da = self._mk_deleteall_marker_array("")
        self.pub_cp.publish(ea)
        self.pub_cm.publish(da)

    def _clear_output_planned_run(self) -> None:
        ea = self._mk_empty_pose_array("")
        da = self._mk_deleteall_marker_array("")
        self.pub_pnp.publish(ea)
        self.pub_pnm.publish(da)

    def _clear_output_planned_stored(self) -> None:
        da = self._mk_deleteall_marker_array("")
        self.pub_psm.publish(da)

    def _clear_output_executed_run(self) -> None:
        ea = self._mk_empty_pose_array("")
        da = self._mk_deleteall_marker_array("")
        self.pub_enp.publish(ea)
        self.pub_enm.publish(da)

    def _clear_output_executed_stored(self) -> None:
        da = self._mk_deleteall_marker_array("")
        self.pub_esm.publish(da)

    # ------------------------------------------------------------------
    # Logic: Republish helpers (granular)
    # ------------------------------------------------------------------

    def _republish_compiled_if_shown(self) -> None:
        if not self._show_compiled:
            return
        c = self._cache
        if c.compiled_poses:
            self.pub_cp.publish(c.compiled_poses)
        if c.compiled_markers:
            self.pub_cm.publish(c.compiled_markers)

    def _republish_planned_run_if_shown(self) -> None:
        if not self._show_planned_run:
            return
        c = self._cache
        if c.planned_new_poses:
            self.pub_pnp.publish(c.planned_new_poses)
        if c.planned_new_markers:
            self.pub_pnm.publish(c.planned_new_markers)

    def _republish_planned_stored_if_shown(self) -> None:
        if not self._show_planned_stored:
            return
        c = self._cache
        if c.planned_stored_markers:
            self.pub_psm.publish(c.planned_stored_markers)

    def _republish_executed_run_if_shown(self) -> None:
        if not self._show_executed_run:
            return
        c = self._cache
        if c.executed_new_poses:
            self.pub_enp.publish(c.executed_new_poses)
        if c.executed_new_markers:
            self.pub_enm.publish(c.executed_new_markers)

    def _republish_executed_stored_if_shown(self) -> None:
        if not self._show_executed_stored:
            return
        c = self._cache
        if c.executed_stored_markers:
            self.pub_esm.publish(c.executed_stored_markers)

    # ------------------------------------------------------------------
    # Logic: Toggles & Clear
    # ------------------------------------------------------------------

    def _on_clear(self, _msg: Empty) -> None:
        self._clear_all_caches()
        self._clear_output_compiled()
        self._clear_output_planned_run()
        self._clear_output_planned_stored()
        self._clear_output_executed_run()
        self._clear_output_executed_stored()
        self.get_logger().info("Clear command -> reset all.")

    def _on_show_compiled(self, msg: Bool) -> None:
        v = bool(msg.data)
        if v != self._show_compiled:
            self._show_compiled = v
            if v:
                self._republish_compiled_if_shown()
            else:
                self._clear_output_compiled()

    # Planned master (backward compatible): affects both run+stored
    def _on_show_planned(self, msg: Bool) -> None:
        v = bool(msg.data)

        if v != self._show_planned_run:
            self._show_planned_run = v
            if v:
                self._republish_planned_run_if_shown()
            else:
                self._clear_output_planned_run()

        if v != self._show_planned_stored:
            self._show_planned_stored = v
            if v:
                self._republish_planned_stored_if_shown()
            else:
                self._clear_output_planned_stored()

    # Executed master (backward compatible): affects both run+stored
    def _on_show_executed(self, msg: Bool) -> None:
        v = bool(msg.data)

        if v != self._show_executed_run:
            self._show_executed_run = v
            if v:
                self._republish_executed_run_if_shown()
            else:
                self._clear_output_executed_run()

        if v != self._show_executed_stored:
            self._show_executed_stored = v
            if v:
                self._republish_executed_stored_if_shown()
            else:
                self._clear_output_executed_stored()

    # Split planned toggles
    def _on_show_planned_run(self, msg: Bool) -> None:
        v = bool(msg.data)
        if v != self._show_planned_run:
            self._show_planned_run = v
            if v:
                self._republish_planned_run_if_shown()
            else:
                self._clear_output_planned_run()

    def _on_show_planned_stored(self, msg: Bool) -> None:
        v = bool(msg.data)
        if v != self._show_planned_stored:
            self._show_planned_stored = v
            if v:
                self._republish_planned_stored_if_shown()
            else:
                self._clear_output_planned_stored()

    # Split executed toggles
    def _on_show_executed_run(self, msg: Bool) -> None:
        v = bool(msg.data)
        if v != self._show_executed_run:
            self._show_executed_run = v
            if v:
                self._republish_executed_run_if_shown()
            else:
                self._clear_output_executed_run()

    def _on_show_executed_stored(self, msg: Bool) -> None:
        v = bool(msg.data)
        if v != self._show_executed_stored:
            self._show_executed_stored = v
            if v:
                self._republish_executed_stored_if_shown()
            else:
                self._clear_output_executed_stored()

    # ------------------------------------------------------------------
    # Timer (1Hz Safety Net)
    # ------------------------------------------------------------------
    def _republish_all(self) -> None:
        self._republish_compiled_if_shown()
        self._republish_planned_run_if_shown()
        self._republish_planned_stored_if_shown()
        self._republish_executed_run_if_shown()
        self._republish_executed_stored_if_shown()


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
