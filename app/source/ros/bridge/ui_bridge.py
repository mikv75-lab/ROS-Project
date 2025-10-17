#!/usr/bin/env python3
from __future__ import annotations

from typing import Optional, Sequence

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from ..common.topics import Topics
from ..common.frames import FRAMES, FramesNS
from ..clients.tool_client import ToolClient
from ..clients.poses_client import PosesClient
from ..clients.motion_client import MotionClient
from ..clients.jogging_client import JoggingClient
from ..clients.scene_client import SceneClient  # <-- NEU: Scene statt Substrate


class UIBridge(Node):
    """
    Zentrale Brücke zwischen UI und ROS-Backend.
    Die UI spricht nur über diese Klasse und niemals direkt mit ROS.

    Bereitgestellte Clients:
        bridge.tool       -> ToolClient
        bridge.poses      -> PosesClient
        bridge.motion     -> MotionClient
        bridge.jog        -> JoggingClient
        bridge.scene      -> SceneClient   (NEU: steuert Mount + Substrat + Spray + Cage)
    """

    def __init__(
        self,
        node_name: str = "ui_bridge",
        *,
        controller: str = "meca_arm_group_controller",
        servo_ns: str = "moveit_servo",
        frames: Optional[FramesNS] = None,
        ee_link_candidates: Sequence[Optional[str]] = ("tcp", "meca_axis_6_link", "tool0", "flange", None),
        group_name: str = "meca_arm_group",
    ) -> None:
        super().__init__(node_name)

        self._frames = frames or FRAMES
        self._topics = Topics(controller=controller, servo_ns=servo_ns)

        # ROS-Clients
        self.tool = ToolClient(self, self._topics)
        self.poses = PosesClient(self, self._topics)
        self.scene = SceneClient(self, self._topics, frames=self._frames)  # <-- ersetzt SubstrateClient

        self.motion = MotionClient(
            self,
            self._topics,
            group_name=group_name,
            ee_link_candidates=ee_link_candidates,
            pose_resolver=self._resolve_pose_via_poses_client,
        )

        self.jog = JoggingClient(self, self._topics, default_frame=self._frames["meca_base"])

        self.get_logger().info("✅ UIBridge bereit (SceneClient aktiv).")

    # ─────────────────────────────────────────────────────────────
    # Controller/Servo dynamisch ändern
    # ─────────────────────────────────────────────────────────────
    def set_controller(self, name: str) -> None:
        old = self._topics.controller
        self._topics = Topics(base_ns=self._topics.base_ns, controller=name, servo_ns=self._topics.servo_ns)
        self.motion = MotionClient(
            self,
            self._topics,
            group_name=self.motion._group,
            ee_link_candidates=self.motion._ee_links,
            pose_resolver=self._resolve_pose_via_poses_client,
        )
        self.get_logger().info(f"✅ Controller geändert: {old} -> {name}")

    def set_servo_ns(self, ns: str) -> None:
        old = self._topics.servo_ns
        self._topics = Topics(base_ns=self._topics.base_ns, controller=self._topics.controller, servo_ns=ns)
        self.jog = JoggingClient(self, self._topics, default_frame=self._frames["meca_base"])
        self.get_logger().info(f"✅ Servo NS geändert: {old} -> {ns}")

    # ─────────────────────────────────────────────────────────────
    # SCENE Wrapper (für UI)
    # ─────────────────────────────────────────────────────────────

    # Mount laden
    def scene_load_mount(self, stl_path: str) -> bool:
        return self.scene.load_mount(stl_path)

    # Substrat laden/entfernen
    def scene_load_workpiece(self, stl_path: str) -> bool:
        return self.scene.load_workpiece(stl_path)

    def scene_remove_workpiece(self) -> None:
        self.scene.remove_workpiece()

    # Environment/Cage laden
    def scene_load_environment(self, stl_path: str) -> bool:
        return self.scene.load_environment(stl_path)

    # Spray Path
    def scene_set_spray_path(self, pose_array) -> None:
        self.scene.set_spray_path(pose_array)

    def scene_clear_spray_path(self) -> None:
        self.scene.clear_spray_path()

    # ─────────────────────────────────────────────────────────────
    # Tool-Funktionen
    # ─────────────────────────────────────────────────────────────
    def tool_set(self, name: str, *, wait: bool = True, timeout: float = 2.0) -> bool:
        return self.tool.set_tool(name, wait_for_confirm=wait, timeout=timeout)

    def tool_current(self) -> str:
        return self.tool.get_current() or "—"

    # ─────────────────────────────────────────────────────────────
    # Pose Funktionen
    # ─────────────────────────────────────────────────────────────
    def pose_get(self, name: str, *, wait: bool = False, timeout: float = 1.5) -> Optional[PoseStamped]:
        if wait:
            return self.poses.wait_for(name, timeout=timeout)
        return self.poses.get(name)

    def pose_set_from_tcp(self, name: str, *, wait: bool = True, timeout: float = 2.0) -> bool:
        return self.poses.set_from_tcp(name, wait_for_update=wait, timeout=timeout)

    # ─────────────────────────────────────────────────────────────
    # Motion
    # ─────────────────────────────────────────────────────────────
    def move_to(self, name: str, *, use_planner: bool = False, plan_only: bool = False, avoid_collisions: bool = False) -> bool:
        return self.motion.move_to_named_pose(
            name,
            use_planner=use_planner,
            plan_only=plan_only,
            avoid_collisions=avoid_collisions,
        )

    def move_to_ps(self, ps: PoseStamped, *, use_planner: bool = False, plan_only: bool = False, avoid_collisions: bool = False) -> bool:
        return self.motion.move_to_pose_stamped(ps, use_planner=use_planner, plan_only=plan_only, avoid_collisions=avoid_collisions)

    # ─────────────────────────────────────────────────────────────
    # Jogging
    # ─────────────────────────────────────────────────────────────
    def jog_enable(self) -> None:
        self.jog.enable()

    def jog_disable(self) -> None:
        self.jog.disable(send_stop=True)

    # ─────────────────────────────────────────────────────────────
    # Intern
    # ─────────────────────────────────────────────────────────────
    def _resolve_pose_via_poses_client(self, name: str) -> Optional[PoseStamped]:
        ps = self.poses.get(name)
        if ps is None:
            return self.poses.wait_for(name, timeout=1.2, trigger_republish=True)
        return ps
