#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import math
import yaml
import importlib

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Pose

from ament_index_python.packages import get_package_share_directory

# project-common
from mecademic_bringup.common.topics import TOPIC_TOOL_SET, TOPIC_TOOL_CURRENT
from mecademic_bringup.common.params import PARAM_TOOL_CONFIG

# optional: MoveIt Commander (für Collision-Mesh)
try:
    import moveit_commander
    HAVE_MOVEIT = True
except Exception:
    HAVE_MOVEIT = False


def rpy_deg_to_quat(r, p, y):
    rx, ry, rz = map(math.radians, (r, p, y))
    cr, sr = math.cos(rx / 2.0), math.sin(rx / 2.0)
    cp, sp = math.cos(ry / 2.0), math.sin(ry / 2.0)
    cy, sy = math.cos(rz / 2.0), math.sin(rz / 2.0)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) or 1.0
    return (qx / n, qy / n, qz / n, qw / n)


def resolve_package_url(url: str) -> str:
    """package://<pkg>/rel/path -> absoluter Pfad ins share/ des Pakets."""
    if not url or not url.startswith("package://"):
        return url
    pkg, rel = url[len("package://"):].split("/", 1)
    return os.path.join(get_package_share_directory(pkg), rel)


class ToolManager(Node):
    def __init__(self):
        super().__init__("tool_manager")

        # --- tools.yaml Parameter ---
        default_yaml = os.path.join(get_package_share_directory("mecademic_bringup"), "config", "tools.yaml")
        self.declare_parameter(PARAM_TOOL_CONFIG, default_yaml)
        self.tools_yaml_path = self.get_parameter(PARAM_TOOL_CONFIG).get_parameter_value().string_value or default_yaml

        # --- YAML laden ---
        self.tools_data = self._load_yaml(self.tools_yaml_path)
        self.tools = self.tools_data.get("tools", {})
        self.current_tool = self.tools_data.get("active_tool", "none")

        # --- TF Broadcaster für TCP ---
        self.static_tf = StaticTransformBroadcaster(self)

        # --- (optional) MoveIt-Scene für Collision-Mesh ---
        if HAVE_MOVEIT:
            try:
                moveit_commander.roscpp_initialize(sys.argv)
            except Exception:
                pass
            self.scene = moveit_commander.PlanningSceneInterface(synchronous=True)
            self.get_logger().info("🧩 MoveIt Commander verfügbar – Collision-Mesh wird unterstützt.")
        else:
            self.scene = None
            self.get_logger().warning("🧩 MoveIt Commander NICHT gefunden – Mesh-Attach wird übersprungen (nur TF/TCP).")

        # --- Topics ---
        self.pub_current = self.create_publisher(String, TOPIC_TOOL_CURRENT, 10)
        self.create_subscription(String, TOPIC_TOOL_SET, self.on_tool_change, 10)

        # --- Startzustand anwenden ---
        self._apply_tool(self.current_tool, init=True)
        self.get_logger().info(f"✅ ToolManager aktiv – aktuelles Tool: '{self.current_tool}'")

    # ---------------- YAML ----------------
    def _load_yaml(self, path: str):
        if not os.path.exists(path):
            raise FileNotFoundError(f"tools.yaml nicht gefunden: {path}")
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}

    def _save_active_tool(self):
        self.tools_data["active_tool"] = self.current_tool
        with open(self.tools_yaml_path, "w") as f:
            yaml.safe_dump(self.tools_data, f)
        self.get_logger().info(f"💾 tools.yaml aktualisiert: active_tool = '{self.current_tool}'")

    # ------------- Hooks (optional) -------------
    def _load_plugin(self, tool_name: str):
        mod_name = f"mecademic_bringup.tools.{tool_name}"
        try:
            mod = importlib.import_module(mod_name)
            return getattr(mod, "ToolPlugin")()
        except Exception as e:
            # Hooks sind optional – wir loggen nur.
            self.get_logger().info(f"ℹ️ Kein Plugin für '{tool_name}' geladen ({e})")
            class _Noop:
                def on_attach(self, *a, **k): pass
                def on_detach(self, *a, **k): pass
            return _Noop()

    # ------------- TCP TF Publish --------------
    def _publish_tcp_tf(self, mount_frame: str, tcp_xyz, tcp_rpy_deg, child_name: str = "tcp"):
        qx, qy, qz, qw = rpy_deg_to_quat(*tcp_rpy_deg)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = mount_frame
        t.child_frame_id = child_name
        t.transform.translation.x = float(tcp_xyz[0] or 0.0)
        t.transform.translation.y = float(tcp_xyz[1] or 0.0)
        t.transform.translation.z = float(tcp_xyz[2] or 0.0)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.static_tf.sendTransform([t])
        self.get_logger().info(f"📡 TF gesetzt: {mount_frame} -> {child_name}  xyz={tcp_xyz}  rpy(deg)={tcp_rpy_deg}")

    # ------------- Mesh Attach / Detach --------------
    def _attach_mesh(self, tool_name: str, mount_link: str, mesh_uri: str, mesh_pose: Pose | None = None, scale=(1.0, 1.0, 1.0)):
        if not HAVE_MOVEIT:
            self.get_logger().info(f"🧩 Überspringe Mesh-Attach (MoveIt Commander fehlt): {tool_name}")
            return
        mesh_path = resolve_package_url(mesh_uri)
        if not os.path.exists(mesh_path):
            self.get_logger().warning(f"⚠️ Mesh-Datei nicht gefunden: {mesh_path} – versuche dennoch zu attachen.")
        pose = mesh_pose or Pose()
        # Vorheriges mit gleichem Namen entfernen
        try:
            self.scene.remove_attached_object(mount_link, name=tool_name)
            self.scene.remove_world_object(tool_name)
        except Exception:
            pass
        # Welt hinzufügen & attachen
        self.scene.add_mesh(name=tool_name, pose=pose, filename=mesh_path, size=scale)
        self.scene.attach_mesh(mount_link, name=tool_name, pose=pose, filename=mesh_path, size=scale)
        self.get_logger().info(f"🧩 Mesh attached: tool='{tool_name}', link='{mount_link}', file='{mesh_path}', scale={scale}")

    def _detach_mesh(self, tool_name: str, mount_link: str):
        if not HAVE_MOVEIT:
            return
        try:
            self.scene.remove_attached_object(mount_link, name=tool_name)
        except Exception:
            pass
        try:
            self.scene.remove_world_object(tool_name)
        except Exception:
            pass
        self.get_logger().info(f"🗑 Mesh detached/removed: '{tool_name}'")

    # ------------- Apply & Clear --------------
    def _apply_tool(self, tool_name: str, init: bool = False):
        if tool_name not in self.tools:
            raise RuntimeError(f"Unbekanntes Tool: '{tool_name}' (nicht in tools.yaml)")

        cfg = self.tools[tool_name]
        mount = cfg.get("mount_frame", "tool_mount")
        tcp_xyz = cfg.get("tcp_offset", [0.0, 0.0, 0.0])
        tcp_rpy = cfg.get("tcp_rpy", [0.0, 0.0, 0.0])
        mesh_uri = cfg.get("mesh", "")
        scale = tuple(cfg.get("mesh_scale", [1.0, 1.0, 1.0]))

        # 1) TCP TF setzen
        self._publish_tcp_tf(mount, tcp_xyz, tcp_rpy, child_name="tcp")

        # 2) Mesh attachen (optional)
        if mesh_uri:
            self._attach_mesh(tool_name, mount, mesh_uri, Pose(), scale)
        else:
            self.get_logger().info(f"ℹ️ Tool '{tool_name}' hat kein Mesh – überspringe Collision-Attach")

        # 3) Hook on_attach
        self._load_plugin(tool_name).on_attach(self)

        # 4) Status publishen + persistent speichern
        if not init:
            self.current_tool = tool_name
            self._save_active_tool()
        self.pub_current.publish(String(data=tool_name))

    def _clear_current_tool(self):
        # Hook on_detach
        try:
            self._load_plugin(self.current_tool).on_detach(self)
        except Exception as e:
            self.get_logger().warning(f"Hook on_detach Fehler: {e}")

        # Mesh entfernen
        old_cfg = self.tools.get(self.current_tool, {})
        old_mount = old_cfg.get("mount_frame", "tool_mount")
        self._detach_mesh(self.current_tool, old_mount)

        # TCP neutral auf mount_frame zurücksetzen
        self._publish_tcp_tf(old_mount, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], child_name="tcp")

    # ------------- Topic Callback --------------
    def on_tool_change(self, msg: String):
        new_tool = (msg.data or "").strip()
        if new_tool == self.current_tool:
            self.get_logger().info(f"ℹ️ Tool bereits aktiv: '{new_tool}'")
            return
        if new_tool not in self.tools:
            self.get_logger().error(f"❌ Unbekanntes Tool: '{new_tool}' (nicht in tools.yaml)")
            return

        self.get_logger().info(f"🔄 Toolwechsel: {self.current_tool} → {new_tool}")

        if self.current_tool in self.tools:
            self._clear_current_tool()

        self._apply_tool(new_tool)
        self.get_logger().info(f"✅ Aktiv: '{new_tool}' (ohne Neustart)")

def main():
    rclpy.init()
    node = ToolManager()
    try:
        rclpy.spin(node)
    finally:
        if HAVE_MOVEIT:
            try:
                moveit_commander.roscpp_shutdown()
            except Exception:
                pass
        rclpy.shutdown()

if __name__ == "__main__":
    main()
