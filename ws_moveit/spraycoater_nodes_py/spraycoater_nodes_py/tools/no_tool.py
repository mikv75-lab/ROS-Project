#!/usr/bin/env python3
from __future__ import annotations
from mecademic_bringup.tools.base_tool import ToolPluginBase


class ToolPlugin(ToolPluginBase):
    """Plugin für 'no_tool' – keine Geometrie, kein Mesh, nur TCP reset."""

    def on_attach(self, node, cfg) -> bool:
        node.get_logger().info("🧩 [no_tool] Kein Werkzeug montiert – TCP auf 0 zurücksetzen")
        return super().on_attach(node, cfg)

    def on_detach(self, node, cfg) -> bool:
        node.get_logger().info("♻️ [no_tool] Bereits leer – keine Aktion erforderlich")
        return super().on_detach(node, cfg)
