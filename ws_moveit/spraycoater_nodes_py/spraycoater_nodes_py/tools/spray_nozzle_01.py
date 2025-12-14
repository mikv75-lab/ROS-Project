#!/usr/bin/env python3
from __future__ import annotations
from mecademic_bringup.tools.base_tool import ToolPluginBase


class ToolPlugin(ToolPluginBase):
    """Plugin für Spray Nozzle 01 – setzt Offset & Mesh laut YAML."""

    def on_attach(self, node, cfg) -> bool:
        node.get_logger().info("🧩 [spray_nozzle_01] Anhängen der Düse 01")
        return super().on_attach(node, cfg)

    def on_detach(self, node, cfg) -> bool:
        node.get_logger().info("♻️ [spray_nozzle_01] Trenne Düse 01 sauber")
        return super().on_detach(node, cfg)
