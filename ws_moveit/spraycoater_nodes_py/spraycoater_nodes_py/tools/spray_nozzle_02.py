#!/usr/bin/env python3
from __future__ import annotations
from mecademic_bringup.tools.base_tool import ToolPluginBase


class ToolPlugin(ToolPluginBase):
    """Plugin für Spray Nozzle 02 – identisch mit Düse 01, andere Offsets."""

    def on_attach(self, node, cfg) -> bool:
        node.get_logger().info("🧩 [spray_nozzle_02] Anhängen der Düse 02")
        return super().on_attach(node, cfg)

    def on_detach(self, node, cfg) -> bool:
        node.get_logger().info("♻️ [spray_nozzle_02] Trenne Düse 02 sauber")
        return super().on_detach(node, cfg)
