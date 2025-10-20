from ..base_plugin import ToolPluginBase

class ToolPlugin(ToolPluginBase):
    def on_attach(self, node, cfg):
        # TCP exakt wie im YAML für "none" setzen
        mount = cfg.get("mount_frame", "tool_mount")
        tcp_xyz = cfg.get("tcp_offset", [0.0, 0.0, 0.0])
        tcp_rpy = cfg.get("tcp_rpy",    [0.0, 0.0, 0.0])

        node.set_tcp(mount, tcp_xyz, tcp_rpy)

        # Mesh entfernen + ACM zurücksetzen (Default-Hilfsfunktionen vorhanden)
        try:
            node.detach_tool_mesh()
        except Exception:
            # Fallback auf direkten Detach Call
            node.detach_tool(mount)

        try:
            node.reset_tool_acm()
        except Exception:
            pass

        node.get_logger().info("🔧 none: attach – TCP gesetzt & Mesh entfernt")
        return True  # handled → kein Default-Flow mehr

    def on_detach(self, node, cfg):
        node.get_logger().info("♻️ none: detach – nichts zu tun")
        return True
