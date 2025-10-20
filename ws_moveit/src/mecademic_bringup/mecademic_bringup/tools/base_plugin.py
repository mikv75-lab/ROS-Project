# minimaler Hook-Contract (Basis)
class ToolPluginBase:
    def on_attach(self, node, tool_cfg):
        # Rückgabe False -> ToolManager macht Default-Flow
        node.get_logger().info("🧩 on_attach (default): nichts zu tun")
        return False

    def on_detach(self, node, tool_cfg):
        node.get_logger().info("🧩 on_detach (default): nichts zu tun")
        return False
