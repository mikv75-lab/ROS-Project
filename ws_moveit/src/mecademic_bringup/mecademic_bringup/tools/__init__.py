# minimaler Hook-Contract
class ToolPluginBase:
    def on_attach(self, node):
        node.get_logger().info("🧩 on_attach (default): nichts zu tun")

    def on_detach(self, node):
        node.get_logger().info("🧩 on_detach (default): nichts zu tun")
