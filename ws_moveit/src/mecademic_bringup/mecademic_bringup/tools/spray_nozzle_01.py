from . import ToolPluginBase

class ToolPlugin(ToolPluginBase):
    def on_attach(self, node):
        node.get_logger().info("🔧 spray_nozzle_01: attach – bereit")

    def on_detach(self, node):
        node.get_logger().info("♻️ spray_nozzle_01: detach – sauber getrennt")
