from ..base_plugin import ToolPluginBase

class ToolPlugin(ToolPluginBase):
    def on_attach(self, node, cfg):
        mount = cfg.get("mount_frame", "tool_mount")
        tcp_xyz = cfg.get("tcp_offset", [0.0, 0.0, 0.0])
        tcp_rpy = cfg.get("tcp_rpy",    [0.0, 0.0, 0.0])
        mesh_uri = cfg.get("mesh", "")
        mesh_off = cfg.get("mesh_offset", [0.0, 0.0, 0.0])
        mesh_rpy = cfg.get("mesh_rpy",    [0.0, 0.0, 0.0])

        # TCP setzen
        node.set_tcp(mount, tcp_xyz, tcp_rpy)

        # Mesh (falls vorhanden) anhängen + ACM-Defaults
        if mesh_uri:
            mesh = node.load_mesh(node.resolve_mesh_path(mesh_uri))
            aco = node.make_attach_msg(mount, mesh, mesh_off, mesh_rpy)
            node.attached_pub.publish(aco)
            node.get_logger().info("🧩 spray_nozzle_01: Mesh attached")
            node.set_tool_acm_defaults()

        node.get_logger().info("🔧 spray_nozzle_01: attach – bereit")
        return True

    def on_detach(self, node, cfg):
        # Mesh loslösen + ACM reset (falls du hier was tun willst)
        try:
            node.detach_tool_mesh()
        except Exception:
            pass
        try:
            node.reset_tool_acm()
        except Exception:
            pass
        node.get_logger().info("♻️ spray_nozzle_01: detach – sauber getrennt")
        return True
