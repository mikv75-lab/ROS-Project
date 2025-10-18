# mecademic_bringup/scene/scene_manager_node.py
from __future__ import annotations
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import String, Empty
from tf2_ros import StaticTransformBroadcaster

from mecademic_bringup.common.frames import (
    FRAME_WORLD, FRAME_SUBSTRATE_MOUNT, FRAME_SCENE, FRAME_SUBSTRATE, VALID_FRAMES
)
from mecademic_bringup.common.topics import (
    TOPIC_SCENE_MOUNT_SET, TOPIC_SCENE_MOUNT_CLEAR, TOPIC_SCENE_MOUNT_INFO,
    TOPIC_SCENE_SUBSTRATE_SET, TOPIC_SCENE_SUBSTRATE_CLEAR, TOPIC_SCENE_SUBSTRATE_INFO,
    TOPIC_SCENE_ENV_SET, TOPIC_SCENE_ENV_CLEAR, TOPIC_SCENE_ENV_INFO,
    TOPIC_SCENE_SPRAY_SET, TOPIC_SCENE_SPRAY_CLEAR,
)
from mecademic_bringup.common.params import PARAM_SCENE_CONFIG, PARAM_SCENE_PARENT_FRAME
from mecademic_bringup.scene.object_registry import ObjectRegistry, SceneObjectState
from mecademic_bringup.scene.mesh_manager import MeshManager
from mecademic_bringup.scene.scene_loader import load_scene_yaml
from mecademic_bringup.utils import rpy_deg_to_quat, resolve_mesh_path

class SceneManagerNode(Node):
    def __init__(self):
        super().__init__("scene_manager")

        # Params
        self.declare_parameter(PARAM_SCENE_CONFIG, "")
        self.declare_parameter(PARAM_SCENE_PARENT_FRAME, FRAME_SUBSTRATE_MOUNT)  # scene hängt IMMER an substrate_mount
        self._scene_yaml = self.get_parameter(PARAM_SCENE_CONFIG).get_parameter_value().string_value
        self._scene_parent = self.get_parameter(PARAM_SCENE_PARENT_FRAME).get_parameter_value().string_value

        # Infra
        self._tf_static = StaticTransformBroadcaster(self)
        self._registry = ObjectRegistry()
        self._meshes = MeshManager(self)

        # Scene Topics (deine originalen Namen)
        self.create_subscription(String, TOPIC_SCENE_MOUNT_SET,   self._on_mount_set,   10)
        self.create_subscription(Empty,  TOPIC_SCENE_MOUNT_CLEAR, self._on_mount_clear, 10)
        self.create_subscription(Empty,  TOPIC_SCENE_MOUNT_INFO,  self._on_mount_info,  10)

        self.create_subscription(String, TOPIC_SCENE_SUBSTRATE_SET,   self._on_substrate_set,   10)
        self.create_subscription(Empty,  TOPIC_SCENE_SUBSTRATE_CLEAR, self._on_substrate_clear, 10)
        self.create_subscription(Empty,  TOPIC_SCENE_SUBSTRATE_INFO,  self._on_substrate_info,  10)

        self.create_subscription(String, TOPIC_SCENE_ENV_SET,   self._on_env_set,   10)
        self.create_subscription(Empty,  TOPIC_SCENE_ENV_CLEAR, self._on_env_clear, 10)
        self.create_subscription(Empty,  TOPIC_SCENE_ENV_INFO,  self._on_env_info,  10)

        # Spraypath (nur Clear/Set-Trigger – eigentliche Marker macht SprayPathManager)
        self.create_subscription(Empty,  TOPIC_SCENE_SPRAY_CLEAR, lambda _: self.get_logger().info("[scene] spraypath clear request"), 10)
        self.create_subscription(String, TOPIC_SCENE_SPRAY_SET,   lambda msg: self.get_logger().info(f"[scene] spraypath set: {msg.data}"), 10)

        # Statische TF-Kette (immer!):
        # world -> substrate_mount
        self._publish_static_tf(FRAME_WORLD, FRAME_SUBSTRATE_MOUNT, (0,0,0), (0,0,0))
        # substrate_mount -> scene  (ehem. workspace_center)
        self._publish_static_tf(FRAME_SUBSTRATE_MOUNT, FRAME_SCENE, (0,0,0), (0,0,0))
        # scene -> substrate
        self._publish_static_tf(FRAME_SCENE, FRAME_SUBSTRATE, (0,0,0), (0,0,0))

        # Szene aus YAML laden (optional)
        if self._scene_yaml:
            path = resolve_mesh_path(self._scene_yaml)
            for state in load_scene_yaml(path):
                self._registry.set(state)
                self._try_visualize(state)

        self.get_logger().info(f"[scene_manager] ✅ gestartet. parent(scene)='{self._scene_parent}', yaml='{self._scene_yaml or '-'}'")

    # --- Helper TF ---
    def _publish_static_tf(self, parent: str, child: str, xyz, rpy_deg):
        if parent not in VALID_FRAMES:
            self.get_logger().warn(f"[scene_manager] parent frame '{parent}' nicht in VALID_FRAMES")
        if child not in VALID_FRAMES:
            self.get_logger().warn(f"[scene_manager] child frame '{child}' nicht in VALID_FRAMES")

        qx,qy,qz,qw = rpy_deg_to_quat(*rpy_deg)
        tf = TransformStamped()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = qx,qy,qz,qw
        self._tf_static.sendTransform([tf])

    # --- Mesh visualize/update ---
    def _try_visualize(self, state: SceneObjectState):
        p = Pose()
        p.position.x, p.position.y, p.position.z = state.xyz
        # Orientierung kommt ggf. aus Mesh selbst/ist neutral – RPY ist hier 0
        self._meshes.publish_mesh(
            obj_id=state.id,
            frame_id=state.frame,
            mesh_resource=state.mesh_resource,
            pose=p,
            scale=state.scale,
            color=(0.7, 0.7, 0.7, 1.0),
        )

    # --- Topic handlers (Mount) ---
    def _on_mount_set(self, msg: String):
        # payload: mesh_path [optional scale_x scale_y scale_z]
        parts = msg.data.split()
        mesh = resolve_mesh_path(parts[0]) if parts else ""
        scale = tuple(map(float, parts[1:4])) if len(parts) >= 4 else (1.0, 1.0, 1.0)
        s = SceneObjectState(id="substrate_mount", frame=FRAME_WORLD, mesh_resource=mesh, scale=scale)
        self._registry.set(s); self._try_visualize(s)
        self.get_logger().info(f"[scene] substrate_mount set: mesh='{mesh}', scale={scale}")

    def _on_mount_clear(self, _):
        self._registry.clear("substrate_mount")
        from mecademic_bringup.scene.mesh_manager import MeshManager as _MM  # only for id calc
        self._meshes.delete_mesh("substrate_mount", FRAME_WORLD)
        self.get_logger().info("[scene] substrate_mount cleared")

    def _on_mount_info(self, _):
        s = self._registry.get("substrate_mount")
        self.get_logger().info(f"[scene] substrate_mount: {s}")

    # --- Topic handlers (Substrate) ---
    def _on_substrate_set(self, msg: String):
        # payload: mesh_path [optional scale_x scale_y scale_z]
        parts = msg.data.split()
        mesh = resolve_mesh_path(parts[0]) if parts else ""
        scale = tuple(map(float, parts[1:4])) if len(parts) >= 4 else (1.0, 1.0, 1.0)
        s = SceneObjectState(id="substrate", frame=FRAME_SCENE, mesh_resource=mesh, scale=scale)
        self._registry.set(s); self._try_visualize(s)
        self.get_logger().info(f"[scene] substrate set: mesh='{mesh}', scale={scale}")

    def _on_substrate_clear(self, _):
        self._registry.clear("substrate")
        self._meshes.delete_mesh("substrate", FRAME_SCENE)
        self.get_logger().info("[scene] substrate cleared")

    def _on_substrate_info(self, _):
        s = self._registry.get("substrate")
        self.get_logger().info(f"[scene] substrate: {s}")

    # --- Topic handlers (Environment) ---
    def _on_env_set(self, msg: String):
        parts = msg.data.split()
        mesh = resolve_mesh_path(parts[0]) if parts else ""
        scale = tuple(map(float, parts[1:4])) if len(parts) >= 4 else (1.0, 1.0, 1.0)
        s = SceneObjectState(id="environment", frame=FRAME_WORLD, mesh_resource=mesh, scale=scale)
        self._registry.set(s); self._try_visualize(s)
        self.get_logger().info(f"[scene] environment set: mesh='{mesh}', scale={scale}")

    def _on_env_clear(self, _):
        self._registry.clear("environment")
        self._meshes.delete_mesh("environment", FRAME_WORLD)
        self.get_logger().info("[scene] environment cleared")

    def _on_env_info(self, _):
        s = self._registry.get("environment")
        self.get_logger().info(f"[scene] environment: {s}")


def main():
    rclpy.init()
    node = SceneManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
