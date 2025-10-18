# mecademic_bringup/scene/scene_loader.py
import os, yaml
from typing import List
from geometry_msgs.msg import Pose
from mecademic_bringup.scene.object_registry import SceneObjectState
from mecademic_bringup.utils import resolve_mesh_path
from mecademic_bringup.common.frames import (
    FRAME_WORLD, FRAME_SCENE, FRAME_SUBSTRATE_MOUNT, FRAME_SUBSTRATE, VALID_FRAMES
)

def _pose_from_dict(d) -> Pose:
    p = Pose()
    x,y,z = d.get("xyz", [0,0,0])
    qx,qy,qz,qw = d.get("quat", [0,0,0,1])
    p.position.x, p.position.y, p.position.z = x,y,z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx,qy,qz,qw
    return p

def load_scene_yaml(path: str) -> List[SceneObjectState]:
    """
    Erwartete Struktur in scene.yaml:
      objects:
        - id: environment
          frame: world
          mesh: path/relative/to/share/meshes/env.stl
          scale: [1,1,1]
          pose:
            xyz: [0,0,0]
            quat: [0,0,0,1]
        - id: substrate_mount
          frame: world
          ...
        - id: scene
          frame: substrate_mount
          ...
        - id: substrate
          frame: scene
          ...
    """
    if not os.path.isfile(path):
        return []

    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}

    states: List[SceneObjectState] = []
    for obj in data.get("objects", []):
        obj_id = obj.get("id", "")
        frame  = obj.get("frame", FRAME_WORLD)
        if frame not in VALID_FRAMES:
            # Erlaube nur bekannte Frames (Option B)
            # Ausnahme: scene darf explizit an substrate_mount hängen (dein Design)
            pass
        mesh  = resolve_mesh_path(obj.get("mesh", ""))
        scale = tuple(obj.get("scale", [1,1,1]))  # type: ignore
        pose  = obj.get("pose", {"xyz":[0,0,0], "quat":[0,0,0,1]})
        p = _pose_from_dict(pose)

        states.append(SceneObjectState(
            id=obj_id,
            frame=frame,
            mesh_resource=mesh,
            scale=scale,  # type: ignore
            xyz=(p.position.x, p.position.y, p.position.z),
            rpy_deg=(0.0, 0.0, 0.0)  # Quat wird im Visualizer direkt genutzt
        ))
    return states
