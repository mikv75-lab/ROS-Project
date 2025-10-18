import os, yaml
from dataclasses import dataclass
from typing import List

@dataclass
class SceneObjectCfg:
    id: str
    mesh: str
    frame: str
    position: list
    rpy_deg: list
    optional: bool = False

@dataclass
class SceneCfg:
    objects: List[SceneObjectCfg]

def load_scene_yaml(path: str) -> SceneCfg:
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    objs = []
    for o in (data.get("scene_objects") or []):
        objs.append(SceneObjectCfg(
            id=o["id"],
            mesh=o.get("mesh", "") or "",
            frame=o.get("frame", "world"),
            position=o.get("position", [0,0,0]),
            rpy_deg=o.get("rpy_deg", [0,0,0]),
            optional=bool(o.get("optional", False)),
        ))
    return SceneCfg(objects=objs)
