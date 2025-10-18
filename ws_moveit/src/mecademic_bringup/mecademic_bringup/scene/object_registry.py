from dataclasses import dataclass
from typing import Optional, Dict

@dataclass
class SceneObjectState:
    id: str
    frame: str
    mesh: str = ""    # resolved absolute path (or "" for none)

class ObjectRegistry:
    def __init__(self):
        self._objs: Dict[str, SceneObjectState] = {}

    def set(self, obj: SceneObjectState):
        self._objs[obj.id] = obj

    def update_mesh(self, obj_id: str, mesh_abs: str):
        if obj_id not in self._objs:
            return
        self._objs[obj_id].mesh = mesh_abs

    def get(self, obj_id: str) -> Optional[SceneObjectState]:
        return self._objs.get(obj_id)

    def mesh(self, obj_id: str) -> str:
        st = self._objs.get(obj_id)
        return st.mesh if st else ""

    def exists(self, obj_id: str) -> bool:
        return obj_id in self._objs