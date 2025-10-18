# mecademic_bringup/scene/object_registry.py
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

@dataclass
class SceneObjectState:
    id: str
    frame: str               # parent frame
    mesh_resource: str = ""  # resolved mesh path (optional)
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)

class ObjectRegistry:
    def __init__(self):
        self._state: Dict[str, SceneObjectState] = {}

    def set(self, state: SceneObjectState):
        self._state[state.id] = state

    def get(self, obj_id: str) -> Optional[SceneObjectState]:
        return self._state.get(obj_id)

    def clear(self, obj_id: str):
        self._state.pop(obj_id, None)

    def items(self):
        return list(self._state.items())
