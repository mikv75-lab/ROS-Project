# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Mapping, Optional

from model.spray_paths.draft import Draft, PoseQuat
from model.spray_paths.trajectory import JTBySegment

def _as_dict(x: Any) -> Dict[str, Any]:
    return x if isinstance(x, dict) else {}

@dataclass
class Recipe:
    """
    Recipe Model (Strict V2).
    Enthält Parameter (Meta) und Attachments (Draft, Trajectories).
    Keine Legacy-Felder mehr.
    """
    # --- Identity ---
    id: str
    description: str = ""
    
    # --- Configuration ---
    tool: Optional[str] = None
    substrate: Optional[str] = None
    substrate_mount: Optional[str] = None
    
    # --- Parameters (dict based) ---
    parameters: Dict[str, Any] = field(default_factory=dict)
    planner: Dict[str, Any] = field(default_factory=dict)
    paths_by_side: Dict[str, Any] = field(default_factory=dict)
    
    # --- Metadata & Runtime Info ---
    meta: Dict[str, Any] = field(default_factory=dict)
    info: Dict[str, Any] = field(default_factory=dict)

    # --- UI State (Non-persistent logic) ---
    valid_save: bool = False
    valid_save_reason: str = "no_preview"
    
    # --- Attachments (Strong Types from Sub-Modules) ---
    draft: Optional[Draft] = None
    
    # Planning / Execution Artifacts
    planned_traj: Optional[JTBySegment] = None
    executed_traj: Optional[JTBySegment] = None
    
    # TCP Geometry (Visuals / Eval) - Uses Draft format
    planned_tcp: Optional[Draft] = None
    executed_tcp: Optional[Draft] = None

    # ----------------------------
    # Properties (UI Sync)
    # ----------------------------
    @property
    def validSave(self) -> bool:
        return self.valid_save
    
    @validSave.setter
    def validSave(self, v: bool) -> None:
        self.valid_save = bool(v)
        self._sync_info()

    @property
    def validSaveReason(self) -> str:
        return self.valid_save_reason

    @validSaveReason.setter
    def validSaveReason(self, v: str) -> None:
        self.valid_save_reason = str(v or "")
        self._sync_info()

    def _sync_info(self) -> None:
        if not self.info: self.info = {}
        self.info["validSave"] = self.valid_save
        self.info["validSaveReason"] = self.valid_save_reason

    # ----------------------------
    # Load / Save (Params only)
    # ----------------------------
    @staticmethod
    def from_params_dict(d: Mapping[str, Any]) -> "Recipe":
        """Strict loading of params.yaml content."""
        d = dict(d or {})
        r = Recipe(
            id=str(d.get("id", "")).strip(),
            description=str(d.get("description", "")),
            tool=d.get("tool"),
            substrate=d.get("substrate"),
            substrate_mount=d.get("substrate_mount"),
            parameters=_as_dict(d.get("parameters")),
            planner=_as_dict(d.get("planner")),
            paths_by_side=_as_dict(d.get("paths_by_side")),
            meta=_as_dict(d.get("meta")),
            info=_as_dict(d.get("info")),
        )
        # Restore UI state from info block
        r.valid_save = bool(r.info.get("validSave", False))
        r.valid_save_reason = str(r.info.get("validSaveReason", "no_preview"))
        return r

    def to_params_dict(self) -> Dict[str, Any]:
        """Strict dumping to params.yaml content."""
        self._sync_info()
        return {
            "id": self.id,
            "description": self.description,
            "tool": self.tool,
            "substrate": self.substrate,
            "substrate_mount": self.substrate_mount,
            "parameters": self.parameters,
            "planner": self.planner,
            "paths_by_side": self.paths_by_side,
            "meta": self.meta,
            "info": self.info,
        }

    # ----------------------------
    # Convenience (Pass-Through)
    # ----------------------------
    def draft_poses_quat(self, side: str) -> List[PoseQuat]:
        if not self.draft: return []
        return self.draft.poses_quat(side)

    def planned_tcp_poses_quat(self, side: str) -> List[PoseQuat]:
        if not self.planned_tcp: return []
        return self.planned_tcp.poses_quat(side)

    def executed_tcp_poses_quat(self, side: str) -> List[PoseQuat]:
        if not self.executed_tcp: return []
        return self.executed_tcp.poses_quat(side)
    
    def compiled_points_mm_for_side(self, side: str) -> List[List[float]]:
        # Helper used by UI SceneManager
        return [[p.x, p.y, p.z] for p in self.draft_poses_quat(side)]
    
    def planned_joint_trajectory(self, *, segment_order=None):
        return self.planned_traj.to_joint_trajectory(segment_order=segment_order) if self.planned_traj else None

    def executed_joint_trajectory(self, *, segment_order=None):
        return self.executed_traj.to_joint_trajectory(segment_order=segment_order) if self.executed_traj else None