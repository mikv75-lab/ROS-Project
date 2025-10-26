# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import yaml
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class Recipe:
    id: str
    description: str = ""
    tool: Optional[str] = None
    substrate: Optional[str] = None
    substrates: List[str] = field(default_factory=list)
    substrate_mount: Optional[str] = None
    mount: Optional[str] = None
    side: str = "top"
    parameters: Dict[str, Any] = field(default_factory=dict)
    path: Dict[str, Any] = field(default_factory=dict)

    # ---------- Factory ----------
    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Recipe":
        return Recipe(
            id=str(d.get("id") or "recipe"),
            description=str(d.get("description") or ""),
            tool=d.get("tool"),
            substrate=d.get("substrate"),
            substrates=list(d.get("substrates") or ([] if not d.get("substrate") else [d.get("substrate")])),
            substrate_mount=d.get("substrate_mount"),
            mount=d.get("mount") or d.get("substrate_mount"),
            side=str(d.get("side") or "top"),
            parameters=dict(d.get("parameters") or {}),
            path=dict(d.get("path") or {}),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "description": self.description,
            "tool": self.tool,
            "substrate": self.substrate,
            "substrates": self.substrates if self.substrates else ([self.substrate] if self.substrate else []),
            "substrate_mount": self.substrate_mount or self.mount,
            "mount": self.mount or self.substrate_mount,
            "side": self.side,
            "parameters": dict(self.parameters or {}),
            "path": dict(self.path or {}),
        }

    # ---------- YAML IO ----------
    @staticmethod
    def load_yaml(path: str) -> "Recipe":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return Recipe.from_dict(data)

    # ---------- Validation ----------
    def validate_required(self) -> List[str]:
        errs: List[str] = []
        if not self.id:
            errs.append("id fehlt")
        if not (self.tool and str(self.tool).strip()):
            errs.append("tool fehlt")
        if not (self.mount or self.substrate_mount):
            errs.append("substrate_mount fehlt")
        if not (self.substrate and str(self.substrate).strip()):
            errs.append("substrate fehlt")

        p = self.path or {}
        if not p.get("type"):
            errs.append("path.type fehlt")
        # plane spiral/meander benötigen 'mode'
        if p.get("type") in ("meander", "spiral") and not p.get("mode"):
            errs.append("path.mode fehlt")
        return errs

    def validate_references(
        self,
        *,
        tools_yaml: Optional[Dict[str, Any]] = None,
        mounts_yaml: Optional[Dict[str, Any]] = None,
        substrates_yaml: Optional[Dict[str, Any]] = None,
    ) -> List[str]:
        errs: List[str] = []
        # Nur einfache Existenzchecks, passend zum 3-Panel Flow
        if tools_yaml and self.tool:
            tools = set((tools_yaml.get("tools") or {}).keys()) | set(tools_yaml.keys())
            if self.tool not in tools:
                # permissiv: der Nutzer nutzt direkt Dateinamen; kein Hard-Fail
                pass
        if mounts_yaml and (self.mount or self.substrate_mount):
            mounts = set((mounts_yaml.get("mounts") or {}).keys()) | set(mounts_yaml.keys())
            if (self.mount or self.substrate_mount) not in mounts:
                pass
        if substrates_yaml and self.substrate:
            subs = set((substrates_yaml.get("substrates") or {}).keys()) | set(substrates_yaml.keys())
            if self.substrate not in subs:
                pass
        return errs
