# -*- coding: utf-8 -*-
from __future__ import annotations

import yaml
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Iterable, Tuple

@dataclass
class Recipe:
    id: str
    description: str = ""
    tool: Optional[str] = None
    substrate: Optional[str] = None
    substrates: List[str] = field(default_factory=list)
    substrate_mount: Optional[str] = None
    mount: Optional[str] = None
    parameters: Dict[str, Any] = field(default_factory=dict)
    paths_by_side: Dict[str, Dict[str, Any]] = field(default_factory=dict)

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
            parameters=dict(d.get("parameters") or {}),
            paths_by_side=dict(d.get("paths_by_side") or {}),
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
            "parameters": dict(self.parameters or {}),
            "paths_by_side": {str(s): dict(p or {}) for s, p in (self.paths_by_side or {}).items()},
        }

    @staticmethod
    def load_yaml(path: str) -> "Recipe":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return Recipe.from_dict(data)

    def validate_required(self) -> List[str]:
        errs: List[str] = []
        if not self.id: errs.append("id fehlt")
        if not (self.mount or self.substrate_mount): errs.append("substrate_mount/mount fehlt")
        if not (self.substrate or self.substrates): errs.append("substrate fehlt")
        if not self.paths_by_side: errs.append("paths_by_side fehlt oder leer")
        for side, path in (self.paths_by_side or {}).items():
            if not isinstance(path, dict) or not str(path.get("type") or "").strip():
                errs.append(f'paths_by_side["{side}"].type fehlt')
        return errs

    def iter_paths(self) -> Iterable[Tuple[str, Dict[str, Any]]]:
        for s, p in (self.paths_by_side or {}).items():
            yield s, p
