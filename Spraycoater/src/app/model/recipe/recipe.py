# -*- coding: utf-8 -*-
from __future__ import annotations

import yaml
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Iterable, Tuple


@dataclass
class Recipe:
    # Meta
    id: str
    description: str = ""

    # Auswahl
    tool: Optional[str] = None
    substrate: Optional[str] = None
    substrates: List[str] = field(default_factory=list)
    substrate_mount: Optional[str] = None
    mount: Optional[str] = None

    # Globale Parameter (für Spray-Tool etc.)
    parameters: Dict[str, Any] = field(default_factory=dict)

    # Kern: pro Side genau EIN Path-Objekt (z.B. {"type":"spiral_plane", ...})
    paths_by_side: Dict[str, Dict[str, Any]] = field(default_factory=dict)

    # ---------- Factory ----------
    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Recipe":
        rid = str(d.get("id") or "recipe")
        desc = str(d.get("description") or "")

        tool = d.get("tool")
        substrate = d.get("substrate")
        substrates = list(d.get("substrates") or ([] if not substrate else [substrate]))

        substrate_mount = d.get("substrate_mount")
        mount = d.get("mount") or substrate_mount

        parameters = dict(d.get("parameters") or {})
        paths_by_side = dict(d.get("paths_by_side") or {})

        return Recipe(
            id=rid,
            description=desc,
            tool=tool,
            substrate=substrate,
            substrates=substrates,
            substrate_mount=substrate_mount,
            mount=mount,
            parameters=parameters,
            paths_by_side=paths_by_side,
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
        if not (self.mount or self.substrate_mount):
            errs.append("substrate_mount/mount fehlt")
        if not (self.substrate or self.substrates):
            errs.append("substrate fehlt")
        if not self.paths_by_side:
            errs.append("paths_by_side fehlt oder leer")
        for side, path in (self.paths_by_side or {}).items():
            if not isinstance(path, dict) or not str(path.get("type") or "").strip():
                errs.append(f'paths_by_side["{side}"].type fehlt')
        return errs

    def validate_references(
        self,
        *,
        tools_yaml: Optional[Dict[str, Any]] = None,
        mounts_yaml: Optional[Dict[str, Any]] = None,
        substrates_yaml: Optional[Dict[str, Any]] = None,
    ) -> List[str]:
        errs: List[str] = []
        # optionale, permissive Checks
        if tools_yaml and self.tool:
            tools = set((tools_yaml.get("tools") or {}).keys()) | set(tools_yaml.keys())
            if self.tool not in tools:
                pass
        if mounts_yaml and (self.mount or self.substrate_mount):
            mounts = set((mounts_yaml.get("mounts") or {}).keys()) | set(mounts_yaml.keys())
            if (self.mount or self.substrate_mount) not in mounts:
                pass
        if substrates_yaml and (self.substrate or self.substrates):
            subs = set((substrates_yaml.get("substrates") or {}).keys()) | set(substrates_yaml.keys())
            for s in ([self.substrate] if self.substrate else []) + list(self.substrates or []):
                if s and s not in subs:
                    pass
        return errs

    # ---------- Helpers ----------
    def iter_paths(self) -> Iterable[Tuple[str, Dict[str, Any]]]:
        """Ergibt (side, path)-Paare in Einfügereihenfolge."""
        for s, p in (self.paths_by_side or {}).items():
            yield s, p
