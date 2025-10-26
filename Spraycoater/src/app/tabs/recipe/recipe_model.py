# Spraycoater/src/app/tabs/recipe/recipe_model.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass, field, asdict
from typing import Any, Dict, Optional, List
import os, yaml, logging

_LOG = logging.getLogger("app.tabs.recipe.model")

@dataclass
class Recipe:
    id: str
    description: str = ""
    tool: str = ""          # Pflicht
    mount: str = ""         # Pflicht (substrate_mount key)
    substrate: str = ""     # Pflicht (substrate key)
    parameters: Dict[str, Any] = field(default_factory=dict)
    path: Dict[str, Any] = field(default_factory=dict)
    valid: bool = False     # wird von validate_* gesetzt

    # ---------- IO ----------
    @staticmethod
    def load_yaml(file_path: str) -> "Recipe":
        fp = os.path.abspath(os.path.normpath(file_path))
        if not os.path.exists(fp):
            raise FileNotFoundError(fp)
        with open(fp, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f) or {}
        return Recipe.from_dict(d)

    def save_yaml(self, dir_path: str, filename: Optional[str] = None) -> str:
        if not dir_path:
            raise ValueError("save_yaml: dir_path fehlt.")
        target_dir = os.path.abspath(os.path.normpath(dir_path))
        if not os.path.isdir(target_dir):
            raise FileNotFoundError(f"save_yaml: Verzeichnis existiert nicht: {target_dir}")
        fname = filename or f"{self.id or 'recipe'}.yaml"
        target = os.path.join(target_dir, fname)
        with open(target, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, allow_unicode=True, sort_keys=False)
        _LOG.info("Recipe gespeichert: %s", target)
        return target

    # ---------- Mapping ----------
    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Recipe":
        return Recipe(
            id=str(d.get("id", "")),
            description=str(d.get("description", "")),
            tool=str(d.get("tool", "")),
            mount=str(d.get("mount", "")),
            substrate=str(d.get("substrate", "") or (d.get("substrates") or [None])[0] or ""),
            parameters=dict(d.get("parameters", {}) or {}),
            path=dict(d.get("path", {}) or {}),
            valid=bool(d.get("valid", False)),
        )

    def to_dict(self) -> Dict[str, Any]:
        out = asdict(self)
        # kompat: altes Feld 'substrates' (einzelnes Element)
        out["substrates"] = [self.substrate] if self.substrate else []
        return out

    # ---------- Convenience ----------
    def getPath(self):
        """Bevorzugte Punkte-Liste (mm) aus path zurückgeben."""
        return (self.path.get("points_mm")
                or self.path.get("polyline_mm")
                or self.path.get("surface_points_mm")
                or [])

    # ---------- Validation ----------
    def validate_required(self) -> List[str]:
        errs: List[str] = []
        if not self.id:        errs.append("id fehlt")
        if not self.tool:      errs.append("tool fehlt")
        if not self.mount:     errs.append("mount fehlt")
        if not self.substrate: errs.append("substrate fehlt")
        ptype = (self.path.get("type") or "").strip().lower()
        if not ptype:
            errs.append("path.type fehlt")
        self.valid = (len(errs) == 0)
        return errs

    def validate_references(
        self,
        *,
        tools_yaml: Dict[str, Any] | None,
        mounts_yaml: Dict[str, Any] | None,
        substrates_yaml: Dict[str, Any] | None,
    ) -> List[str]:
        errs: List[str] = []
        if tools_yaml is not None:
            tools = (tools_yaml.get("tools") or {})
            if self.tool not in tools:
                errs.append(f"Tool '{self.tool}' nicht in tools.yaml")
        if mounts_yaml is not None:
            mounts = (mounts_yaml.get("mounts") or {})
            if self.mount not in mounts:
                errs.append(f"Mount '{self.mount}' nicht in substrate_mounts.yaml")
        if substrates_yaml is not None:
            subs = (substrates_yaml.get("substrates") or {})
            if self.substrate not in subs:
                errs.append(f"Substrat '{self.substrate}' nicht in substrates.yaml")
        self.valid = (len(errs) == 0 and self.valid)
        return errs
