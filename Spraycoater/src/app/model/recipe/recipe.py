# -*- coding: utf-8 -*-
from __future__ import annotations

import yaml
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Iterable, Tuple


@dataclass
class Recipe:
    """Kompaktes Rezeptmodell für den Spraycoater.

    - id:            Rezeptname
    - description:   freie Details/Beschreibung
    - tool:          gewählte Düse/Tool-ID
    - substrate(s):  ausgewähltes Substrat (und optional Liste)
    - substrate_mount: gewählter Mount
    - parameters:    globale Parameter (flow, speed, ...)
    - paths_by_side: pro Side GENAU EIN Path-Dict (z.B. {"type":"meander_plane", ...})
    """
    # Meta
    id: str                              # = Rezeptname
    description: str = ""                # = optionale Details

    # Auswahl
    tool: Optional[str] = None
    substrate: Optional[str] = None
    substrates: List[str] = field(default_factory=list)
    substrate_mount: Optional[str] = None

    # Globale Parameter (für Spray-Tool etc.)
    parameters: Dict[str, Any] = field(default_factory=dict)

    # Kern: pro Side genau EIN Path-Objekt
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

        parameters = dict(d.get("parameters") or {})
        paths_by_side = dict(d.get("paths_by_side") or {})

        return Recipe(
            id=rid,
            description=desc,
            tool=tool,
            substrate=substrate,
            substrates=substrates,
            substrate_mount=substrate_mount,
            parameters=parameters,
            paths_by_side=paths_by_side,
        )

    @staticmethod
    def with_default_params(*, name: str = "recipe", description: str = "", params: Optional[Dict[str, Any]] = None) -> "Recipe":
        """Erzeugt ein leeres Rezept mit übergebenen Default-Parametern (nur Globals)."""
        return Recipe(
            id=name,
            description=description,
            tool=None,
            substrate=None,
            substrates=[],
            substrate_mount=None,
            parameters=dict(params or {}),
            paths_by_side={},
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "description": self.description,
            "tool": self.tool,
            "substrate": self.substrate,
            "substrates": self.substrates if self.substrates else ([self.substrate] if self.substrate else []),
            "substrate_mount": self.substrate_mount,
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
        if not (self.substrate or self.substrates):
            errs.append("substrate fehlt")
        if not self.substrate_mount:
            errs.append("substrate_mount fehlt")
        if not self.paths_by_side:
            errs.append("paths_by_side fehlt oder leer")
        for side, path in (self.paths_by_side or {}).items():
            if not isinstance(path, dict) or not str(path.get("type") or "").strip():
                errs.append(f'paths_by_side["{side}"].type fehlt')
        return errs

    # ---------- Helpers ----------
    def iter_paths(self) -> Iterable[Tuple[str, Dict[str, Any]]]:
        """Ergibt (side, path)-Paare in Einfügereihenfolge."""
        for s, p in (self.paths_by_side or {}).items():
            yield s, p

    def get_path_for_side(self, side: str) -> Optional[Dict[str, Any]]:
        """Gibt das Path-Dict für eine Side zurück (oder None)."""
        if not side:
            return None
        return (self.paths_by_side or {}).get(str(side))

    def to_preview_payload(self, sides: Optional[List[str]] = None) -> Dict[str, Any]:
        """Erzeugt ein Payload-Dict für die Preview. Optional nur ausgewählte Sides exportieren."""
        if sides:
            side_set = {str(s) for s in sides}
            pbs = {s: dict(p or {}) for s, p in (self.paths_by_side or {}).items() if s in side_set}
        else:
            pbs = {s: dict(p or {}) for s, p in (self.paths_by_side or {}).items()}

        return {
            "id": self.id,
            "description": self.description,
            "tool": self.tool,
            "substrate": self.substrate,
            "substrate_mount": self.substrate_mount,
            "parameters": dict(self.parameters or {}),
            "paths_by_side": pbs,
        }
