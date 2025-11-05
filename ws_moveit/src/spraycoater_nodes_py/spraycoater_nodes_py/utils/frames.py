# -*- coding: utf-8 -*-
# spraycoater_nodes_py/utils/frames.py
from __future__ import annotations
import os
import yaml
from typing import Dict, Optional


class Frames:
    """
    Kleiner Resolver für Frame-Aliasse aus frames.yaml.
    Erwartet eine *flache* Struktur:
      frames:
        world: "world"
        robot_mount: "robot_mount"
        tcp: "tcp"
        ...
    """
    def __init__(self, mapping: Dict[str, str]):
        self._m = dict(mapping or {})

    def get(self, key: str, default: Optional[str] = None) -> Optional[str]:
        return self._m.get(key, default)

    def resolve(self, name_or_alias: str) -> str:
        """
        Alias -> Frame-ID; falls kein Alias existiert, Eingabe unverändert zurückgeben.
        So bleiben direkte Frame-IDs kompatibel.
        """
        if not name_or_alias:
            raise ValueError("leerer Frame-Name/Alias")
        return self._m.get(name_or_alias, name_or_alias)

    def as_dict(self) -> Dict[str, str]:
        return dict(self._m)


def _validate_mapping(mapping: Dict[str, str], ctx_label: str) -> None:
    if not isinstance(mapping, dict) or not mapping:
        raise ValueError(f"{ctx_label} ist leer/ungültig")

    # Eindeutigkeit der Ziel-Frame-IDs prüfen
    vals = list(mapping.values())
    dups = {v for v in vals if vals.count(v) > 1}
    if dups:
        raise ValueError(f"Doppelte Frame-IDs in {ctx_label}: {sorted(dups)}")

    # Minimal-Set (bei Bedarf erweitern)
    for req in ("world", "robot_mount", "tcp"):
        if req not in mapping:
            raise ValueError(f"{ctx_label} fehlt Schlüssel '{req}'")


def load_frames(yaml_path: str) -> Frames:
    """
    Lädt die *flache* Frames-Map aus frames.yaml (ohne Gruppen).
    """
    if not yaml_path or not os.path.exists(yaml_path):
        raise FileNotFoundError(f"frames.yaml fehlt: {yaml_path}")

    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    frames_root = data.get("frames")
    if not isinstance(frames_root, dict):
        raise ValueError(f"'frames' fehlt/ist kein Mapping in {yaml_path}")

    # Verhindern, dass eine gruppierte Struktur unbemerkt durchrutscht
    if any(isinstance(v, dict) for v in frames_root.values()):
        raise ValueError(
            f"{yaml_path} enthält gruppierte 'frames'. "
            f"Diese Variante erwartet eine flache 'frames'-Map."
        )

    mapping = {str(k): str(v) for k, v in frames_root.items()}
    _validate_mapping(mapping, "frames")

    return Frames(mapping)
