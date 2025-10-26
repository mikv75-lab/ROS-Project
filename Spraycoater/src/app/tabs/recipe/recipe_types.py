# -*- coding: utf-8 -*-
from __future__ import annotations

def page_index_for(rtype: str, rmode: str) -> int:
    if rtype == "meander" and rmode == "plane":     return 0
    if rtype == "spiral"  and rmode == "plane":     return 1
    if rtype == "spiral"  and rmode == "cylinder":  return 2
    if rtype == "explicit":                         return 3
    return 0

def type_label_for(rtype: str, rmode: str) -> str:
    if rtype == "meander" and rmode == "plane":     return "Meander (plane)"
    if rtype == "spiral"  and rmode == "plane":     return "Spiral (plane)"
    if rtype == "spiral"  and rmode == "cylinder":  return "Spiral (cylinder)"
    if rtype == "explicit":                         return "Explicit"
    return "Meander (plane)"

def spec_key_for_page_index(idx: int) -> str:
    mapping = [
        "path.meander.plane",
        "path.spiral.plane",
        "path.spiral.cylinder",
        "path.explicit",
    ]
    if idx < 0 or idx >= len(mapping):
        idx = 0
    return mapping[idx]

def visuals_preset_for(rtype: str, rmode: str) -> tuple[str, str]:
    """(shape_text, path_text) für die Preview-Presets."""
    shape = "Zylinder" if rmode == "cylinder" else "Würfel"
    path  = "Raster" if rtype == "meander" else "Spiral"
    return shape, path
