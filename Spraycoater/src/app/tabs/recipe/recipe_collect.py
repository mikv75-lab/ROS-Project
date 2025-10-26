# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Any, Dict
from .recipe_types import spec_key_for_page_index

def _val_from_widget(w):
    if w is None:
        return None
    if hasattr(w, "value"):
        return float(w.value())
    if hasattr(w, "isChecked"):
        return bool(w.isChecked())
    if hasattr(w, "currentText"):
        return str(w.currentText())
    if isinstance(w, tuple) and len(w) == 2:
        return [float(w[0].value()), float(w[1].value())]
    return None

def collect_globals(specs: Dict[str, Any], param_widgets: Dict[str, Any]) -> Dict[str, Any]:
    out: Dict[str, Any] = {}
    gspec = specs.get("globals", {})
    for key in gspec.keys():
        w = param_widgets.get(f"globals.{key}")
        val = _val_from_widget(w)
        if val is not None:
            out[key] = val
    return out

def _set_nested(d: Dict[str, Any], dotted: str, value: Any) -> None:
    cur = d
    parts = dotted.split(".")
    for p in parts[:-1]:
        if p not in cur or not isinstance(cur[p], dict):
            cur[p] = {}
        cur = cur[p]
    cur[parts[-1]] = value

def collect_path_for(spec_key: str, specs: Dict[str, Any], param_widgets: Dict[str, Any]) -> Dict[str, Any]:
    path_obj: Dict[str, Any] = {}
    _, ptype, pmode = spec_key.split(".")  # "path", "<type>", "<mode|explicit>"

    if ptype == "explicit":
        path_obj["type"] = "explicit"
    else:
        path_obj["type"] = ptype
        path_obj["mode"] = pmode

    section = specs.get(spec_key, {})
    for dotted_key in section.keys():
        w = param_widgets.get(f"path.{dotted_key}")
        val = _val_from_widget(w)
        if val is not None:
            _set_nested(path_obj, dotted_key, val)
    return path_obj

def build_recipe_from_forms(src_recipe: Dict[str, Any],
                            page_index: int,
                            specs: Dict[str, Any],
                            param_widgets: Dict[str, Any]) -> Dict[str, Any]:
    """Erzeugt ein neues Rezept-Dict basierend auf UI-Werten + Metadaten aus src_recipe."""
    return {
        "id":          src_recipe.get("id"),
        "description": src_recipe.get("description"),
        "tool":        src_recipe.get("tool"),
        "substrates":  src_recipe.get("substrates", []),
        "mount":       src_recipe.get("mount"),
        "side":        src_recipe.get("side"),
        "parameters":  collect_globals(specs, param_widgets),
        "path":        collect_path_for(spec_key_for_page_index(page_index), specs, param_widgets),
    }
