#!/usr/bin/env python3
from __future__ import annotations
import os
import yaml
from dataclasses import dataclass
from typing import Dict, Any, Tuple, Optional
from pathlib import Path

# Diese Datei liegt in: app/source/app/startup.py
# -> APP_ROOT = .../app
APP_ROOT = Path(__file__).resolve().parents[2]   # .../app

# Es gibt genau EINE Config (kein Fallback-Scan)
CONFIG_PATH = APP_ROOT / "resource" / "configs" / "startup.yaml"

def _vec3(v: Any, keypath: str) -> Tuple[float, float, float]:
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise ValueError(f"{keypath} muss Liste/Tuple mit 3 Zahlen sein")
    try:
        x, y, z = float(v[0]), float(v[1]), float(v[2])
        return (x, y, z)
    except Exception:
        raise ValueError(f"{keypath} enthält nicht-konvertierbare Werte")

def _bool(v: Any, keypath: str) -> bool:
    if isinstance(v, bool):
        return v
    raise ValueError(f"{keypath} muss bool sein")

def _opt_str(v: Any) -> Optional[str]:
    return v if isinstance(v, str) and v else None

def _resolve_path(p: str) -> str:
    p = str(p)
    # relativ zur APP_ROOT auflösen (damit Dinge wie resource/stl/... funktionieren)
    return p if os.path.isabs(p) else str((APP_ROOT / p).resolve())

def load_startup_cfg(path: Optional[str] = None) -> Dict[str, Any]:
    used_path = Path(path) if path else CONFIG_PATH
    if not used_path.exists():
        raise FileNotFoundError(f"startup.yaml nicht gefunden: {used_path}")
    print(f"[startup] using config: {used_path}", flush=True)

    with open(used_path, "r") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError("startup.yaml: Top-Level muss Mapping sein")

    launch = _bool(data.get("launch"), "launch")

    wtm = data.get("world_to_meca_mount")
    if not isinstance(wtm, dict):
        raise ValueError("world_to_meca_mount fehlt oder ist kein Mapping")
    wtm_xyz = _vec3(wtm.get("xyz"), "world_to_meca_mount.xyz")
    wtm_rpy = _vec3(wtm.get("rpy_deg"), "world_to_meca_mount.rpy_deg")

    wm = data.get("workspace_mount")
    if not isinstance(wm, dict):
        raise ValueError("workspace_mount fehlt oder ist kein Mapping")
    wm_enable = _bool(wm.get("enable"), "workspace_mount.enable")
    wm_mesh_raw = _opt_str(wm.get("mesh_path"))
    wm_mesh = _resolve_path(wm_mesh_raw) if wm_mesh_raw else None
    wm_xyz = _vec3(wm.get("xyz"), "workspace_mount.xyz")
    wm_rpy = _vec3(wm.get("rpy_deg"), "workspace_mount.rpy_deg")

    cg = data.get("cage")
    if not isinstance(cg, dict):
        raise ValueError("cage fehlt oder ist kein Mapping")
    cg_enable = _bool(cg.get("enable"), "cage.enable")
    cg_mesh_raw = _opt_str(cg.get("mesh_path"))
    cg_mesh = _resolve_path(cg_mesh_raw) if cg_mesh_raw else None
    cg_xyz = _vec3(cg.get("xyz"), "cage.xyz")
    cg_rpy = _vec3(cg.get("rpy_deg"), "cage.rpy_deg")

    return {
        "launch": launch,
        "world_to_meca_mount": {"xyz": wtm_xyz, "rpy_deg": wtm_rpy},
        "workspace_mount": {
            "enable": wm_enable,
            "mesh_path": wm_mesh,
            "xyz": wm_xyz,
            "rpy_deg": wm_rpy,
        },
        "cage": {
            "enable": cg_enable,
            "mesh_path": cg_mesh,
            "xyz": cg_xyz,
            "rpy_deg": cg_rpy,
        },
        "_used_path": str(used_path),
    }

def launch_args_from_cfg(cfg: Dict[str, Any]) -> list[str]:
    def fmt3(t): return f"{t[0]} {t[1]} {t[2]}"
    wtm = cfg["world_to_meca_mount"]
    wm  = cfg["workspace_mount"]
    cg  = cfg["cage"]

    args: list[str] = [
        f'world_to_meca_xyz:="{fmt3(wtm["xyz"])}"',
        f'world_to_meca_rpy_deg:="{fmt3(wtm["rpy_deg"])}"',
        f'workspace_mount_enable:={"true" if wm["enable"] else "false"}',
        f'workspace_mount_xyz:="{fmt3(wm["xyz"])}"',
        f'workspace_mount_rpy_deg:="{fmt3(wm["rpy_deg"])}"',
        f'cage_enable:={"true" if cg["enable"] else "false"}',
        f'cage_xyz:="{fmt3(cg["xyz"])}"',
        f'cage_rpy_deg:="{fmt3(cg["rpy_deg"])}"',
    ]
    if wm.get("mesh_path"):
        args.append(f'workspace_mount_mesh:="{wm["mesh_path"]}"')
    if cg.get("mesh_path"):
        args.append(f'cage_mesh:="{cg["mesh_path"]}"')
    return args

@dataclass
class LaunchConfig:
    launch: bool
    cfg: Dict[str, Any]

    @classmethod
    def load(cls, path: Optional[str] = None) -> "LaunchConfig":
        cfg = load_startup_cfg(path)
        return cls(launch=bool(cfg["launch"]), cfg=cfg)

    def as_launch_args(self) -> list[str]:
        return launch_args_from_cfg(self.cfg)

    @property
    def used_path(self) -> str:
        return str(self.cfg.get("_used_path", "")) or "<unknown>"
