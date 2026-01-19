# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab3d/mesh_utils.py
from __future__ import annotations

import os
from typing import Any, Dict, Iterable, List, Optional, Tuple, cast

import numpy as np
import pyvista as pv

# We reuse the strict path resolvers from startup (SSoT).
# This is intentional: package:// URIs must resolve exactly as in the rest of the app.
from config.startup import resolve_path, resolve_package_uri  # strict helpers (no silent fallbacks)

Bounds6 = Tuple[float, float, float, float, float, float]


# ============================================================
# Bounds Utilities
# Bounds tuple format: (xmin, xmax, ymin, ymax, zmin, zmax)
# ============================================================

def _as_bounds6(b: object) -> Optional[Bounds6]:
    """
    Normalize an input to a 6-float bounds tuple.
    Accepts:
      - pyvista mesh .bounds (sequence of 6)
      - tuple/list of 6 numbers
    Returns None if invalid.
    """
    try:
        if b is None:
            return None
        if isinstance(b, (tuple, list)) and len(b) == 6:
            vals = [float(x) for x in b]
            if any(not np.isfinite(v) for v in vals):
                return None
            return (vals[0], vals[1], vals[2], vals[3], vals[4], vals[5])
    except Exception:
        return None
    return None


def _bounds_of_poly(mesh: Optional[pv.PolyData]) -> Optional[Bounds6]:
    """
    Get bounds from a pyvista PolyData (or any pv dataset that has .bounds).
    Returns None if mesh is None or bounds invalid.
    """
    if mesh is None:
        return None
    try:
        b = getattr(mesh, "bounds", None)
        return _as_bounds6(b)
    except Exception:
        return None


def _union_bounds(bounds_list: object) -> Bounds6:
    """
    Union of multiple bounds.
    bounds_list: iterable of bounds-like items (tuple/list len 6, or pv mesh with .bounds).
    Returns a valid 6-tuple. If nothing usable is provided, returns a conservative default.

    NOTE: SceneManager expects this name to exist.
    """
    # conservative fallback (matches your preview defaults)
    fallback: Bounds6 = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

    if bounds_list is None:
        return fallback

    xs_min: List[float] = []
    xs_max: List[float] = []
    ys_min: List[float] = []
    ys_max: List[float] = []
    zs_min: List[float] = []
    zs_max: List[float] = []

    try:
        for item in bounds_list:
            b: Optional[Bounds6] = None

            # item might be a mesh-like with .bounds
            try:
                if hasattr(item, "bounds"):
                    b = _as_bounds6(getattr(item, "bounds", None))
            except Exception:
                b = None

            # or item might be a raw 6-tuple/list
            if b is None:
                b = _as_bounds6(item)

            if b is None:
                continue

            xmin, xmax, ymin, ymax, zmin, zmax = b

            # handle inverted/degenerate bounds defensively
            if xmax < xmin:
                xmin, xmax = xmax, xmin
            if ymax < ymin:
                ymin, ymax = ymax, ymin
            if zmax < zmin:
                zmin, zmax = zmax, zmin

            xs_min.append(xmin)
            xs_max.append(xmax)
            ys_min.append(ymin)
            ys_max.append(ymax)
            zs_min.append(zmin)
            zs_max.append(zmax)

    except Exception:
        # if iteration fails for any reason, still return fallback
        return fallback

    if not xs_min:
        return fallback

    out: Bounds6 = (
        float(min(xs_min)),
        float(max(xs_max)),
        float(min(ys_min)),
        float(max(ys_max)),
        float(min(zs_min)),
        float(max(zs_max)),
    )

    # final sanity check
    if any(not np.isfinite(v) for v in out):
        return fallback

    return out


# ============================================================
# Strict SSoT Mesh Resolution
# ============================================================

def _project_root() -> str:
    """
    Single Source of Truth: the project base directory is SC_PROJECT_ROOT.
    """
    base = os.environ.get("SC_PROJECT_ROOT", "").strip()
    if not base:
        raise RuntimeError("SC_PROJECT_ROOT ist nicht gesetzt (strict).")
    base = os.path.abspath(os.path.normpath(os.path.expanduser(base)))
    if not os.path.isdir(base):
        raise RuntimeError(f"SC_PROJECT_ROOT zeigt nicht auf ein Verzeichnis: {base!r}")
    return base


def _looks_like_mesh_path(s: str) -> bool:
    s = (s or "").strip()
    if not s:
        return False
    if s.startswith("package://"):
        return True
    if os.path.isabs(s):
        return True
    # heuristics: contains a path separator or an extension
    if ("/" in s) or ("\\" in s):
        return True
    ext = os.path.splitext(s)[1].lower()
    return ext in (".stl", ".obj", ".ply", ".vtp", ".vtk")


def _resolve_mesh_path(ctx: Any, mesh_ref: str) -> str:
    """
    Resolve a mesh reference to an absolute filesystem path.

    Supported:
      - package://<pkg>/...   (strict via resolve_package_uri)
      - absolute filesystem path
      - relative path (strict relative to SC_PROJECT_ROOT)

    No silent fallbacks: if it cannot be resolved or does not exist -> error.
    """
    mesh_ref = str(mesh_ref or "").strip()
    if not mesh_ref:
        raise ValueError("mesh_ref is empty")

    # package://
    if mesh_ref.startswith("package://"):
        abs_path = resolve_package_uri(mesh_ref)
        abs_path = os.path.abspath(os.path.normpath(abs_path))
        if not os.path.isfile(abs_path):
            raise FileNotFoundError(f"Mesh not found (resolved package://): {mesh_ref} -> {abs_path}")
        return abs_path

    # absolute path
    if os.path.isabs(mesh_ref):
        abs_path = os.path.abspath(os.path.normpath(mesh_ref))
        if not os.path.isfile(abs_path):
            raise FileNotFoundError(f"Mesh not found (abs): {abs_path}")
        return abs_path

    # relative path -> resolve against SC_PROJECT_ROOT (SSoT)
    base = _project_root()
    abs_path = resolve_path(base, mesh_ref)
    if not os.path.isfile(abs_path):
        raise FileNotFoundError(f"Mesh not found (rel to SC_PROJECT_ROOT): {mesh_ref} -> {abs_path}")
    return abs_path


def _read_mesh(mesh_ref: str, ctx: Any = None) -> pv.PolyData:
    path = _resolve_mesh_path(ctx, mesh_ref)
    return cast(pv.PolyData, pv.read(path))


def _get_mounts_yaml(ctx: Any) -> Dict[str, Any]:
    """
    Returns the loaded substrate_mounts.yaml (mapping), from:
      - ctx.mounts_yaml (AppContext)
      - ctx.content.mounts_yaml (AppContent)
    """
    # AppContext.mounts_yaml
    y = getattr(ctx, "mounts_yaml", None)
    if isinstance(y, dict) and y:
        return y

    # AppContent.mounts_yaml (AppContent stores it as attribute)
    content = getattr(ctx, "content", None)
    if content is not None:
        y2 = getattr(content, "mounts_yaml", None)
        if isinstance(y2, dict) and y2:
            return y2

    raise RuntimeError("mounts_yaml nicht im ctx gefunden (weder ctx.mounts_yaml noch ctx.content.mounts_yaml).")


def _mount_mesh_ref_from_key(ctx: Any, mount_key: str) -> str:
    """
    Lookup mounts[mount_key].mesh in substrate_mounts.yaml.
    """
    mount_key = str(mount_key or "").strip()
    if not mount_key:
        raise ValueError("mount_key is empty")

    y = _get_mounts_yaml(ctx)
    mounts = y.get("mounts")
    if not isinstance(mounts, dict) or not mounts:
        raise RuntimeError("substrate_mounts.yaml: 'mounts' fehlt oder ist leer.")

    spec = mounts.get(mount_key)
    if not isinstance(spec, dict):
        raise KeyError(f"substrate_mounts.yaml: mount '{mount_key}' nicht gefunden.")

    mesh = spec.get("mesh")
    mesh = str(mesh or "").strip()
    if not mesh:
        raise KeyError(f"substrate_mounts.yaml: mounts['{mount_key}'].mesh fehlt/leer.")
    return mesh


def _mount_scene_offset_xyz_mm(ctx: Any, mount_key: str) -> Optional[Tuple[float, float, float]]:
    """
    Optional: mounts[mount_key].scene_offset.xyz in mm.
    """
    try:
        y = _get_mounts_yaml(ctx)
        mounts = y.get("mounts")
        if not isinstance(mounts, dict):
            return None
        spec = mounts.get(str(mount_key))
        if not isinstance(spec, dict):
            return None
        scene_offset = spec.get("scene_offset")
        if not isinstance(scene_offset, dict):
            return None
        xyz = scene_offset.get("xyz")
        if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
            return None
        x, yv, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        if not (np.isfinite(x) and np.isfinite(yv) and np.isfinite(z)):
            return None
        return (x, yv, z)
    except Exception:
        return None


# ============================================================
# Mesh Loading / Placement (SSoT-aware)
# ============================================================

def load_mount_mesh_from_key(ctx: Any, mount_key: str) -> pv.PolyData:
    """
    Load mount mesh by key.

    Correct behavior (SSoT):
      - If mount_key is a real mesh path (package://, abs, rel-with-ext) -> read directly.
      - Else treat as mount ID and look up in substrate_mounts.yaml -> mounts[mount_key].mesh
    """
    mount_key = str(mount_key or "").strip()
    if not mount_key:
        raise ValueError("mount_key is empty")

    # If it's already a path-like ref, read directly (still strict-resolved)
    if _looks_like_mesh_path(mount_key):
        return _read_mesh(mount_key, ctx)

    # Otherwise interpret as catalog key -> mounts yaml
    mesh_ref = _mount_mesh_ref_from_key(ctx, mount_key)
    return _read_mesh(mesh_ref, ctx)


def load_substrate_mesh_from_key(ctx: Any, substrate_key: str) -> pv.PolyData:
    """
    Load substrate mesh by key.

    NOTE: Your project likely has a substrate catalog too (similar to mounts).
    For now (strict, but practical):
      - If substrate_key is path-like -> read directly.
      - Else try to resolve it as relative mesh name inside the bringup resources, if possible:
          ctx.content.scene_yaml may define substrate meshes, or your recipes may store package:// already.
    """
    substrate_key = str(substrate_key or "").strip()
    if not substrate_key:
        raise ValueError("substrate_key is empty")

    if _looks_like_mesh_path(substrate_key):
        return _read_mesh(substrate_key, ctx)

    # If recipe provides only a key, this is ambiguous without a dedicated substrate catalog.
    # Fail loudly so you can enforce SSoT (recommended: store package://... in recipe.substrate).
    raise FileNotFoundError(
        f"Substrate mesh key '{substrate_key}' ist kein Pfad und es gibt hier keinen Substrate-Katalog-Resolver. "
        "Empfehlung: Rezept.substrate als package://.../*.stl speichern (SSoT)."
    )


def load_cage_mesh(ctx: Any) -> pv.PolyData:
    """
    Load cage mesh.

    SSoT path: ctx.content.scene_yaml() -> scene_objects[] where id == 'cage' -> mesh
    Fallback: ctx.cage_mesh (string mesh ref).
    """
    # Preferred: scene.yaml
    try:
        content = getattr(ctx, "content", None)
        if content is not None:
            scene_fn = getattr(content, "scene_yaml", None)
            if callable(scene_fn):
                scene = scene_fn()
                objs = scene.get("scene_objects")
                if isinstance(objs, list):
                    for o in objs:
                        if isinstance(o, dict) and str(o.get("id", "")).strip() == "cage":
                            mesh_ref = str(o.get("mesh", "") or "").strip()
                            if mesh_ref:
                                return _read_mesh(mesh_ref, ctx)
    except Exception:
        # keep searching/fallback below
        pass

    mesh_ref = getattr(ctx, "cage_mesh", None)
    if isinstance(mesh_ref, str) and mesh_ref.strip():
        return _read_mesh(mesh_ref.strip(), ctx)

    raise FileNotFoundError("No cage mesh configured (neither scene.yaml[id=cage].mesh nor ctx.cage_mesh).")


def place_substrate_on_mount(ctx: Any, substrate_mesh: pv.PolyData, *, mount_key: str) -> pv.PolyData:
    """
    Place substrate on mount.

    Minimal but consistent behavior:
      - if mounts[mount_key].scene_offset.xyz exists, apply it (mm translation)
    """
    out = substrate_mesh.copy(deep=True)

    off = _mount_scene_offset_xyz_mm(ctx, mount_key)
    if off is not None:
        dx, dy, dz = off
        out.translate([dx, dy, dz], inplace=True)

    return out


def place_substrate_on_mount_origin(substrate_mesh: pv.PolyData) -> pv.PolyData:
    """
    Place substrate at origin. Kept for API compatibility.
    """
    return substrate_mesh.copy(deep=True)


# ============================================================
# Path post-processing (required by SceneManager)
# ============================================================

def _postprocess_compiled_path_strict(
    tcp_mm: Any,
    normal: Any,
    *,
    side_path_params: Dict[str, Any],
) -> Tuple[np.ndarray, np.ndarray]:
    """
    STRICT postprocess hook used by SceneManager.

    Minimal safe behavior:
      - converts tcp_mm and normal to Nx3 float arrays
      - ensures same length by truncation
      - optional trim support (start/end indices)
      - returns (tcp, normals)
    """
    P = np.asarray(tcp_mm, dtype=float).reshape(-1, 3) if tcp_mm is not None else np.zeros((0, 3), dtype=float)
    N = np.asarray(normal, dtype=float).reshape(-1, 3) if normal is not None else np.zeros((0, 3), dtype=float)

    n = int(min(len(P), len(N)))
    if n <= 0:
        return np.zeros((0, 3), dtype=float), np.zeros((0, 3), dtype=float)

    P = P[:n].copy()
    N = N[:n].copy()

    # Optional: basic trim config
    try:
        trim = side_path_params.get("trim", None)
        if isinstance(trim, dict):
            a = int(trim.get("start", 0) or 0)
            b = trim.get("end", None)
            b_i = int(b) if b is not None else n
            if b_i < 0:
                b_i = max(0, n + b_i)
            a = max(0, min(n, a))
            b_i = max(a, min(n, b_i))
            P = P[a:b_i]
            N = N[a:b_i]
    except Exception:
        pass

    return P, N
