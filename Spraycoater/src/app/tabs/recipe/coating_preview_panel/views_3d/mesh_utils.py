# mesh_utils.py
from __future__ import annotations

import os
import math
import logging
from typing import Tuple, Optional, Dict

import meshio
import numpy as np
import pyvista as pv

_LOG = logging.getLogger("app.tabs.recipe.mesh_utils")

# ---------- Kontextbasierte Pfad-Helfer ----------
def _project_root(ctx) -> str:
    # Liefert den Projekt-Root:
    # - bevorzugt ctx.project_root (wenn vorhanden)
    # - sonst relativ zum aktuellen File (4 Ebenen hoch)
    if ctx and getattr(ctx, "project_root", None):
        return os.path.abspath(ctx.project_root)
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))

def _resource_root(ctx) -> str:
    # Liefert den Resource-Root:
    # - bevorzugt ctx.resource_root (wenn vorhanden)
    # - sonst <project_root>/resource
    if ctx and getattr(ctx, "resource_root", None):
        return os.path.abspath(ctx.resource_root)
    return os.path.join(_project_root(ctx), "resource")

# ---------- Pfad-Auflösung ----------
def _resolve_mesh_path(uri_or_path: str, ctx=None) -> str:
    # Nimmt einen URI/Path und versucht daraus einen existierenden Dateipfad zu machen:
    # - absoluter Pfad (wenn existiert)
    # - package://... via ament_index
    # - resource/... relativ zum Projekt bzw. resource_root
    # - sonst relativ zum project_root
    p = (uri_or_path or "").strip()
    if not p:
        raise FileNotFoundError("Leerer Mesh-Pfad")

    if os.path.isabs(p) and os.path.exists(p):
        return p

    if p.startswith("package://"):
        rest = p[len("package://"):]
        pkg, _, sub = rest.partition("/")
        if not pkg or not sub:
            raise FileNotFoundError(f"Ungültige package-URI: {p}")
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(pkg)
        cand = os.path.join(share, sub)
        if os.path.exists(cand):
            return cand
        raise FileNotFoundError(f"package-URI nicht gefunden: {p}")

    if p.startswith("resource/"):
        # 1) direkt relativ zum project_root
        cand = os.path.join(_project_root(ctx), p)
        if os.path.exists(cand):
            return cand
        # 2) relativ zum expliziten resource_root
        cand2 = os.path.join(_resource_root(ctx), p[len("resource/"):])
        if os.path.exists(cand2):
            return cand2
        raise FileNotFoundError(f"resource-Pfad nicht gefunden: {p}")

    # relative Pfade am Projekt-Root probieren
    cand = os.path.join(_project_root(ctx), p)
    if os.path.exists(cand):
        return cand

    # zuletzt: ggf. ist p bereits relativ/normal, aber existiert im CWD
    if os.path.exists(p):
        return os.path.abspath(p)

    raise FileNotFoundError(f"Mesh nicht gefunden/auflösbar: {uri_or_path}")

# ---------- Geometrie-Utils ----------
def _rpy_deg_to_matrix(rpy_deg: Tuple[float, float, float]) -> np.ndarray:
    # Konvertiert Roll/Pitch/Yaw in Grad zu einer 3x3 Rotationsmatrix.
    # Reihenfolge: Rz(yaw) @ Ry(pitch) @ Rx(roll)
    r, p, y = [math.radians(v) for v in rpy_deg]
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r),  math.cos(r)]])
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [0,            1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y),  math.cos(y), 0],
                   [0,            0,           1]])
    return Rz @ Ry @ Rx

def apply_transform(
    mesh: pv.PolyData,
    *,
    translate_mm: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    rpy_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> pv.PolyData:
    # Wendet Rotation (rpy_deg) und Translation (translate_mm) auf ein Mesh an.
    # Rückgabe ist eine Kopie (deep copy), Original bleibt unverändert.
    out = mesh.copy(deep=True)
    R = _rpy_deg_to_matrix(rpy_deg)
    out.points[:] = (out.points @ R.T) + np.asarray(translate_mm, dtype=float)[None, :]
    return out

def center_mesh_at_origin(mesh: pv.PolyData, *, mode: str = "geom_center") -> pv.PolyData:
    # Zentriert ein Mesh auf (0,0,0).
    # - mode="bbox_center": Bounding-Box Mittelpunkt
    # - sonst: geometrisches Zentrum (mesh.center)
    # Hinweis: ist als Utility gedacht und ersetzt nicht die Mount-Floor-Normalisierung.
    m = mesh.copy()
    if mode == "bbox_center":
        xmin, xmax, ymin, ymax, zmin, zmax = m.bounds
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        cz = 0.5 * (zmin + zmax)
        c = np.array([cx, cy, cz], dtype=float)
    else:
        c = np.array(m.center, dtype=float)
    m.translate(-c, inplace=True)
    return m

def mount_to_origin_floor(mesh: pv.PolyData, *, center_mode: str = "bbox_center") -> pv.PolyData:
    # Normalisiert ein Mount-Mesh in ein "Mount-Frame":
    # - XY so verschieben, dass die Mitte bei (0,0) liegt (BBox-Mitte oder mesh.center)
    # - Z so verschieben, dass die Unterseite (z_min) auf Z=0 liegt
    m = mesh.copy(deep=True)
    xmin, xmax, ymin, ymax, zmin, _ = m.bounds
    if center_mode == "bbox_center":
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
    else:
        cx, cy, _ = m.center
    m.translate((-cx, -cy, -zmin), inplace=True)
    return m

def place_substrate_on_mount_origin(
    sub_mesh: pv.PolyData,
    *,
    offset_xy: Tuple[float, float] = (0.0, 0.0),
    offset_z: float = 0.0,
    rpy_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    triangulate: bool = True,
) -> pv.PolyData:
    # Platziert ein Substrat relativ zum bereits normalisierten Mount-Frame:
    # 1) Substrat-Unterseite (z_min) wird auf z=0 gesetzt
    # 2) anschließend werden scene_offset (Translation + Rotation) angewendet
    m = sub_mesh.copy()
    if triangulate:
        # optional triangulieren (falls Mesh nicht aus Dreiecken besteht)
        try:
            if hasattr(m, "is_all_triangles") and not m.is_all_triangles():
                m = m.triangulate()
        except Exception:
            pass

    # Unterseite auf z=0
    zmin = float(m.bounds[4])
    m.translate((0.0, 0.0, -zmin), inplace=True)

    # scene_offset anwenden
    dx, dy = offset_xy
    dz = float(offset_z)
    m = apply_transform(m, translate_mm=(dx, dy, dz), rpy_deg=rpy_deg)
    return m

def __load_mesh_meshio(path: str) -> pv.PolyData:
    # Liest ein Mesh über meshio ein und baut daraus ein pv.PolyData.
    # Erwartet Triangles (Nx3). Es wird bewusst kein clean()/Normals o.ä. gemacht.
    if not os.path.exists(path):
        raise FileNotFoundError(f"Mesh-Pfad existiert nicht: {path}")

    m = meshio.read(path)

    # Punkte extrahieren (Nx3)
    pts = np.asarray(m.points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise RuntimeError(f"Ungültige Punktmatrix: shape={pts.shape}")
    pts = np.ascontiguousarray(pts[:, :3])

    # Triangles holen: bevorzugt cells_dict["triangle"], ansonsten heuristisch aus m.cells
    cells = getattr(m, "cells_dict", None) or {}
    tri = cells.get("triangle", None)
    if tri is None or tri.size == 0:
        for cb in getattr(m, "cells", []):
            data = getattr(cb, "data", None)
            if data is None and isinstance(cb, (tuple, list)) and len(cb) == 2:
                data = cb[1]
            if data is None:
                continue
            arr = np.asarray(data)
            if arr.ndim == 2 and arr.shape[1] == 3 and arr.size:
                tri = arr
                break
    if tri is None or tri.size == 0:
        raise RuntimeError("Keine triangular faces gefunden (erwartet Nx3).")

    tri = np.ascontiguousarray(tri, dtype=np.int64)

    # Index-Range prüfen
    n_pts = pts.shape[0]
    if tri.min() < 0 or tri.max() >= n_pts:
        raise ValueError(f"Triangle-Index außerhalb 0..{n_pts-1}: min={tri.min()}, max={tri.max()} (n_pts={n_pts})")

    # PyVista-Faces-Format: [3,i,j,k, 3,i,j,k, ...]
    faces = np.empty((tri.shape[0], 4), dtype=np.int64)
    faces[:, 0] = 3
    faces[:, 1:] = tri
    faces = faces.ravel(order="C")

    pv_mesh = pv.PolyData(pts, faces)
    return pv_mesh

# ---------- Loader ----------
def load_mesh(uri_or_path: str, ctx=None) -> pv.PolyData:
    # Public Loader: löst zuerst den Pfad auf und lädt dann via meshio->pyvista.
    path = _resolve_mesh_path(uri_or_path, ctx)
    _LOG.debug("load_mesh: resolved path = %s", path)
    return __load_mesh_meshio(path)

# ---------- Mounts: strikt aus substrate_mounts.yaml ----------
def _get_mounts_yaml(ctx) -> Dict:
    # Holt ctx.mounts_yaml (bereits als Dict geladen) und liefert den "mounts"-Block.
    my = getattr(ctx, "mounts_yaml", None)
    if not isinstance(my, dict):
        raise FileNotFoundError("ctx.mounts_yaml fehlt oder ist ungültig.")
    mounts = my.get("mounts")
    if not isinstance(mounts, dict) or not mounts:
        raise FileNotFoundError("ctx.mounts_yaml['mounts'] fehlt oder ist leer.")
    return mounts

def _get_mount_entry_strict(ctx, mount_key: str) -> Dict:
    # Liefert den Mount-Entry für mount_key und wirft bei Fehlern klare Exceptions.
    key = (mount_key or "").strip()
    if not key:
        raise FileNotFoundError("Mount-Key ist leer.")
    mounts = _get_mounts_yaml(ctx)
    entry = mounts.get(key)
    if not entry:
        keys = ", ".join(sorted(mounts.keys()))
        raise FileNotFoundError(
            f"Mount '{key}' nicht in mounts_yaml['mounts'] gefunden. Verfügbar: [{keys}]"
        )
    return entry

def resolve_mount_mesh_path(ctx, mount_key: str) -> str:
    # Liest aus mounts_yaml den Mesh-URI des Mounts und löst ihn zu einem Pfad auf.
    entry = _get_mount_entry_strict(ctx, mount_key)
    mesh_uri = entry.get("mesh")
    if not mesh_uri or not isinstance(mesh_uri, str):
        raise FileNotFoundError(f"Mount '{mount_key}' hat kein gültiges 'mesh'-Feld.")
    path = _resolve_mesh_path(mesh_uri, ctx)
    _LOG.info("Mount '%s' mesh -> %s", mount_key, path)
    return path

def load_mount_mesh_from_key(ctx, mount_key: str) -> pv.PolyData:
    # Lädt ein Mount-Mesh per Key und normalisiert es in den Mount-Frame (Origin + Floor).
    path = resolve_mount_mesh_path(ctx, mount_key)
    _LOG.debug("load_mount_mesh_from_key: path = %s", path)
    mesh = __load_mesh_meshio(path)
    mesh = mount_to_origin_floor(mesh, center_mode="bbox_center")
    return mesh

def get_mount_scene_offset_from_key(
    ctx, mount_key: str
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    # Liefert die relative Pose des Substrats gegenüber dem Mount-Frame:
    # - xyz (Translation) in mm
    # - rpy_deg (Rotation) in Grad
    entry = _get_mount_entry_strict(ctx, mount_key)
    so = entry.get("scene_offset")
    if not isinstance(so, dict):
        raise FileNotFoundError(f"Mount '{mount_key}' hat keinen gültigen scene_offset-Eintrag.")
    xyz = tuple((so.get("xyz") or [0.0, 0.0, 0.0])[:3])
    rpy = tuple((so.get("rpy_deg") or [0.0, 0.0, 0.0])[:3])
    return xyz, rpy

# ---------- Substrate Loader (datei-basiert) ----------
def load_substrate_mesh_from_key(ctx, substrate_key: str) -> pv.PolyData:
    # Lädt ein Substrat-Mesh per Key:
    # - Wenn substrate_key schon ein Pfad/URI ist -> direkt laden
    # - sonst fallback: resource/substrates/<key>.stl im Projekt
    # - sonst fallback: spraycoater_bringup share/resource/substrates/<key>.stl
    key = (substrate_key or "").strip()
    if not key:
        raise FileNotFoundError("Leerer Substrate-Key.")

    if key.startswith("package://") or key.startswith("resource/") or os.path.isabs(key) or os.path.exists(key):
        return load_mesh(key, ctx)

    cand = os.path.join(_project_root(ctx), "resource", "substrates", f"{key}.stl")
    if os.path.exists(cand):
        return load_mesh(cand, ctx)

    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("spraycoater_bringup")
    cand_pkg = os.path.join(share, "resource", "substrates", f"{key}.stl")
    if os.path.exists(cand_pkg):
        return load_mesh(cand_pkg, ctx)

    raise FileNotFoundError(f"Substrate-Mesh nicht gefunden: {substrate_key}")

# ---------- Platzierung (Mount bleibt am Ursprung) ----------
def place_substrate_on_mount(
    ctx, substrate_mesh: pv.PolyData, *, mount_key: Optional[str]
) -> pv.PolyData:
    # Platziert ein Substrat auf einem Mount, wobei der Mount-Frame bereits am Ursprung liegt.
    # - mount_key wird genutzt, um scene_offset (xyz/rpy) aus mounts_yaml zu holen
    # - das Substrat wird auf z=0 gesetzt und dann relativ verschoben/rotiert
    if not mount_key:
        raise FileNotFoundError("place_substrate_on_mount: mount_key ist erforderlich.")

    (xyz_mm, rpy_deg) = get_mount_scene_offset_from_key(ctx, mount_key)

    sub = place_substrate_on_mount_origin(
        substrate_mesh,
        offset_xy=(float(xyz_mm[0]), float(xyz_mm[1])),
        offset_z=float(xyz_mm[2]),
        rpy_deg=(float(rpy_deg[0]), float(rpy_deg[1]), float(rpy_deg[2])),
        triangulate=True,
    )
    return sub
