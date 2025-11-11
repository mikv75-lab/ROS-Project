# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Callable, Dict, Any

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QHBoxLayout, QCheckBox, QLabel, QSizePolicy
)

from .overlays import OverlayRenderer  # Renderer lebt in der GroupBox


class OverlaysGroupBox(QGroupBox):
    """
    Overlays – alles in EINER horizontalen Zeile:
      Mask [ ]   Path [x]   Hits [ ]   Misses [ ]   Normals [ ]   Lokale KS [ ]
    """

    maskToggled = pyqtSignal(bool)
    pathToggled = pyqtSignal(bool)
    hitsToggled = pyqtSignal(bool)
    missesToggled = pyqtSignal(bool)
    normalsToggled = pyqtSignal(bool)
    localFramesToggled = pyqtSignal(bool)

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        # --- Funktionen/Callbacks aus dem Panel/Scene ----
        add_mesh_fn: Callable[..., Any],
        clear_layer_fn: Callable[[str], None],
        add_path_polyline_fn: Callable[..., Any],
        show_poly_fn: Callable[..., Any],
        show_frames_at_fn: Callable[..., None],
        set_layer_visible_fn: Callable[..., Any],
        update_2d_scene_fn: Callable[[Any, Any, Any], None],
        layers: Dict[str, str],
        get_bounds: Callable[[], Any],
    ):
        super().__init__("Overlays", parent)
        self._build_ui()

        # Renderer hier erzeugen
        self.overlays = OverlayRenderer(
            add_mesh_fn=add_mesh_fn,
            clear_layer_fn=clear_layer_fn,
            add_path_polyline_fn=add_path_polyline_fn,
            show_poly_fn=show_poly_fn,
            show_frames_at_fn=show_frames_at_fn,
            set_layer_visible_fn=set_layer_visible_fn,
            update_2d_scene_fn=update_2d_scene_fn,
            layers=layers,
            get_bounds=get_bounds,
        )

        # Wiring Sichtbarkeit -> OverlayRenderer (+ Rebuild beim Aktivieren)
        self.chkShowMask.toggled.connect(self._on_mask_toggled)
        self.chkShowPath.toggled.connect(self._on_path_toggled)
        self.chkShowHits.toggled.connect(self._on_hits_toggled)
        self.chkShowMisses.toggled.connect(self._on_misses_toggled)
        self.chkShowNormals.toggled.connect(self._on_normals_toggled)
        self.chkShowLocalFrames.toggled.connect(self._on_frames_toggled)

    # ---------- UI ----------
    def _build_ui(self) -> None:
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(14)

        def add(lbl: str, checked: bool = False) -> QCheckBox:
            lab = QLabel(lbl, self)
            lab.setStyleSheet("font-weight:600;")
            chk = QCheckBox(self)
            chk.setChecked(checked)
            lay.addWidget(lab)
            lay.addWidget(chk)
            return chk

        self.chkShowMask        = add("Mask", False)
        self.chkShowPath        = add("Path", True)
        self.chkShowHits        = add("Hits", False)
        self.chkShowMisses      = add("Misses", False)
        self.chkShowNormals     = add("Normals", False)
        self.chkShowLocalFrames = add("Lokale KS", False)

        lay.addStretch(1)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

        # Signale für externe Listener (optional)
        self.maskToggled = self.chkShowMask.toggled
        self.pathToggled = self.chkShowPath.toggled
        self.hitsToggled = self.chkShowHits.toggled
        self.missesToggled = self.chkShowMisses.toggled
        self.normalsToggled = self.chkShowNormals.toggled
        self.localFramesToggled = self.chkShowLocalFrames.toggled

    # ---------- Convenience ----------
    def set_defaults(
        self, *, mask=False, path=True, hits=False, misses=False, normals=False, local_frames=False
    ) -> None:
        self.chkShowMask.setChecked(bool(mask))
        self.chkShowPath.setChecked(bool(path))
        self.chkShowHits.setChecked(bool(hits))
        self.chkShowMisses.setChecked(bool(misses))
        self.chkShowNormals.setChecked(bool(normals))
        self.chkShowLocalFrames.setChecked(bool(local_frames))

    def apply_visibility(self, vis: dict | None) -> None:
        """Direkte Sichtbarkeitsübernahme (ohne UI ändern)."""
        if hasattr(self, "overlays") and self.overlays:
            self.overlays.apply_visibility(vis or {})

    # ---- Durchreicher fürs Rendern der kompilierten Pfade ----
    def render_compiled(self, **kwargs) -> dict:
        return self.overlays.render_compiled(**kwargs)

    # ---------- Toggle-Handler: set_visible + optionaler Rebuild ----------
    def _rebuild_if_enabled(self, flag: bool, keys: list[str]) -> None:
        if flag and hasattr(self, "overlays") and self.overlays:
            self.overlays.rebuild_layers(keys)

    def _on_mask_toggled(self, v: bool) -> None:
        self.overlays.set_mask_visible(v)
        self._rebuild_if_enabled(v, ["mask"])

    def _on_path_toggled(self, v: bool) -> None:
        self.overlays.set_path_visible(v)
        self._rebuild_if_enabled(v, ["path"])

    def _on_hits_toggled(self, v: bool) -> None:
        self.overlays.set_hits_visible(v)
        self._rebuild_if_enabled(v, ["hits"])

    def _on_misses_toggled(self, v: bool) -> None:
        self.overlays.set_misses_visible(v)
        self._rebuild_if_enabled(v, ["misses"])

    def _on_normals_toggled(self, v: bool) -> None:
        self.overlays.set_normals_visible(v)
        self._rebuild_if_enabled(v, ["normals"])

    def _on_frames_toggled(self, v: bool) -> None:
        self.overlays.set_frames_visible(v)
        self._rebuild_if_enabled(v, ["frames"])
