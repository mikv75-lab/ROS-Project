# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Callable, Dict, Any, List

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QHBoxLayout, QCheckBox, QLabel, QSizePolicy
)

from .overlays import OverlayRenderer  # ✨ NEU: Renderer lebt in der GroupBox

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
        parent: Optional[Widget] = None,
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
        yaml_out_fn: Optional[Callable[[str], None]] = None,
    ):
        super().__init__("Overlays", parent)
        self._build_ui()

        # ✨ Renderer hier erzeugen
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
            yaml_out_fn=yaml_out_fn,
        )

        # Wiring Sichtbarkeit -> OverlayRenderer
        self.maskToggled.connect(lambda v: self.overlays.set_mask_visible(v))
        self.pathToggled.connect(lambda v: self.overlays.set_path_visible(v))
        self.hitsToggled.connect(lambda v: self.overlays.set_hits_visible(v))
        self.missesToggled.connect(lambda v: self.overlays.set_misses_visible(v))
        self.normalsToggled.connect(lambda v: self.overlays.set_normals_visible(v))
        self.localFramesToggled.connect(lambda v: self.overlays.set_frames_visible(v))

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

        # UI-Events -> Signale
        self.chkShowMask.toggled.connect(self.maskToggled)
        self.chkShowPath.toggled.connect(self.pathToggled)
        self.chkShowHits.toggled.connect(self.hitsToggled)
        self.chkShowMisses.toggled.connect(self.missesToggled)
        self.chkShowNormals.toggled.connect(self.normalsToggled)
        self.chkShowLocalFrames.toggled.connect(self.localFramesToggled)

    # ---------- Convenience ----------
    def set_defaults(
        self, *, mask=False, path=True, hits=False, misses=False, normals=False, local_frames=False
    ) -> None:
        # Checkboxes setzen (triggert damit auch den Renderer via Signals)
        self.chkShowMask.setChecked(bool(mask))
        self.chkShowPath.setChecked(bool(path))
        self.chkShowHits.setChecked(bool(hits))
        self.chkShowMisses.setChecked(bool(misses))
        self.chkShowNormals.setChecked(bool(normals))
        self.chkShowLocalFrames.setChecked(bool(local_frames))

    def apply_visibility(self, vis: dict | None) -> None:
        """Optional direkte Sichtbarkeitsübernahme (ohne UI ändern)."""
        if hasattr(self, "overlays") and self.overlays:
            self.overlays.apply_visibility(vis or {})

    # ---- Durchreicher fürs Rendern der kompilierten Pfade ----
    def render_compiled(self, **kwargs) -> dict:
        return self.overlays.render_compiled(**kwargs)
