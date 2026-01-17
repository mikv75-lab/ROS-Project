# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional, Callable, Dict, Any

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QGroupBox, QWidget, QHBoxLayout, QCheckBox, QLabel, QSizePolicy

from .overlays import OverlayRenderer


class OverlaysGroupBox(QGroupBox):
    maskToggled = pyqtSignal(bool)
    pathToggled = pyqtSignal(bool)
    hitsToggled = pyqtSignal(bool)
    missesToggled = pyqtSignal(bool)
    normalsToggled = pyqtSignal(bool)

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        add_mesh_fn: Callable[..., Any],
        clear_layer_fn: Callable[[str], None],
        add_path_polyline_fn: Callable[..., Any],
        show_poly_fn: Callable[..., Any],
        set_layer_visible_fn: Callable[..., Any],
        update_2d_scene_fn: Callable[[Any, Any, Any], None],
        layers: Dict[str, str],
        get_bounds: Callable[[], Any],
    ):
        super().__init__("Overlays", parent)

        self._build_ui()

        self.overlays = OverlayRenderer(
            add_mesh_fn=add_mesh_fn,
            clear_layer_fn=clear_layer_fn,
            add_path_polyline_fn=add_path_polyline_fn,
            show_poly_fn=show_poly_fn,
            # show_frames_at_fn entfernt
            set_layer_visible_fn=set_layer_visible_fn,
            update_2d_scene_fn=update_2d_scene_fn,
            layers=layers,
            get_bounds=get_bounds,
        )

        self._connect_signals()

    def _build_ui(self) -> None:
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(14)

        def add_toggle(text: str, checked: bool = True) -> QCheckBox:
            lab = QLabel(text, self)
            lab.setStyleSheet("font-weight:600;")
            chk = QCheckBox(self)
            chk.setChecked(bool(checked))
            lay.addWidget(lab)
            lay.addWidget(chk)
            return chk

        # Alle standardmäßig auf True (checked)
        self.chkShowMask = add_toggle("Mask", True)
        self.chkShowPath = add_toggle("Path", True)
        self.chkShowHits = add_toggle("Hits", True)
        self.chkShowMisses = add_toggle("Misses", True)
        self.chkShowNormals = add_toggle("Normals", True)
        # Frames entfernt

        lay.addStretch(1)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

    def _connect_signals(self) -> None:
        self.chkShowMask.toggled.connect(self._on_mask_toggled)
        self.chkShowPath.toggled.connect(self._on_path_toggled)
        self.chkShowHits.toggled.connect(self._on_hits_toggled)
        self.chkShowMisses.toggled.connect(self._on_misses_toggled)
        self.chkShowNormals.toggled.connect(self._on_normals_toggled)

        self.chkShowMask.toggled.connect(self.maskToggled.emit)
        self.chkShowPath.toggled.connect(self.pathToggled.emit)
        self.chkShowHits.toggled.connect(self.hitsToggled.emit)
        self.chkShowMisses.toggled.connect(self.missesToggled.emit)
        self.chkShowNormals.toggled.connect(self.normalsToggled.emit)

    # ✅ WIRD VOM PreviewPanel BENÖTIGT
    def current_visibility(self) -> Dict[str, bool]:
        return {
            "path": bool(self.chkShowPath.isChecked()),
            "hits": bool(self.chkShowHits.isChecked()),
            "misses": bool(self.chkShowMisses.isChecked()),
            "normals": bool(self.chkShowNormals.isChecked()),
            "mask": bool(self.chkShowMask.isChecked()),
            # "frames" entfernt
        }

    def _rebuild_if_enabled(self, flag: bool, keys: list[str]) -> None:
        if flag and getattr(self, "overlays", None):
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