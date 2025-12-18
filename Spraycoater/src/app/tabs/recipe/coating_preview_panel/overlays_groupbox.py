# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional, Callable, Dict, Any

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QGroupBox, QWidget, QHBoxLayout, QCheckBox, QLabel, QSizePolicy

from .views_3d.overlays import OverlayRenderer


class OverlaysGroupBox(QGroupBox):
    """
    Overlays: eine horizontale Zeile mit Toggles
      Mask  Path  Hits  Misses  Normals  Lokale KS
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
        self._build_renderer(
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
        self._connect_signals()

    # ---------------- UI ----------------

    def _build_ui(self) -> None:
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(14)

        def add_toggle(text: str, checked: bool = False) -> QCheckBox:
            lab = QLabel(text, self)
            lab.setStyleSheet("font-weight:600;")
            chk = QCheckBox(self)
            chk.setChecked(bool(checked))
            lay.addWidget(lab)
            lay.addWidget(chk)
            return chk

        self.chkShowMask = add_toggle("Mask", False)
        self.chkShowPath = add_toggle("Path", True)
        self.chkShowHits = add_toggle("Hits", False)
        self.chkShowMisses = add_toggle("Misses", False)
        self.chkShowNormals = add_toggle("Normals", False)
        self.chkShowLocalFrames = add_toggle("Lokale KS", False)

        lay.addStretch(1)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

    def _build_renderer(
        self,
        *,
        add_mesh_fn: Callable[..., Any],
        clear_layer_fn: Callable[[str], None],
        add_path_polyline_fn: Callable[..., Any],
        show_poly_fn: Callable[..., Any],
        show_frames_at_fn: Callable[..., None],
        set_layer_visible_fn: Callable[..., Any],
        update_2d_scene_fn: Callable[[Any, Any, Any], None],
        layers: Dict[str, str],
        get_bounds: Callable[[], Any],
    ) -> None:
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

    def _connect_signals(self) -> None:
        self.chkShowMask.toggled.connect(self._on_mask_toggled)
        self.chkShowPath.toggled.connect(self._on_path_toggled)
        self.chkShowHits.toggled.connect(self._on_hits_toggled)
        self.chkShowMisses.toggled.connect(self._on_misses_toggled)
        self.chkShowNormals.toggled.connect(self._on_normals_toggled)
        self.chkShowLocalFrames.toggled.connect(self._on_frames_toggled)

        # externe Listener optional: Signale weiterreichen
        self.chkShowMask.toggled.connect(self.maskToggled.emit)
        self.chkShowPath.toggled.connect(self.pathToggled.emit)
        self.chkShowHits.toggled.connect(self.hitsToggled.emit)
        self.chkShowMisses.toggled.connect(self.missesToggled.emit)
        self.chkShowNormals.toggled.connect(self.normalsToggled.emit)
        self.chkShowLocalFrames.toggled.connect(self.localFramesToggled.emit)

    # ---------------- Convenience ----------------

    def set_defaults(
        self,
        *,
        mask: bool = False,
        path: bool = True,
        hits: bool = False,
        misses: bool = False,
        normals: bool = False,
        local_frames: bool = False,
    ) -> None:
        self.chkShowMask.setChecked(bool(mask))
        self.chkShowPath.setChecked(bool(path))
        self.chkShowHits.setChecked(bool(hits))
        self.chkShowMisses.setChecked(bool(misses))
        self.chkShowNormals.setChecked(bool(normals))
        self.chkShowLocalFrames.setChecked(bool(local_frames))

    def apply_visibility(self, vis: dict | None) -> None:
        """Sichtbarkeit übernehmen (ohne UI zu ändern)."""
        if hasattr(self, "overlays") and self.overlays:
            self.overlays.apply_visibility(vis or {})

    def render_compiled(self, **kwargs) -> dict:
        return self.overlays.render_compiled(**kwargs)

    # ---------------- Intern: rebuild on enable ----------------

    def _rebuild_if_enabled(self, flag: bool, keys: list[str]) -> None:
        if flag and hasattr(self, "overlays") and self.overlays:
            self.overlays.rebuild_layers(keys)

    # ---------------- Toggle handler ----------------

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
