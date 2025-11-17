# -*- coding: utf-8 -*-
# File: widgets/process_control_widget.py
from __future__ import annotations
import os
from typing import Optional, Dict, Any, List

from PyQt6 import uic, QtCore
from PyQt6.QtWidgets import (
    QWidget,
    QPushButton,
    QLabel,
    QVBoxLayout,
    QGroupBox,
    QPlainTextEdit,
    QHBoxLayout,
    QFileDialog,
    QMessageBox,
)
from PyQt6.QtGui import QFont

from app.model.recipe.recipe import Recipe


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/app/widgets
    return os.path.abspath(os.path.join(here, "..", "..", ".."))


def _ui_path(filename: str) -> str:
    # erwartet: resource/ui/tabs/process/process_control.ui
    return os.path.join(_project_root(), "resource", "ui", "tabs", "process", filename)


class ProcessControlWidget(QWidget):
    """
    Process-UI (Fallback ohne .ui-Datei):

      Recipe-Box:
        - lblRecipeName (Name des aktuellen Rezepts)
        - links:  txtRecipeSummary  (Meta, parameters, planner, paths – YAML-ähnlich)
        - rechts: txtRecipeCoords   (alle kompilierten Posen, eine Pose pro Zeile)

      Darunter:
        Process Control: Init / Load Recipe / Start / Stop (Buttons nebeneinander)

    Outbound-Signale:
      initRequested, loadRecipeRequested, startRequested, stopRequested

    Bridge-Auto-Wiring (falls vorhanden):
      - _process.signals / _proc.signals:
          initRequested, loadRecipeRequested, startRequested, stopRequested (emit)
          recipeLoaded(name: str) ODER currentRecipeChanged(name: str) -> set_recipe_name
    """
    initRequested       = QtCore.pyqtSignal()
    loadRecipeRequested = QtCore.pyqtSignal()
    startRequested      = QtCore.pyqtSignal()
    stopRequested       = QtCore.pyqtSignal()

    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        self._recipe: Optional[Recipe] = None

        self.lblRecipeName: Optional[QLabel] = None
        self.txtRecipeSummary: Optional[QPlainTextEdit] = None
        self.txtRecipeCoords: Optional[QPlainTextEdit] = None
        self.btnInit: Optional[QPushButton] = None
        self.btnLoadRecipe: Optional[QPushButton] = None
        self.btnStart: Optional[QPushButton] = None
        self.btnStop: Optional[QPushButton] = None

        ui_file = _ui_path("process_control.ui")
        if not os.path.exists(ui_file):
            # ---------- Fallback: UI on-the-fly ----------
            wrap = QVBoxLayout(self)
            wrap.setContentsMargins(8, 8, 8, 8)
            wrap.setSpacing(8)

            # Recipe-Box (soll vertikal gestreckt werden → Stretch-Faktor 1)
            self.grpRecipe = QGroupBox("Recipe", self)
            vrec = QVBoxLayout(self.grpRecipe)
            vrec.setContentsMargins(8, 8, 8, 8)
            vrec.setSpacing(6)

            self.lblRecipeName = QLabel("-", self.grpRecipe)
            font = self.lblRecipeName.font()
            font.setBold(True)
            self.lblRecipeName.setFont(font)
            vrec.addWidget(self.lblRecipeName)

            # Split: links Werte, rechts Coords
            split = QHBoxLayout()
            split.setContentsMargins(0, 0, 0, 0)
            split.setSpacing(6)

            self.txtRecipeSummary = QPlainTextEdit(self.grpRecipe)
            self.txtRecipeSummary.setReadOnly(True)
            mono = QFont("Monospace")
            mono.setStyleHint(QFont.StyleHint.TypeWriter)
            self.txtRecipeSummary.setFont(mono)
            self.txtRecipeSummary.setMinimumHeight(220)
            split.addWidget(self.txtRecipeSummary, 1)

            self.txtRecipeCoords = QPlainTextEdit(self.grpRecipe)
            self.txtRecipeCoords.setReadOnly(True)
            self.txtRecipeCoords.setFont(mono)
            self.txtRecipeCoords.setMinimumHeight(220)
            split.addWidget(self.txtRecipeCoords, 1)

            vrec.addLayout(split)

            # Recipe bekommt den Haupt-Stretch
            wrap.addWidget(self.grpRecipe, 1)

            # Process Control (Buttons nebeneinander)
            self.grpProcess = QGroupBox("Process Control", self)
            v = QVBoxLayout(self.grpProcess)
            v.setContentsMargins(8, 8, 8, 8)
            v.setSpacing(6)

            btn_row = QHBoxLayout()
            btn_row.setContentsMargins(0, 0, 0, 0)
            btn_row.setSpacing(6)

            self.btnInit = QPushButton("Init", self.grpProcess)
            self.btnLoadRecipe = QPushButton("Load Recipe", self.grpProcess)
            self.btnStart = QPushButton("Start", self.grpProcess)
            self.btnStop = QPushButton("Stop", self.grpProcess)

            for b in (self.btnInit, self.btnLoadRecipe, self.btnStart, self.btnStop):
                b.setMinimumHeight(28)
                btn_row.addWidget(b)

            v.addLayout(btn_row)
            v.addStretch(1)

            # Process-Control bekommt wenig Höhe → Stretch-Faktor 0
            wrap.addWidget(self.grpProcess, 0)

        else:
            # ---------- UI aus .ui-Datei ----------
            uic.loadUi(ui_file, self)
            self.lblRecipeName = self.findChild(QLabel, "lblRecipeName")
            self.txtRecipeSummary = self.findChild(QPlainTextEdit, "txtRecipeSummary")
            self.txtRecipeCoords = self.findChild(QPlainTextEdit, "txtRecipeCoords")
            self.btnInit = self.findChild(QPushButton, "btnInit")
            self.btnLoadRecipe = self.findChild(QPushButton, "btnLoadRecipe")
            self.btnStart = self.findChild(QPushButton, "btnStart")
            self.btnStop = self.findChild(QPushButton, "btnStop")

            mono = QFont("Monospace")
            mono.setStyleHint(QFont.StyleHint.TypeWriter)
            if self.txtRecipeSummary is not None:
                self.txtRecipeSummary.setReadOnly(True)
                self.txtRecipeSummary.setFont(mono)
            if self.txtRecipeCoords is not None:
                self.txtRecipeCoords.setReadOnly(True)
                self.txtRecipeCoords.setFont(mono)

        # ---------- Buttons -> Widget-Signale ----------
        if self.btnInit is not None:
            self.btnInit.clicked.connect(self.initRequested.emit)

        if self.btnLoadRecipe is not None:
            # 1) internes Laden
            self.btnLoadRecipe.clicked.connect(self._on_load_recipe_clicked)
            # 2) nach außen signalisieren
            self.btnLoadRecipe.clicked.connect(self.loadRecipeRequested.emit)

        if self.btnStart is not None:
            self.btnStart.clicked.connect(self.startRequested.emit)
        if self.btnStop is not None:
            self.btnStop.clicked.connect(self.stopRequested.emit)

        # ---------- Bridge-Durchleitung ----------
        self._wire_outbound_to_bridge()
        self._wire_inbound_from_bridge()

    # ---------------- Public API ----------------
    def set_recipe_name(self, name: str | None) -> None:
        """Nur der Name (wenn von der Bridge per Signal kommt)."""
        if not self.lblRecipeName:
            return
        txt = (name or "").strip()
        self.lblRecipeName.setText(txt if txt else "-")

    def set_recipe(self, recipe: Optional[Recipe]) -> None:
        """
        Setzt das aktuelle Rezeptobjekt und aktualisiert:
          - lblRecipeName
          - txtRecipeSummary (ohne Coords)
          - txtRecipeCoords  (nur kompilierten Posen)
        """
        self._recipe = recipe

        # Name
        if self.lblRecipeName is not None:
            if recipe is None:
                self.lblRecipeName.setText("-")
            else:
                rid = (recipe.id or "").strip()
                self.lblRecipeName.setText(rid if rid else "<unnamed>")

        # Textfelder
        if self.txtRecipeSummary is not None:
            self.txtRecipeSummary.setPlainText(self._build_summary_text(recipe))
            self.txtRecipeSummary.moveCursor(self.txtRecipeSummary.textCursor().Start)  # nach oben
        if self.txtRecipeCoords is not None:
            self.txtRecipeCoords.setPlainText(self._build_coords_text(recipe))
            self.txtRecipeCoords.moveCursor(self.txtRecipeCoords.textCursor().Start)

    def current_recipe(self) -> Optional[Recipe]:
        """Für Prozess-Controller: liefert das aktuell geladene Recipe."""
        return self._recipe

    # ---------------- Intern: Text-Building ----------------
    def _build_summary_text(self, recipe: Optional[Recipe]) -> str:
        if recipe is None:
            return ""

        lines: List[str] = []

        # Meta
        lines.append(f"id: {recipe.id or ''}")
        if recipe.description:
            lines.append(f"description: {recipe.description}")
        if recipe.tool:
            lines.append(f"tool: {recipe.tool}")
        if recipe.substrate:
            lines.append(f"substrate: {recipe.substrate}")
        if recipe.substrate_mount:
            lines.append(f"substrate_mount: {recipe.substrate_mount}")

        subs = recipe.substrates if recipe.substrates else (
            [recipe.substrate] if recipe.substrate else []
        )
        if subs and not (len(subs) == 1 and recipe.substrate and subs[0] == recipe.substrate):
            lines.append("substrates:")
            for s in subs:
                lines.append(f"  - {s}")

        # Parameters
        if recipe.parameters:
            lines.append("")
            lines.append("parameters:")
            for k in sorted(recipe.parameters.keys()):
                v = recipe.parameters[k]
                lines.append(f"  {k}: {v}")

        # Planner
        if recipe.planner:
            lines.append("")
            lines.append("planner:")
            for k in sorted(recipe.planner.keys()):
                v = recipe.planner[k]
                lines.append(f"  {k}: {v}")

        # Paths-Definitionen (ohne Punkte)
        if recipe.paths_by_side:
            lines.append("")
            lines.append("paths_by_side:")
            for side, p in recipe.paths_by_side.items():
                lines.append(f"  {side}:")
                for pk, pv in p.items():
                    if pk in ("points_mm", "polyline_mm"):
                        continue
                    lines.append(f"    {pk}: {pv}")

        return "\n".join(lines)

    def _build_coords_text(self, recipe: Optional[Recipe]) -> str:
        if recipe is None:
            return ""

        pc = recipe.paths_compiled or {}
        sides = pc.get("sides") or {}
        if not isinstance(sides, dict) or not sides:
            return ""

        lines: List[str] = []
        frame = pc.get("frame")
        tool_frame = pc.get("tool_frame")

        if frame or tool_frame:
            if frame:
                lines.append(f"frame: {frame}")
            if tool_frame:
                lines.append(f"tool_frame: {tool_frame}")
            lines.append("")

        for side, sdata in sides.items():
            lines.append(f"[side: {side}]")
            poses = sdata.get("poses_quat") or []
            if not poses:
                lines.append("  (no poses)")
                lines.append("")
                continue

            for i, p in enumerate(poses):
                x = float(p.get("x", 0.0))
                y = float(p.get("y", 0.0))
                z = float(p.get("z", 0.0))
                qx = float(p.get("qx", 0.0))
                qy = float(p.get("qy", 0.0))
                qz = float(p.get("qz", 0.0))
                qw = float(p.get("qw", 1.0))
                # Eine Zeile pro Pose
                lines.append(
                    f"  {i:04d}: "
                    f"x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                    f"qx={qx:.4f}, qy={qy:.4f}, qz={qz:.4f}, qw={qw:.4f}"
                )
            lines.append("")

        return "\n".join(lines)

    # ---------------- Intern: Rezept laden ----------------
    def _on_load_recipe_clicked(self) -> None:
        """
        Öffnet einen File-Dialog, lädt ein Rezept aus YAML, setzt self._recipe
        und aktualisiert die Recipe-GroupBox (links Werte, rechts Coords).
        """
        start_dir = getattr(getattr(self.ctx, "paths", None), "recipe_dir", os.getcwd())
        fname, _ = QFileDialog.getOpenFileName(
            self,
            "Rezept laden",
            start_dir,
            "YAML (*.yaml *.yml)",
        )
        if not fname:
            return

        try:
            recipe = Recipe.load_yaml(fname)
        except Exception as e:
            try:
                QMessageBox.critical(self, "Ladefehler", f"Rezept konnte nicht geladen werden:\n{e}")
            except RuntimeError:
                QMessageBox.critical(None, "Ladefehler", f"Rezept konnte nicht geladen werden:\n{e}")
            return

        self.set_recipe(recipe)

        base = os.path.basename(fname)
        try:
            QMessageBox.information(self, "Rezept geladen", base)
        except RuntimeError:
            QMessageBox.information(None, "Rezept geladen", base)

    # ---------------- Bridge Wiring ----------------
    def _wire_outbound_to_bridge(self) -> None:
        br = getattr(self.bridge, "_process", None) or getattr(self.bridge, "_proc", None)
        sig = getattr(br, "signals", None) if br else None
        if not sig:
            return

        if hasattr(sig, "initRequested"):
            self.initRequested.connect(sig.initRequested.emit)
        if hasattr(sig, "loadRecipeRequested"):
            self.loadRecipeRequested.connect(sig.loadRecipeRequested.emit)
        if hasattr(sig, "startRequested"):
            self.startRequested.connect(sig.startRequested.emit)
        if hasattr(sig, "stopRequested"):
            self.stopRequested.connect(sig.stopRequested.emit)

    def _wire_inbound_from_bridge(self) -> None:
        br = (
            getattr(self.bridge, "_process", None)
            or getattr(self.bridge, "_proc", None)
            or getattr(self.bridge, "_recipe", None)
        )
        sig = getattr(br, "signals", None) if br else None
        if not sig:
            return

        # verschiedene mögliche Namensvarianten unterstützen
        if hasattr(sig, "recipeLoaded"):
            sig.recipeLoaded.connect(self.set_recipe_name)  # str
        if hasattr(sig, "currentRecipeChanged"):
            sig.currentRecipeChanged.connect(self.set_recipe_name)  # str
        if hasattr(sig, "recipeNameChanged"):
            sig.recipeNameChanged.connect(self.set_recipe_name)  # str
