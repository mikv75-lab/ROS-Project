# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional, Dict, Any, Tuple, List

from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QFormLayout,
    QDoubleSpinBox,
    QSpinBox,
    QCheckBox,
    QComboBox,
    QLabel,
    QFrame,
    QLineEdit,
    QSizePolicy,
    QTabBar,
    QTextEdit,
)

from model.recipe.recipe import Recipe
from model.recipe.recipe_store import RecipeStore
from widgets.planner_groupbox import PlannerGroupBox
from .side_path_editor import SidePathEditor


Bounds = Tuple[float, float, float, float, float, float]


# ----------------------------- UI helpers -----------------------------

def _hline() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setFrameShadow(QFrame.Shadow.Sunken)
    return f


def _set_policy(
    w: QWidget,
    *,
    h: QSizePolicy.Policy = QSizePolicy.Policy.Expanding,
    v: QSizePolicy.Policy = QSizePolicy.Policy.Preferred,
) -> None:
    sp = w.sizePolicy()
    sp.setHorizontalPolicy(h)
    sp.setVerticalPolicy(v)
    w.setSizePolicy(sp)


# ----------------------------- Tabbar with checkboxes -----------------------------

class _TabBarWithChecks(QTabBar):
    """
    QTabBar mit Checkbox pro Tab. Dient als TabBar-Teil eines "QTabWidget-ähnlichen" Aufbaus.
    - currentChanged: wird wie üblich vom QTabBar gesendet
    - checkedChanged(side_name, state): eigenes Signal für Checkbox-Status
    """

    checkedChanged = pyqtSignal(str, bool)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMovable(False)
        self.setExpanding(False)
        self.setUsesScrollButtons(True)
        self._checks: Dict[int, QCheckBox] = {}

    def addTabWithCheck(self, label: str, side_name: str, checked: bool = True) -> int:
        idx = self.addTab(label)
        cb = QCheckBox(self)
        cb.setChecked(bool(checked))
        cb.stateChanged.connect(lambda s, i=idx, sn=side_name: self.checkedChanged.emit(sn, bool(s)))
        self.setTabButton(idx, QTabBar.ButtonPosition.RightSide, cb)
        self._checks[idx] = cb
        return idx

    def setChecked(self, idx: int, checked: bool) -> None:
        cb = self._checks.get(idx)
        if cb is not None:
            cb.setChecked(bool(checked))

    def isChecked(self, idx: int) -> bool:
        cb = self._checks.get(idx)
        return bool(cb.isChecked()) if cb is not None else False


# ----------------------------- Main content widget -----------------------------

class RecipeEditorContent(QWidget):
    """
    Editor-Inhalt:
      - Meta (ID/Desc)
      - Auswahl: recipe-id (aus Store), tool/substrate/mount
      - Parameter / Planner
      - Path editors pro Side
    """

    def __init__(self, *, ctx, store: RecipeStore, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.store = store

        self.sel_recipe: Optional[QComboBox] = None

        self.sel_tool: Optional[QComboBox] = None
        self.sel_substrate: Optional[QComboBox] = None
        self.sel_mount: Optional[QComboBox] = None

        self.e_name: Optional[QLineEdit] = None
        self.e_desc: Optional[QLineEdit] = None

        self._planner_box: Optional[PlannerGroupBox] = None

        self._side_tabbar: Optional[_TabBarWithChecks] = None
        self._side_editors: Dict[str, SidePathEditor] = {}

        self._txt_info: Optional[QTextEdit] = None

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # --- selection row ---
        gb_sel = QGroupBox("Selection", self)
        gb_sel_l = QHBoxLayout(gb_sel)
        gb_sel_l.setContentsMargins(8, 8, 8, 8)
        gb_sel_l.setSpacing(8)

        self.sel_recipe = QComboBox(gb_sel)
        self.sel_recipe.setMinimumWidth(220)
        gb_sel_l.addWidget(QLabel("Recipe:", gb_sel))
        gb_sel_l.addWidget(self.sel_recipe)

        self.sel_tool = QComboBox(gb_sel)
        self.sel_substrate = QComboBox(gb_sel)
        self.sel_mount = QComboBox(gb_sel)

        gb_sel_l.addWidget(QLabel("Tool:", gb_sel))
        gb_sel_l.addWidget(self.sel_tool, 1)
        gb_sel_l.addWidget(QLabel("Substrate:", gb_sel))
        gb_sel_l.addWidget(self.sel_substrate, 1)
        gb_sel_l.addWidget(QLabel("Mount:", gb_sel))
        gb_sel_l.addWidget(self.sel_mount, 1)

        root.addWidget(gb_sel)

        # --- meta ---
        self.gb_meta = QGroupBox("Meta", self)
        self.gb_meta_l = QFormLayout(self.gb_meta)
        self.gb_meta_l.setContentsMargins(8, 8, 8, 8)
        self.gb_meta_l.setSpacing(6)

        self.e_name = QLineEdit(self.gb_meta)
        self.e_name.setPlaceholderText("recipes[..].id (SSoT)")
        self.e_name.setReadOnly(True)
        self.e_name.setToolTip("ID ist durch recipes[..].id vorgegeben und wird automatisch gespeichert/geladen.")
        self.e_desc = QLineEdit(self.gb_meta)
        self.e_desc.setPlaceholderText("Short description ...")

        self.gb_meta_l.addRow("Name:", self.e_name)
        self.gb_meta_l.addRow("Description:", self.e_desc)

        root.addWidget(self.gb_meta)

        # --- planner ---
        self._planner_box = PlannerGroupBox(ctx=self.ctx, store=self.store, parent=self)
        root.addWidget(self._planner_box)

        root.addWidget(_hline())

        # --- sides / paths ---
        self._side_tabbar = _TabBarWithChecks(self)
        root.addWidget(self._side_tabbar)

        self._paths_host = QWidget(self)
        self._paths_host_l = QVBoxLayout(self._paths_host)
        self._paths_host_l.setContentsMargins(0, 0, 0, 0)
        self._paths_host_l.setSpacing(6)
        root.addWidget(self._paths_host, 1)

        # --- info ---
        gb_info = QGroupBox("Info", self)
        gb_info_l = QVBoxLayout(gb_info)
        gb_info_l.setContentsMargins(8, 8, 8, 8)
        self._txt_info = QTextEdit(gb_info)
        self._txt_info.setReadOnly(True)
        gb_info_l.addWidget(self._txt_info)
        root.addWidget(gb_info)

        _set_policy(self._paths_host, v=QSizePolicy.Policy.Expanding)

        if self._side_tabbar is not None:
            self._side_tabbar.currentChanged.connect(self._on_side_tab_changed)

    # ---------------- meta ----------------

    def set_meta(self, *, name: str = "", desc: str = "") -> None:
        if self.e_name is not None:
            self.e_name.setText(name or "")
        if self.e_desc is not None:
            self.e_desc.setText(desc or "")

    # ---------------- internal handlers ----------------

    def _current_ctx_key(self) -> str:
        tool = self.sel_tool.currentText().strip() if (self.sel_tool and self.sel_tool.currentIndex() >= 0) else ""
        sub = self.sel_substrate.currentText().strip() if (self.sel_substrate and self.sel_substrate.currentIndex() >= 0) else ""
        mnt = self.sel_mount.currentText().strip() if (self.sel_mount and self.sel_mount.currentIndex() >= 0) else ""
        return f"{tool}|{sub}|{mnt}"

    def _on_side_tab_changed(self, idx: int) -> None:
        # show only the selected editor
        for i, (side, ed) in enumerate(self._side_editors.items()):
            ed.setVisible(i == idx)

    # ---------------- UI <-> Model ----------------

    def apply_model_to_ui(self, model: Recipe, rec_def: Dict[str, Any]) -> None:
        # fill tool/substrate/mount combos based on rec_def, then select values from model
        tools = [str(x) for x in (rec_def.get("tools") or [])]
        subs = [str(x) for x in (rec_def.get("substrates") or [])]
        mounts = [str(x) for x in (rec_def.get("substrate_mounts") or [])]

        def _fill(cb: Optional[QComboBox], values: List[str], current: Optional[str]) -> None:
            if cb is None:
                return
            cb.blockSignals(True)
            cb.clear()
            for v in values:
                cb.addItem(v)
            if current and current in values:
                cb.setCurrentText(current)
            cb.blockSignals(False)

        _fill(self.sel_tool, tools, getattr(model, "tool", None))
        _fill(self.sel_substrate, subs, getattr(model, "substrate", None))
        _fill(self.sel_mount, mounts, getattr(model, "substrate_mount", None))

        # planner
        if self._planner_box is not None:
            self._planner_box.apply_model_to_ui(model)

        # sides
        sides = list((rec_def.get("sides") or ["top"]))
        self._rebuild_side_editors(sides, model)

        # info
        if self._txt_info is not None:
            self._txt_info.setPlainText(
                f"recipe_id: {model.id}\n"
                f"tool: {getattr(model, 'tool', '')}\n"
                f"substrate: {getattr(model, 'substrate', '')}\n"
                f"mount: {getattr(model, 'substrate_mount', '')}\n"
                f"sides: {', '.join(sides)}\n"
            )

    def _rebuild_side_editors(self, sides: List[str], model: Recipe) -> None:
        # clear previous editors
        for ed in self._side_editors.values():
            try:
                ed.setParent(None)
                ed.deleteLater()
            except Exception:
                pass
        self._side_editors.clear()

        if self._side_tabbar is not None:
            self._side_tabbar.blockSignals(True)
            self._side_tabbar.clear()
            self._side_tabbar.blockSignals(False)

        # create new editors
        for side in sides:
            if self._side_tabbar is not None:
                self._side_tabbar.addTabWithCheck(side, side, checked=True)

            ed = SidePathEditor(ctx=self.ctx, store=self.store, side_name=side, parent=self._paths_host)
            ed.apply_model_to_ui(model)
            ed.setVisible(False)
            self._paths_host_l.addWidget(ed)
            self._side_editors[side] = ed

        # show first
        if self._side_tabbar is not None and self._side_tabbar.count() > 0:
            self._side_tabbar.setCurrentIndex(0)
            # update visibility
            self._on_side_tab_changed(0)

    def apply_ui_to_model(self, model: Recipe) -> None:
        # combos -> model
        if self.sel_tool is not None and self.sel_tool.currentIndex() >= 0:
            model.tool = self.sel_tool.currentText().strip()
        if self.sel_substrate is not None and self.sel_substrate.currentIndex() >= 0:
            model.substrate = self.sel_substrate.currentText().strip()
        if self.sel_mount is not None and self.sel_mount.currentIndex() >= 0:
            model.substrate_mount = self.sel_mount.currentText().strip()

        # desc only (name/id is SSoT from recipes[..].id)
        if self.e_desc is not None:
            model.description = self.e_desc.text().strip()

        # planner
        if self._planner_box is not None:
            self._planner_box.apply_ui_to_model(model)

        # side editors -> paths_by_side
        for side, ed in self._side_editors.items():
            ed.apply_ui_to_model(model)
