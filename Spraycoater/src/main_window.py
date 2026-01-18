#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File: src/app/main_window.py

from __future__ import annotations

import logging
from typing import Optional, Any

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QGuiApplication, QCloseEvent
from PyQt6.QtWidgets import QMainWindow, QTabWidget, QVBoxLayout, QSizePolicy  # <-- FIX: add QSizePolicy

from pyvistaqt import QtInteractor

from tabs.process.process_tab import ProcessTab
from tabs.recipe.recipe_tab import RecipeTab
from tabs.robot.robot_tab import ServiceRobotTab
from tabs.plc_tab import PlcTab
from tabs.system_tab import SystemTab

from plc.plc_client import PlcClientBase
from ros.bridge.ros_bridge import RosBridge

_LOG = logging.getLogger("main_window")


class MainWindow(QMainWindow):
    def __init__(
        self,
        *,
        ctx,
        ros: Optional[RosBridge],
        plc: PlcClientBase | None = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        if ctx is None:
            raise RuntimeError("AppContext ist None – Startup fehlgeschlagen?")

        self.ctx = ctx
        self.ros: Optional[RosBridge] = ros
        self.plc: PlcClientBase | None = plc

        self.setWindowTitle("SprayCoater UI")

        # ============================================================
        # NEW SSoT:
        #   ctx.store : RecipeStore
        #   ctx.repo  : RecipeRepo
        # ============================================================
        self.store: Any = getattr(self.ctx, "store", None)
        self.repo: Any = getattr(self.ctx, "repo", None)

        if self.store is None:
            raise RuntimeError("ctx.store fehlt (RecipeStore). load_startup() liefert ctx.store.")
        if self.repo is None:
            raise RuntimeError("ctx.repo fehlt (RecipeRepo). load_startup() liefert ctx.repo.")

        # Shared PyVista Preview (ein Interactor für die ganze App)
        self.previewPlot = QtInteractor(self)
        self.previewPlot.setFocusPolicy(Qt.FocusPolicy.ClickFocus)

        tabs = QTabWidget(self)

        # 1) Process
        self.processTab = ProcessTab(
            ctx=ctx,
            repo=ctx.repo,
            ros=ros,
            plc=plc,
            parent=self,
        )
        tabs.addTab(self.processTab, "Process")

        # 2) Recipe
        self.recipeTab = RecipeTab(
            ctx=self.ctx,
            store=self.store,
            repo=self.repo,
            attach_preview_widget=self.attach_preview_widget,
        )
        tabs.addTab(self.recipeTab, "Recipe")

        # 3) Robot
        self.serviceRobotTab = ServiceRobotTab(
            ctx=self.ctx,
            store=self.store,
            ros=self.ros,
        )
        tabs.addTab(self.serviceRobotTab, "Robot")

        # 4) PLC
        self.plcTab = PlcTab(
            ctx=self.ctx,
            store=self.store,
            ros=self.ros,
            plc=self.plc,
        )
        tabs.addTab(self.plcTab, "Plc")

        # 5) System
        self.systemTab = SystemTab(ctx=self.ctx)
        tabs.addTab(self.systemTab, "System")

        self.setCentralWidget(tabs)

        self._centered_once = False
        QTimer.singleShot(0, self.center_on_primary)

    # ============================================================
    # PyVista / QtInteractor attach
    # ============================================================

    def attach_preview_widget(self, host_widget) -> None:
        try:
            ly = host_widget.layout()
            if ly is None:
                ly = QVBoxLayout(host_widget)
                ly.setContentsMargins(0, 0, 0, 0)
                ly.setSpacing(0)
            else:
                ly.setContentsMargins(0, 0, 0, 0)
                ly.setSpacing(0)

            # remove any placeholders already inside the host
            while ly.count() > 0:
                item = ly.takeAt(0)
                w = item.widget()
                if w is not None:
                    try:
                        w.setParent(None)
                    except Exception:
                        pass

            self.previewPlot.setParent(host_widget)
            self.previewPlot.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

            # stretch=1 => takes all remaining space
            ly.addWidget(self.previewPlot, 1)

            # Walk up and inform Tab3D (it provides set_interactor)
            panel = host_widget
            while panel is not None and not hasattr(panel, "set_interactor"):
                panel = panel.parent()

            if panel is not None and hasattr(panel, "set_interactor"):
                try:
                    panel.set_interactor(self.previewPlot)
                except Exception:
                    _LOG.exception("panel.set_interactor(previewPlot) failed")

            self.previewPlot.setEnabled(True)
            self.previewPlot.show()
            self.previewPlot.update()
            if hasattr(self.previewPlot, "render"):
                self.previewPlot.render()
        except Exception:
            _LOG.exception("Attach preview widget failed")

    # ============================================================
    # Window positioning
    # ============================================================

    def showEvent(self, event) -> None:
        super().showEvent(event)
        if not self._centered_once:
            self.center_on_primary()
            self._centered_once = True

    def center_on_primary(self) -> None:
        try:
            screen = QGuiApplication.primaryScreen()
            if not screen:
                return
            geo = screen.availableGeometry()
            frame = self.frameGeometry()
            frame.moveCenter(geo.center())
            self.move(frame.topLeft())
        except Exception:
            _LOG.exception("center_on_primary failed")

    # ============================================================
    # Shutdown
    # ============================================================

    def closeEvent(self, event: QCloseEvent) -> None:
        try:
            if self.ros is not None:
                self.ros.stop()
        except Exception:
            _LOG.exception("RosBridge stop failed")

        try:
            if self.plc is not None:
                is_conn = getattr(self.plc, "is_connected", False)
                if callable(is_conn):
                    is_conn = bool(is_conn())
                if bool(is_conn):
                    self.plc.disconnect()
        except Exception:
            _LOG.exception("PLC disconnect failed")

        try:
            from ros.ros_launcher import BRINGUP_RUNNING, shutdown_bringup
            if callable(BRINGUP_RUNNING) and BRINGUP_RUNNING():
                shutdown_bringup()
        except Exception:
            _LOG.exception("bringup shutdown failed")

        super().closeEvent(event)
