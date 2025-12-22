#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File: src/app/main_window.py
#
# Fixes:
# - __init__ akzeptiert konsistent `ros: Optional[RosBridge]`
# - Tabs bekommen `ros=self.ros` (kann None sein)
# - closeEvent stoppt Bridge + PLC + optional Bringup clean

from __future__ import annotations

import logging
from typing import Optional

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QGuiApplication
from PyQt6.QtWidgets import QMainWindow, QTabWidget, QVBoxLayout

from pyvistaqt import QtInteractor

from app.tabs.process.process_tab import ProcessTab
from app.tabs.recipe.recipe_tab import RecipeTab
from app.tabs.robot.robot_tab import ServiceRobotTab
from app.tabs.plc_tab import PlcTab
from app.tabs.system_tab import SystemTab
from app.model.recipe.recipe_store import RecipeStore

from plc.plc_client import PlcClientBase

from ros.bridge.ros_bridge import RosBridge

_LOG = logging.getLogger("app.main_window")


class MainWindow(QMainWindow):
    def __init__(self, *, ctx, ros: Optional[RosBridge], plc: PlcClientBase | None = None, parent=None):
        super().__init__(parent)
        if ctx is None:
            raise RuntimeError("AppContext ist None – Startup fehlgeschlagen?")

        self.ctx = ctx
        self.ros: Optional[RosBridge] = ros  # darf None sein
        self.plc: PlcClientBase | None = plc

        self.setWindowTitle("SprayCoater UI")

        self.store = RecipeStore.from_ctx(self.ctx)

        # Shared PyVista Preview
        self.previewPlot = QtInteractor(self)
        self.previewPlot.setFocusPolicy(Qt.FocusPolicy.ClickFocus)

        tabs = QTabWidget(self)

        # 1) Process
        self.processTab = ProcessTab(ctx=self.ctx, ros=self.ros, plc=self.plc)
        tabs.addTab(self.processTab, "Process")

        # 2) Recipe
        self.recipeTab = RecipeTab(
            ctx=self.ctx,
            store=self.store,
            ros=self.ros,
            attach_preview_widget=self.attach_preview_widget,
        )
        tabs.addTab(self.recipeTab, "Recipe")

        # 3) Robot
        self.serviceRobotTab = ServiceRobotTab(
            ctx=self.ctx,
            store=self.store,
            ros=self.ros,  # kann None sein -> Tab handled das
        )
        tabs.addTab(self.serviceRobotTab, "Robot")

        # 4) Plc
        self.serviceSignalsTab = PlcTab(
            ctx=self.ctx,
            store=self.store,
            ros=self.ros,
            plc=self.plc,
        )
        tabs.addTab(self.serviceSignalsTab, "Plc")

        # 5) System
        self.systemTab = SystemTab(ctx=self.ctx)
        tabs.addTab(self.systemTab, "System")

        self.setCentralWidget(tabs)

        self._centered_once = False
        QTimer.singleShot(0, self.center_on_primary)

    def attach_preview_widget(self, host_widget):
        """
        RecipeTab gibt uns ein Host-Widget; wir hängen den gemeinsamen QtInteractor dort ein.
        """
        try:
            ly = host_widget.layout()
            if ly is None:
                ly = QVBoxLayout(host_widget)
                ly.setContentsMargins(0, 0, 0, 0)

            self.previewPlot.setParent(host_widget)
            try:
                ly.addWidget(self.previewPlot)
            except Exception:
                pass

            # optional: Parent-Panel mit set_interactor(...) updaten
            panel = host_widget
            while panel is not None and not hasattr(panel, "set_interactor"):
                panel = panel.parent()

            if panel is not None and hasattr(panel, "set_interactor"):
                panel.set_interactor(self.previewPlot)

            self.previewPlot.setEnabled(True)
            self.previewPlot.show()
            self.previewPlot.update()
            if hasattr(self.previewPlot, "render"):
                self.previewPlot.render()
        except Exception:
            _LOG.exception("Attach preview widget failed")

    def showEvent(self, event):
        super().showEvent(event)
        if not self._centered_once:
            self.center_on_primary()
            self._centered_once = True

    def center_on_primary(self):
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

    def closeEvent(self, event):
        # 1) stop ROS bridge
        try:
            if self.ros is not None:
                try:
                    self.ros.stop()
                except Exception:
                    _LOG.exception("RosBridge stop failed")
        except Exception:
            pass

        # 2) PLC disconnect
        try:
            if self.plc is not None and getattr(self.plc, "is_connected", False):
                self.plc.disconnect()
        except Exception:
            _LOG.exception("PLC disconnect failed")

        # 3) bringup shutdown (optional)
        try:
            from ros.ros_launcher import BRINGUP_RUNNING, shutdown_bringup
            if BRINGUP_RUNNING():
                shutdown_bringup()
        except Exception:
            _LOG.exception("bringup shutdown failed")

        super().closeEvent(event)
