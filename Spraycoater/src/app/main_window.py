#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
import logging

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QGuiApplication
from PyQt6.QtWidgets import QMainWindow, QTabWidget, QVBoxLayout

from pyvistaqt import QtInteractor

from app.tabs.process.process_tab import ProcessTab
from app.tabs.recipe.recipe_tab import RecipeTab
from app.tabs.robot.robot_tab import ServiceRobotTab
from app.tabs.signals_tab import ServiceSignalsTab
from app.tabs.syringe_tab import SyringeTab
from app.tabs.system.system_tab import SystemTab
from app.model.recipe.recipe_store import RecipeStore

# PLC kommt aus src/plc
from plc.plc_client import PlcClientBase

_LOG = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    def __init__(self, *, ctx, bridge, plc: PlcClientBase | None = None, parent=None):
        super().__init__(parent)
        if ctx is None:
            raise RuntimeError("AppContext ist None – Startup fehlgeschlagen?")
        self.ctx = ctx
        self.bridge = bridge
        self.plc: PlcClientBase | None = plc

        self.setWindowTitle("SprayCoater UI")

        # zentraler RecipeStore
        self.store = RecipeStore.from_ctx(self.ctx)

        # persistenter PyVista-Interactor (Preview), im MainWindow gehostet
        self.previewPlot = QtInteractor(self)
        self.previewPlot.setFocusPolicy(Qt.FocusPolicy.ClickFocus)

        tabs = QTabWidget(self)

        # 1) Process
        self.processTab = ProcessTab(ctx=self.ctx, bridge=self.bridge, plc=self.plc)
        tabs.addTab(self.processTab, "Process")

        # 2) Recipe
        self.recipeTab = RecipeTab(
            ctx=self.ctx,
            store=self.store,
            bridge=self.bridge,
            attach_preview_widget=self.attach_preview_widget,
        )
        tabs.addTab(self.recipeTab, "Recipe")

        # 3) Service – Robot (alter ServiceTab-Inhalt)
        self.serviceRobotTab = ServiceRobotTab(
            ctx=self.ctx,
            store=self.store,
            bridge=self.bridge,
        )
        tabs.addTab(self.serviceRobotTab, "Robot")

        # 4) Service – SPS-Signale
        self.serviceSignalsTab = ServiceSignalsTab(
            ctx=self.ctx,
            store=self.store,
            bridge=self.bridge,
            plc=self.plc,
        )
        tabs.addTab(self.serviceSignalsTab, "Signale")

        # 5) Service – Syringe / Dispenser
        self.syringeTab = SyringeTab(
            ctx=self.ctx,
            store=self.store,
            bridge=self.bridge,
            plc=self.plc,
        )
        tabs.addTab(self.syringeTab, "Syringe")

        # 6) System
        self.systemTab = SystemTab(ctx=self.ctx, bridge=self.bridge)
        tabs.addTab(self.systemTab, "System")

        self.setCentralWidget(tabs)

        self._centered_once = False
        QTimer.singleShot(0, self.center_on_primary)

    # ------------------------------------------------------------------
    # Preview-Host einhängen (vom RecipeTab aufgerufen)
    # ------------------------------------------------------------------
    def attach_preview_widget(self, host_widget):
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

    # ------------------------------------------------------------------
    # Window-Handling
    # ------------------------------------------------------------------
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
        # ROS-Bridge sauber runterfahren
        try:
            if self.bridge and getattr(self.bridge, "is_connected", False):
                self.bridge.shutdown()
        except Exception:
            pass

        # PLC sauber trennen
        try:
            if self.plc is not None and self.plc.is_connected:
                self.plc.disconnect()
        except Exception:
            pass

        # Bringup ggf. stoppen
        try:
            from ros.ros_launcher import BRINGUP_RUNNING, shutdown_bringup
            if BRINGUP_RUNNING():
                shutdown_bringup()
        except Exception:
            pass

        super().closeEvent(event)
