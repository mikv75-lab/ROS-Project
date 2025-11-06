# src/app/main_window.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QMainWindow, QTabWidget, QVBoxLayout

from app.tabs.process.process_tab import ProcessTab
from app.tabs.recipe.recipe_tab import RecipeTab
from app.tabs.service.service_tab import ServiceTab
from app.tabs.system.system_tab import SystemTab

from pyvistaqt import QtInteractor

_LOG = logging.getLogger(__name__)

class MainWindow(QMainWindow):
    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        if ctx is None:
            raise RuntimeError("AppContext ist None – Startup fehlgeschlagen?")
        self.ctx = ctx
        self.bridge = bridge
        self.setWindowTitle("SprayCoater UI")
        self.resize(1280, 800)

        # Persistenter Preview-Interactor (lebt im MainWindow)
        self.previewPlot = QtInteractor(self)
        self.previewPlot.setFocusPolicy(Qt.FocusPolicy.ClickFocus)

        # Tabs
        tabs = QTabWidget(self)
        tabs.addTab(ProcessTab(ctx=self.ctx, bridge=self.bridge), "Process")

        self.recipeTab = RecipeTab(   # Referenz behalten
            ctx=self.ctx,
            bridge=self.bridge,
            attach_preview_widget=self.attach_preview_widget,
        )
        tabs.addTab(self.recipeTab, "Recipe")

        tabs.addTab(ServiceTab(ctx=self.ctx, bridge=self.bridge), "Service")
        tabs.addTab(SystemTab(ctx=self.ctx, bridge=self.bridge), "System")
        self.setCentralWidget(tabs)

    # Preview-Host einhängen (vom RecipeTab aufgerufen)
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
            self.previewPlot.setEnabled(True)
            self.previewPlot.show()
            self.previewPlot.update()
            if hasattr(self.previewPlot, "render"):
                self.previewPlot.render()
        except Exception:
            _LOG.exception("Attach preview widget failed")

    def closeEvent(self, event):
        try:
            if self.bridge and getattr(self.bridge, "is_connected", False):
                self.bridge.shutdown()
        except Exception:
            pass
        try:
            from ros.ros_launcher import BRINGUP_RUNNING, shutdown_bringup
            if BRINGUP_RUNNING():
                shutdown_bringup()
        except Exception:
            pass
        super().closeEvent(event)
