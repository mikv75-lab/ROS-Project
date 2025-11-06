# src/app/main_window.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QMainWindow, QTabWidget, QVBoxLayout, QWidget

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

        self.recipeTab = RecipeTab(
            ctx=self.ctx,
            bridge=self.bridge,
            attach_preview_widget=self.attach_preview_widget,
        )
        tabs.addTab(self.recipeTab, "Recipe")

        tabs.addTab(ServiceTab(ctx=self.ctx, bridge=self.bridge), "Service")
        tabs.addTab(SystemTab(ctx=self.ctx, bridge=self.bridge), "System")
        self.setCentralWidget(tabs)

        _LOG.info("MainWindow initialized (ctx=%s, bridge=%s)", bool(self.ctx), bool(self.bridge))

    # Preview-Host einhängen (vom RecipeTab aufgerufen)
    def attach_preview_widget(self, host_widget: QWidget):
        try:
            if host_widget is None:
                _LOG.warning("attach_preview_widget: host_widget is None")
                return

            # Sicherstellen, dass der Interactor keinen anderen Parent mehr hält
            try:
                if self.previewPlot.parent() is not host_widget:
                    self.previewPlot.setParent(None)
            except Exception:
                pass

            # Layout sicherstellen
            ly = host_widget.layout()
            if ly is None:
                ly = QVBoxLayout(host_widget)
                ly.setContentsMargins(0, 0, 0, 0)
            elif isinstance(ly, QVBoxLayout):
                # Margins defensiv setzen, falls nicht bereits 0
                ly.setContentsMargins(0, 0, 0, 0)

            # Widget einhängen
            self.previewPlot.setParent(host_widget)
            try:
                ly.addWidget(self.previewPlot)
            except Exception:
                # falls bereits enthalten, ignorieren
                pass

            self.previewPlot.setEnabled(True)
            self.previewPlot.show()
            self.previewPlot.update()

            # Nur rendern, wenn Methode existiert
            render_fn = getattr(self.previewPlot, "render", None)
            if callable(render_fn):
                render_fn()

        except Exception:
            _LOG.exception("Attach preview widget failed")

    def closeEvent(self, event):
        # 1) Preview sauber schließen, um VTK/Qt-Abstürze zu vermeiden
        try:
            if self.previewPlot is not None:
                try:
                    self.previewPlot.hide()
                except Exception:
                    pass
                try:
                    # pyvistaqt kümmert sich intern um den VTK-Interactor
                    self.previewPlot.close()
                except Exception:
                    pass
        except Exception:
            _LOG.exception("Error while closing previewPlot")

        # 2) Bridge herunterfahren
        try:
            if self.bridge and getattr(self.bridge, "is_connected", False):
                # bevorzugt 'shutdown', fallback 'disconnect'
                if hasattr(self.bridge, "shutdown"):
                    self.bridge.shutdown()
                elif hasattr(self.bridge, "disconnect"):
                    self.bridge.disconnect()
        except Exception:
            _LOG.exception("Error while shutting down bridge")

        # 3) Bringup stoppen
        try:
            from ros.ros_launcher import BRINGUP_RUNNING, shutdown_bringup
            if BRINGUP_RUNNING():
                shutdown_bringup()
        except Exception:
            _LOG.exception("Error while shutting down bringup")

        super().closeEvent(event)
