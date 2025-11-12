# -*- coding: utf-8 -*-
from __future__ import annotations
import logging

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QGuiApplication
from PyQt6.QtWidgets import QMainWindow, QTabWidget, QVBoxLayout

from pyvistaqt import QtInteractor

from app.tabs.process.process_tab import ProcessTab
from app.tabs.recipe.recipe_tab import RecipeTab
from app.tabs.service.service_tab import ServiceTab
from app.tabs.system.system_tab import SystemTab
from app.model.recipe.recipe_store import RecipeStore

_LOG = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        if ctx is None:
            raise RuntimeError("AppContext ist None – Startup fehlgeschlagen?")
        self.ctx = ctx
        self.bridge = bridge
        self.setWindowTitle("SprayCoater UI")

        # 🔹 Zentralen RecipeStore einmal erstellen und weiterreichen
        self.store = RecipeStore.from_ctx(self.ctx)

        # Persistenter Preview-Interactor (lebt im MainWindow)
        self.previewPlot = QtInteractor(self)
        self.previewPlot.setFocusPolicy(Qt.FocusPolicy.ClickFocus)

        # Tabs
        tabs = QTabWidget(self)
        tabs.addTab(ProcessTab(ctx=self.ctx, bridge=self.bridge), "Process")

        # RecipeTab bekommt Store + attach_preview
        self.recipeTab = RecipeTab(   # Referenz behalten
            ctx=self.ctx,
            store=self.store,
            bridge=self.bridge,
            attach_preview_widget=self.attach_preview_widget,
        )
        tabs.addTab(self.recipeTab, "Recipe")

        # ServiceTab bekommt denselben Store
        tabs.addTab(ServiceTab(ctx=self.ctx, store=self.store, bridge=self.bridge), "Service")
        tabs.addTab(SystemTab(ctx=self.ctx, bridge=self.bridge), "System")
        self.setCentralWidget(tabs)

        # Nach Layout/Show einmalig zentrieren
        self._centered_once = False
        QTimer.singleShot(0, self.center_on_primary)

    # Preview-Host einhängen (vom RecipeTab aufgerufen)
    def attach_preview_widget(self, host_widget):
        try:
            ly = host_widget.layout()
            if ly is None:
                ly = QVBoxLayout(host_widget)
                ly.setContentsMargins(0, 0, 0, 0)

            # Interactor einhängen
            self.previewPlot.setParent(host_widget)
            try:
                ly.addWidget(self.previewPlot)
            except Exception:
                pass

            # Panel-Objekt robust finden (Elternkette hochlaufen)
            panel = host_widget
            while panel is not None and not hasattr(panel, "set_interactor"):
                panel = panel.parent()

            if panel is not None and hasattr(panel, "set_interactor"):
                panel.set_interactor(self.previewPlot)  # -> emit interactorReady

            # Erstes Rendern anstoßen
            self.previewPlot.setEnabled(True)
            self.previewPlot.show()
            self.previewPlot.update()
            if hasattr(self.previewPlot, "render"):
                self.previewPlot.render()
        except Exception:
            _LOG.exception("Attach preview widget failed")

    def showEvent(self, event):
        super().showEvent(event)
        # Nur beim ersten Anzeigen zentrieren (falls Größe erst dann feststeht)
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
