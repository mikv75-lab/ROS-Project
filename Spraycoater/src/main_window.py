#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File: src/app/main_window.py
#
# Main UI Window:
# - bekommt ctx + ros (optional) + plc (optional)
# - Tabs bekommen ctx + ros/plc + ggf. recipe-store/repo
# - Shared PyVista/QtInteractor wird ins Recipe-Preview-Panel eingehängt
# - closeEvent fährt ROS-Bridge + PLC + optional Bringup sauber runter

from __future__ import annotations

import logging
from typing import Optional, Any

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QGuiApplication, QCloseEvent
from PyQt6.QtWidgets import QMainWindow, QTabWidget, QVBoxLayout

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
    def __init__(self, *, ctx, ros: Optional[RosBridge], plc: PlcClientBase | None = None, parent=None) -> None:
        super().__init__(parent)
        if ctx is None:
            raise RuntimeError("AppContext ist None – Startup fehlgeschlagen?")

        self.ctx = ctx
        self.ros: Optional[RosBridge] = ros  # darf None sein
        self.plc: PlcClientBase | None = plc

        self.setWindowTitle("SprayCoater UI")

        # ============================================================
        # Recipe Repo/Store: über ctx.content bereitstellen
        # (dein Ziel: Tabs holen sich alles über ctx/content)
        # ============================================================
        self.recipe_repo: Any = None
        try:
            if getattr(self.ctx, "content", None) is not None:
                # lazy getter aus AppContent
                self.recipe_repo = self.ctx.content.recipe_repo()
        except Exception:
            _LOG.exception("RecipeRepo konnte nicht aus ctx.content erzeugt werden")

        # Shared PyVista Preview (ein Interactor für die ganze App)
        self.previewPlot = QtInteractor(self)
        self.previewPlot.setFocusPolicy(Qt.FocusPolicy.ClickFocus)

        tabs = QTabWidget(self)

        # 1) Process
        self.processTab = ProcessTab(ctx=self.ctx, ros=self.ros, plc=self.plc)
        tabs.addTab(self.processTab, "Process")

        # 2) Recipe
        # - store/repo wird (wenn vorhanden) übergeben, ansonsten kann RecipeTab intern über ctx arbeiten
        self.recipeTab = RecipeTab(
            ctx=self.ctx,
            store=self.recipe_repo,  # kann None sein, je nach deiner Repo-Implementierung
            ros=self.ros,
            attach_preview_widget=self.attach_preview_widget,
        )
        tabs.addTab(self.recipeTab, "Recipe")

        # 3) Robot
        self.serviceRobotTab = ServiceRobotTab(
            ctx=self.ctx,
            store=self.recipe_repo,  # kann None sein
            ros=self.ros,            # kann None sein -> Tab muss das handeln
        )
        tabs.addTab(self.serviceRobotTab, "Robot")

        # 4) PLC
        self.plcTab = PlcTab(
            ctx=self.ctx,
            store=self.recipe_repo,  # kann None sein
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
        """
        RecipeTab gibt uns ein Host-Widget; wir hängen den gemeinsamen QtInteractor dort ein.
        host_widget ist z.B. das Widget, das CoatingPreviewPanel.get_pv_host() zurückgibt.
        """
        try:
            ly = host_widget.layout()
            if ly is None:
                ly = QVBoxLayout(host_widget)
                ly.setContentsMargins(0, 0, 0, 0)

            # Interactor in den Host einhängen
            self.previewPlot.setParent(host_widget)
            try:
                ly.addWidget(self.previewPlot)
            except Exception:
                pass

            # Optional: parent-chain nach set_interactor(...) durchsuchen
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
        # 1) stop ROS bridge
        try:
            if self.ros is not None:
                self.ros.stop()
        except Exception:
            _LOG.exception("RosBridge stop failed")

        # 2) PLC disconnect (robust gegenüber property/method)
        try:
            if self.plc is not None:
                is_conn = getattr(self.plc, "is_connected", False)
                if callable(is_conn):
                    is_conn = bool(is_conn())
                if bool(is_conn):
                    self.plc.disconnect()
        except Exception:
            _LOG.exception("PLC disconnect failed")

        # 3) bringup shutdown (optional)
        try:
            from ros.ros_launcher import BRINGUP_RUNNING, shutdown_bringup
            if callable(BRINGUP_RUNNING) and BRINGUP_RUNNING():
                shutdown_bringup()
        except Exception:
            _LOG.exception("bringup shutdown failed")

        super().closeEvent(event)
