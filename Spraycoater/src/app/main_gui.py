# -*- coding: utf-8 -*-
"""
Main GUI for the Spraycoater app.

- Zentrale Plot-Verwaltung im MainWindow via BackgroundPlotter (PyVistaQt)
- RecipeTab/PreviewPanel senden nur Signale; alle add_mesh()/view_*
  Methoden leben hier.
"""

import os
import sys
import logging

# ---- Qt/PyVista-Umgebung vor Imports konsistent setzen ----
os.environ.setdefault("QT_API", "PyQT6")
os.environ.setdefault("PYVISTA_QT_API", "pyqt6")
os.environ.setdefault("QT_X11_NO_MITSHM", "1")  # stabiler auf XLaunch/X11
# Optional, falls kein GL-Treiber:  os.environ.setdefault("QT_OPENGL", "software")
# Optional: reine SW-GL:             os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")

# Matplotlib nie interaktiv
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib
matplotlib.use("Agg", force=True)

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QSplashScreen, QMessageBox,
    QDockWidget, QWidget, QVBoxLayout
)

# VTK Logging in Datei umlenken (hilfreich zum Debuggen)
import vtk
from vtkmodules.vtkCommonCore import vtkFileOutputWindow, vtkOutputWindow
vtk.vtkObject.GlobalWarningDisplayOn()

import pyvista as pv
from pyvistaqt import BackgroundPlotter

# --- Projektpfade (wie gehabt) ---
HERE         = os.path.abspath(os.path.dirname(__file__))           # .../src/app
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))      # .../
RES_ROOT     = os.path.join(PROJECT_ROOT, "resource")

def resource_path(*parts: str) -> str:
    return os.path.join(RES_ROOT, *parts)

# ---- VTK-Logdatei ----
LOG_DIR = os.path.join(PROJECT_ROOT, "data", "logs")
os.makedirs(LOG_DIR, exist_ok=True)
_vtk_log = os.path.join(LOG_DIR, "vtk.log")
_fow = vtkFileOutputWindow()
_fow.SetFileName(_vtk_log)
vtkOutputWindow.SetInstance(_fow)
logging.getLogger(__name__).info("VTK log -> %s", _vtk_log)

# ---- PyVista globale Defaults (robust im UI) ----
pv.OFF_SCREEN = False
pv.global_theme.smooth_shading = False
pv.global_theme.multi_samples = 0
pv.global_theme.depth_peeling.enabled = False

# ---- App-Module (Tabs & Startup) ----
from app.tabs.process.process_tab import ProcessTab
from app.tabs.recipe.recipe_tab import RecipeTab
from app.tabs.service.service_tab import ServiceTab
from app.tabs.system.system_tab import SystemTab
from app.startup_fsm import StartupMachine


def _startup_path_strict() -> str:
    cfg = resource_path("config", "startup.yaml")
    if not os.path.exists(cfg):
        raise FileNotFoundError(f"startup.yaml nicht gefunden: {cfg}")
    return cfg


def _make_splash():
    pm_path = resource_path("images", "splash.png")
    pm = QPixmap(pm_path) if os.path.exists(pm_path) else QPixmap(640, 360)
    if pm.isNull():
        pm = QPixmap(640, 360)
        pm.fill(Qt.black)
    splash = QSplashScreen(pm)
    splash.setEnabled(False)
    splash.showMessage("Lade…", Qt.AlignHCenter | Qt.AlignBottom, Qt.white)
    splash.show()
    return splash


class MainWindow(QMainWindow):
    """Zentrales Fenster + persistenter BackgroundPlotter + Plot-API."""

    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        if ctx is None:
            raise RuntimeError("AppContext ist None – Startup fehlgeschlagen?")
        self.ctx = ctx
        self.bridge = bridge
        self.setWindowTitle("SprayCoater UI")
        self.resize(1280, 800)

        # ==== Persistenter Preview-Plotter (eigenes Fenster) ====
        # Hinweis: BackgroundPlotter öffnet ein eigenes Fenster (stabil),
        #          das unabhängig von Tabs/Widget-Lebenszyklen lebt.
        self.plotter = BackgroundPlotter(show=True, off_screen=False, title="Preview")
        try:
            # Fenstergröße des Plotter-Fensters (falls unterstützt)
            self.plotter.app_window.resize(900, 700)
        except Exception:
            pass

        # Smoke-Test
        try:
            self.plotter.add_mesh(pv.Sphere(radius=5.0), color="lightgray", opacity=0.9, show_edges=True)
            self.plotter.view_isometric()
            self.plotter.reset_camera()
            self.plotter.render()
        except Exception:
            pass

        # (Optional) Dock als Platzhalter, falls du später einen eingebetteten Interactor nutzen willst
        self.previewDock = QDockWidget("Preview (Status)", self)
        self.previewDock.setObjectName("PreviewDock")
        self.previewDock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        self.addDockWidget(Qt.RightDockWidgetArea, self.previewDock)
        host = QWidget(self.previewDock)
        host.setLayout(QVBoxLayout())
        host.layout().setContentsMargins(8, 8, 8, 8)
        host.layout().addWidget(QWidget())  # Placeholder
        self.previewDock.setWidget(host)

        # ==== Tabs ====
        tabs = QTabWidget(self)
        tabs.addTab(ProcessTab(ctx=self.ctx, bridge=self.bridge), "Process")
        # WICHTIG: preview_api=self an RecipeTab durchreichen
        tabs.addTab(RecipeTab(ctx=self.ctx, bridge=self.bridge, preview_api=self, parent=self), "Recipe")
        tabs.addTab(ServiceTab(ctx=self.ctx, bridge=self.bridge), "Service")
        tabs.addTab(SystemTab(ctx=self.ctx, bridge=self.bridge), "System")
        self.setCentralWidget(tabs)

    # ---------- ZENTRALE PLOT-API (nur hier VTK/PyVista aufrufen) ----------
    def preview_clear(self):
        try:
            self.plotter.clear()
            self.plotter.render()
        except Exception:
            logging.getLogger(__name__).exception("preview_clear failed")

    def preview_view_iso(self):
        try:
            self.plotter.view_isometric()
            self.plotter.reset_camera()
            self.plotter.render()
        except Exception:
            logging.getLogger(__name__).exception("preview_view_iso failed")

    def preview_view_top(self):
        try:
            self.plotter.view_xy()
            self.plotter.reset_camera()
            self.plotter.render()
        except Exception:
            logging.getLogger(__name__).exception("preview_view_top failed")

    def preview_view_front(self):
        try:
            self.plotter.view_yz()
            self.plotter.reset_camera()
            self.plotter.render()
        except Exception:
            logging.getLogger(__name__).exception("preview_view_front failed")

    def preview_view_left(self):
        try:
            self.plotter.view_xz()
            self.plotter.reset_camera()
            self.plotter.render()
        except Exception:
            logging.getLogger(__name__).exception("preview_view_left failed")

    def preview_view_right(self):
        try:
            self.plotter.view_xz()
            try:
                self.plotter.camera.azimuth(180)
            except Exception:
                pass
            self.plotter.reset_camera()
            self.plotter.render()
        except Exception:
            logging.getLogger(__name__).exception("preview_view_right failed")

    def preview_view_back(self):
        try:
            self.plotter.view_yz()
            try:
                self.plotter.camera.azimuth(180)
            except Exception:
                pass
            self.plotter.reset_camera()
            self.plotter.render()
        except Exception:
            logging.getLogger(__name__).exception("preview_view_back failed")

    def preview_render_model(self, mount_key: str, substrate_key: str):
        """Lädt Mount/Substrat und rendert beides."""
        from app.tabs.recipe.coating_preview_panel.mesh_utils import (
            load_mount_mesh_from_key,
            load_substrate_mesh_from_key,
            place_substrate_on_mount,
        )
        self.preview_clear()
        try:
            mount_mesh = load_mount_mesh_from_key(self.ctx, mount_key)
            self.plotter.add_mesh(mount_mesh, color="lightgray", opacity=0.3, lighting=False, reset_camera=False)
        except Exception:
            logging.getLogger(__name__).exception("Mount-Mesh Fehler")

        try:
            sub_mesh = load_substrate_mesh_from_key(self.ctx, substrate_key)
            sub_mesh = place_substrate_on_mount(self.ctx, sub_mesh, mount_key=mount_key)
            self.plotter.add_mesh(sub_mesh, color="#3498db", opacity=0.95, lighting=False, reset_camera=False)
        except Exception:
            logging.getLogger(__name__).exception("Substrat-Mesh Fehler")

        self.preview_view_iso()

    # ---------- Window lifecycle ----------
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


def main():
    # Pflicht: Runtime-Verzeichnis für Qt
    if "XDG_RUNTIME_DIR" not in os.environ:
        tmp_run = f"/tmp/runtime-{os.getuid()}"
        os.makedirs(tmp_run, exist_ok=True)
        os.environ["XDG_RUNTIME_DIR"] = tmp_run

    # FastDDS SHM in Docker häufig problematisch
    os.environ.setdefault("FASTDDS_SHM_TRANSPORT_DISABLE", "1")

    app = QApplication(sys.argv)

    # Splash & Startup-FSM
    splash = _make_splash()
    app.processEvents()

    fsm = StartupMachine(
        startup_yaml_path=_startup_path_strict(),
        logging_yaml_path=resource_path("config", "logging.yaml"),
        abort_on_error=False,
    )

    splash_msg = lambda t: splash.showMessage(t, Qt.AlignHCenter | Qt.AlignBottom, Qt.white)
    fsm.progress.connect(lambda p: splash_msg(f"{p} …"))
    fsm.warning.connect(lambda w: splash_msg(f"⚠ {w}"))
    fsm.error.connect(lambda e: splash_msg(f"✖ {e}"))

    def _on_ready(ctx, bridge):
        if ctx is None:
            QMessageBox.critical(None, "Startup fehlgeschlagen", "Kein gültiger AppContext. Siehe Log.")
            return
        win = MainWindow(ctx=ctx, bridge=bridge)
        splash.finish(win)
        win.show()

    fsm.ready.connect(_on_ready)
    QTimer.singleShot(0, fsm.start)

    rc = app.exec()
    sys.exit(rc)


if __name__ == "__main__":
    main()
