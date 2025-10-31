# -*- coding: utf-8 -*-
# --- EARLY CRASH/DUMP SETUP (MUSS *GANZ OBEN* STEHEN) ---
import os, sys, signal, traceback, atexit, logging
import faulthandler

# Qt/PyVista: stabile Defaults VOR Imports
os.environ["QT_API"] = "PyQT6"
os.environ.setdefault("PYVISTA_QT_API", "pyqt6")
os.environ.setdefault("QT_X11_NO_MITSHM", "1")
# Optional: reine Software-GL
# os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
# os.environ.setdefault("QT_OPENGL", "software")

HERE         = os.path.abspath(os.path.dirname(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SRC_ROOT     = os.path.join(PROJECT_ROOT, "src")
RES_ROOT     = os.path.join(PROJECT_ROOT, "resource")

for p in (SRC_ROOT, RES_ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

LOG_DIR = os.path.join(PROJECT_ROOT, "data", "logs")
os.makedirs(LOG_DIR, exist_ok=True)
CRASH_PATH = os.path.join(LOG_DIR, "crash.dump")

try:
    if os.path.exists(CRASH_PATH):
        os.remove(CRASH_PATH)
except Exception:
    pass

try:
    _CRASH_FH = open(CRASH_PATH, "w", buffering=1, encoding="utf-8")
    faulthandler.enable(file=_CRASH_FH, all_threads=True)
    for _sig in (signal.SIGSEGV, signal.SIGABRT, signal.SIGBUS, signal.SIGILL, signal.SIGFPE):
        try:
            faulthandler.register(_sig, file=_CRASH_FH, all_threads=True)
        except Exception:
            pass
except Exception:
    _CRASH_FH = None

def _excepthook(exc_type, exc, tb):
    try:
        logging.critical("UNCAUGHT EXCEPTION", exc_info=(exc_type, exc, tb))
    except Exception:
        pass
    try:
        if _CRASH_FH:
            traceback.print_exception(exc_type, exc, tb, file=_CRASH_FH)
            _CRASH_FH.flush()
    except Exception:
        pass
sys.excepthook = _excepthook

def _on_exit_flush():
    try:
        if _CRASH_FH:
            _CRASH_FH.flush()
    except Exception:
        pass
atexit.register(_on_exit_flush)
# --- ENDE EARLY CRASH/DUMP SETUP ---

# Matplotlib nie interaktiv
os.environ["MPLBACKEND"] = "Agg"
import matplotlib
matplotlib.use("Agg", force=True)
try:
    import matplotlib.pyplot as plt
    plt.ioff()
except Exception:
    pass

import vtk
vtk.vtkObject.GlobalWarningDisplayOn()

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QSplashScreen, QMessageBox,
    QDockWidget, QWidget, QVBoxLayout
)
QApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)

# Qt runtime dir
if "XDG_RUNTIME_DIR" not in os.environ:
    tmp_run = f"/tmp/runtime-{os.getuid()}"
    os.makedirs(tmp_run, exist_ok=True)
    os.environ["XDG_RUNTIME_DIR"] = tmp_run

# FastDDS SHM off
os.environ.setdefault("FASTDDS_SHM_TRANSPORT_DISABLE", "1")

from vtkmodules.vtkCommonCore import vtkFileOutputWindow, vtkOutputWindow
vtk.vtkObject.GlobalWarningDisplayOn()
vtk_log_dir = os.path.join(PROJECT_ROOT, "data", "logs")
os.makedirs(vtk_log_dir, exist_ok=True)
vtk_log = os.path.join(vtk_log_dir, "vtk.log")
fow = vtkFileOutputWindow(); fow.SetFileName(vtk_log)
vtkOutputWindow.SetInstance(fow)
logging.getLogger(__name__).info("VTK log -> %s", vtk_log)

# PyVistaQt Interactor (global, persistent)
from pyvistaqt import QtInteractor
import pyvista as pv
pv.OFF_SCREEN = False
pv.global_theme.smooth_shading = False
pv.global_theme.multi_samples = 0
pv.global_theme.depth_peeling.enabled = False

from app.tabs.process.process_tab import ProcessTab
from app.tabs.recipe.recipe_tab import RecipeTab
from app.tabs.service.service_tab import ServiceTab
from app.tabs.system.system_tab import SystemTab
from app.startup_fsm import StartupMachine


def resource_path(*parts: str) -> str:
    return os.path.join(RES_ROOT, *parts)


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
    if pm.width() == 640 and pm.height() == 360:
        pm.fill(Qt.black)
    splash = QSplashScreen(pm)
    splash.setEnabled(False)
    splash.showMessage("Lade…", Qt.AlignHCenter | Qt.AlignBottom, Qt.white)
    splash.show()
    return splash


class MainWindow(QMainWindow):
    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        if ctx is None:
            raise RuntimeError("AppContext ist None – Startup fehlgeschlagen?")
        self.ctx = ctx
        self.bridge = bridge
        self.setWindowTitle("SprayCoater UI")
        self.resize(1280, 800)

        # === Persistenter Preview-Interactor (ohne Parent erzeugen!) ===
        self.previewPlot = QtInteractor(self)
        # Fokus nur bei Klick → frisst keine UI-Events
        self.previewPlot.setFocusPolicy(Qt.FocusPolicy.ClickFocus)

        # === Dock mit Host, optional nutzbar (z. B. wenn Tab nicht sichtbar) ===
        self.previewDock = QDockWidget("Preview", self)
        self.previewDock.setObjectName("PreviewDock")
        self.previewDock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        self.addDockWidget(Qt.RightDockWidgetArea, self.previewDock)

        self.previewHostDock = QWidget(self.previewDock)
        dockLayout = QVBoxLayout(self.previewHostDock)
        dockLayout.setContentsMargins(0, 0, 0, 0)
        dockLayout.setSpacing(0)
        self.previewDock.setWidget(self.previewHostDock)

        # Standardmäßig im Dock hosten (bis der Tab seinen Host anbietet)
        self.attach_preview_widget(self.previewHostDock)

        # === Tabs ===
        tabs = QTabWidget(self)
        tabs.addTab(ProcessTab(ctx=self.ctx, bridge=self.bridge), "Process")
        tabs.addTab(RecipeTab(
            ctx=self.ctx,
            bridge=self.bridge,
            attach_preview_widget=self.attach_preview_widget,
            preview_api=self  # dieses Objekt stellt die Preview-API
        ), "Recipe")
        tabs.addTab(ServiceTab(ctx=self.ctx, bridge=self.bridge), "Service")
        tabs.addTab(SystemTab(ctx=self.ctx,  bridge=self.bridge), "System")
        self.setCentralWidget(tabs)

    # ---------- Preview-Host einhängen ----------
    def attach_preview_widget(self, host_widget):
        """QtInteractor in gegebenen Host einsetzen (ohne Initialize/Start!)."""
        try:
            ly = host_widget.layout()
            if ly is None:
                ly = QVBoxLayout(host_widget)
                ly.setContentsMargins(0, 0, 0, 0)
                ly.setSpacing(0)

            self.previewPlot.setParent(host_widget)
            if self.previewPlot not in getattr(ly, 'children', lambda: [])():
                ly.addWidget(self.previewPlot)

            # Keine interactor.Initialize()/Start() → sonst blockiert VTK Qt-Events!
            self.previewPlot.setEnabled(True)
            self.previewPlot.show()
            self.previewPlot.update()
            if hasattr(self.previewPlot, "render"):
                self.previewPlot.render()
        except Exception:
            logging.exception("Attach preview widget failed")

    # ---------- Preview-API, wird an RecipeTab gereicht ----------
    def preview_clear(self):
        try:
            self.previewPlot.clear()
            self.previewPlot.render()
        except Exception:
            logging.exception("preview_clear failed")

    def preview_add_mesh(self, mesh, **kwargs):
        try:
            fn = getattr(mesh, "is_all_triangles", None)
            if callable(fn) and not fn():
                mesh = mesh.triangulate()
        except Exception:
            pass
        try:
            # Ein Render erst am Ende vom Update-Zyklus
            self.previewPlot.add_mesh(mesh, reset_camera=False, render=False, **kwargs)
        except Exception:
            logging.exception("preview_add_mesh failed")

    def preview_view_iso(self):
        try:
            self.previewPlot.view_isometric()
        except Exception:
            pass

    def preview_view_top(self):
        try:
            self.previewPlot.view_xy()
        except Exception:
            pass

    def preview_view_front(self):
        try:
            self.previewPlot.view_yz()
        except Exception:
            pass

    def preview_view_left(self):
        try:
            self.previewPlot.view_xz()
        except Exception:
            pass

    def preview_view_right(self):
        try:
            self.previewPlot.view_xz()
            try:
                self.previewPlot.camera.azimuth(180)
            except Exception:
                pass
        except Exception:
            pass

    def preview_view_back(self):
        try:
            self.previewPlot.view_yz()
            try:
                self.previewPlot.camera.azimuth(180)
            except Exception:
                pass
        except Exception:
            pass

    def preview_render(self, reset_camera=True):
        try:
            if reset_camera:
                self.previewPlot.reset_camera()
            self.previewPlot.render()
        except Exception:
            logging.exception("preview_render failed")

    # ---------- Close ----------
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


def _nonblocking_logging_shutdown():
    try:
        logging.shutdown()
    except Exception:
        pass


def main():
    app = QApplication(sys.argv)
    app.aboutToQuit.connect(lambda: QTimer.singleShot(0, _nonblocking_logging_shutdown))

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

    rc = 0
    try:
        rc = app.exec_()
    finally:
        try:
            if _CRASH_FH:
                _CRASH_FH.flush()
                _CRASH_FH.close()
        except Exception:
            pass
    sys.exit(rc)


if __name__ == "__main__":
    main()
