# -*- coding: utf-8 -*-
from __future__ import annotations
import os, sys, signal, traceback, atexit, logging
import faulthandler

# ==== Früh: stabile Qt/PyVista-Defaults ====
os.environ["QT_API"] = "PyQT6"
os.environ.setdefault("PYVISTA_QT_API", "pyqt6")
os.environ.setdefault("QT_X11_NO_MITSHM", "1")
os.environ["MPLBACKEND"] = "Agg"

# OpenGL:
# - Software-Rendering erzwingen (kein GLX/ANGLE-Stress mit Xming)
os.environ["QT_OPENGL"] = "software"

# Klassisches X11 im Container
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

HERE         = os.path.abspath(os.path.dirname(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SRC_ROOT     = os.path.join(PROJECT_ROOT, "src")
RES_ROOT     = os.path.join(PROJECT_ROOT, "resource")
for p in (SRC_ROOT, RES_ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

# ==== Crashdump ====
LOG_DIR = os.path.join(PROJECT_ROOT, "data", "logs")
os.makedirs(LOG_DIR, exist_ok=True)
CRASH_PATH = os.path.join(LOG_DIR, "crash.dump")

# Vorhandenen Crashdump (vom letzten Lauf) einmal ausgeben, dann löschen
try:
    if os.path.exists(CRASH_PATH):
        try:
            with open(CRASH_PATH, "r", encoding="utf-8") as _old:
                print("\n=== Previous crash.dump ===", file=sys.stderr)
                print(_old.read(), file=sys.stderr)
                sys.stderr.flush()
        except Exception:
            pass
        try:
            os.remove(CRASH_PATH)
        except Exception:
            pass
except Exception:
    pass

# Neuen Crashdump-Filehandler anlegen
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
    # 1) Immer direkt in die Konsole (STDERR) drucken
    try:
        print("\n=== UNCAUGHT EXCEPTION ===", file=sys.stderr)
        traceback.print_exception(exc_type, exc, tb, file=sys.stderr)
        sys.stderr.flush()
    except Exception:
        pass

    # 2) Zusätzlich über logging-Subsystem
    try:
        logging.critical("UNCAUGHT EXCEPTION", exc_info=(exc_type, exc, tb))
    except Exception:
        pass

    # 3) Und in crash.dump schreiben
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

# ==== Imports, nachdem Backend fixiert ist ====
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
from PyQt6.QtWidgets import QApplication, QSplashScreen, QMessageBox

# Qt runtime dir
if "XDG_RUNTIME_DIR" not in os.environ:
    tmp_run = f"/tmp/runtime-{os.getuid()}"
    os.makedirs(tmp_run, exist_ok=True)
    os.environ["XDG_RUNTIME_DIR"] = tmp_run

# FastDDS SHM off
os.environ.setdefault("FASTDDS_SHM_TRANSPORT_DISABLE", "1")

from vtkmodules.vtkCommonCore import vtkFileOutputWindow, vtkOutputWindow
vtk_log_dir = os.path.join(PROJECT_ROOT, "data", "logs")
os.makedirs(vtk_log_dir, exist_ok=True)
vtk_log = os.path.join(vtk_log_dir, "vtk.log")
fow = vtkFileOutputWindow()
fow.SetFileName(vtk_log)
vtkOutputWindow.SetInstance(fow)
logging.getLogger(__name__).info("VTK log -> %s", vtk_log)

# ==== App-Imports ====
from app.startup_fsm import StartupMachine
from app.main_window import MainWindow   # MainWindow nutzt Splitter (RViz-Placeholder rechts) & PyVista nur im RecipeTab


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


def _nonblocking_logging_shutdown():
    try:
        logging.shutdown()
    except Exception:
        pass


class ExceptionApp(QApplication):
    """
    QApplication-Subklasse, damit Exceptions in Qt-Events
    (Slots, Timer, QThreads, Signals) NICHT still verschluckt werden,
    sondern über _excepthook in Konsole + crash.dump landen.
    """
    def notify(self, receiver, event):
        try:
            return super().notify(receiver, event)
        except Exception:
            _excepthook(*sys.exc_info())
            # False zurückgeben, damit Qt das Event als "nicht handled" sieht
            return False


def main():
    # ExceptionApp statt "normaler" QApplication verwenden
    app = ExceptionApp(sys.argv)
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

    # bleibt bei (ctx, bridge)
    def _on_ready(ctx, bridge):
        if ctx is None:
            QMessageBox.critical(None, "Startup fehlgeschlagen", "Kein gültiger AppContext. Siehe Log.")
            return
        win = MainWindow(ctx=ctx, bridge=bridge)  # zeigt Splitter (RViz Placeholder rechts) + PyVista nur im RecipeTab
        splash.finish(win)
        win.show()

    fsm.ready.connect(_on_ready)
    QTimer.singleShot(0, fsm.start)

    rc = 0
    try:
        rc = app.exec()
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
