#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Entry: main_gui.py (oder dein aktuelles Startscript)

from __future__ import annotations

import os
import sys
import signal
import traceback
import atexit
import logging
import faulthandler

# ==== Früh: stabile Qt/PyVista-Defaults ====
os.environ["QT_API"] = "PyQT6"
os.environ.setdefault("PYVISTA_QT_API", "pyqt6")
os.environ.setdefault("QT_X11_NO_MITSHM", "1")
os.environ["MPLBACKEND"] = "Agg"

# OpenGL:
os.environ.setdefault("QT_OPENGL", "software")
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

# WebEngine in Docker/root (falls du später Foxglove/Browser einbettest)
try:
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
except Exception:
    pass

HERE = os.path.abspath(os.path.dirname(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
SRC_ROOT = os.path.join(PROJECT_ROOT, "src")
RES_ROOT = os.path.join(PROJECT_ROOT, "resource")
for p in (SRC_ROOT, RES_ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

# ==== Logging sehr früh initialisieren ====
LOG_DIR = os.path.join(PROJECT_ROOT, "data", "logs")
os.makedirs(LOG_DIR, exist_ok=True)

try:
    from app.utils.logging_setup import init_logging  # dein Modul
    init_logging(LOG_DIR)
except Exception:
    # Fallback: wenigstens irgendwas auf stderr
    logging.basicConfig(level=logging.INFO, format="%(levelname).1s %(name)s: %(message)s")

# Optional: Warnings + Qt-Message-Handler (dein Modul)
try:
    from app.utils.warnings_setup import enable_all_warnings  # falls du es so abgelegt hast
    enable_all_warnings(qt=False, pywarnings="always")  # Qt-Handler erst nach PyQt6-Import
except Exception:
    pass

_LOG = logging.getLogger("app.entry")

# ==== Crashdump ====
CRASH_PATH = os.path.join(LOG_DIR, "crash.dump")

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
        print("\n=== UNCAUGHT EXCEPTION ===", file=sys.stderr)
        traceback.print_exception(exc_type, exc, tb, file=sys.stderr)
        sys.stderr.flush()
    except Exception:
        pass

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

# ==== Imports, nachdem Backend fixiert ist ====
import matplotlib

matplotlib.use("Agg", force=True)
try:
    import matplotlib.pyplot as plt

    plt.ioff()
except Exception:
    pass

# ROS2 / FastDDS shared memory (Docker)
os.environ.setdefault("FASTDDS_SHM_TRANSPORT_DISABLE", "1")

# XDG runtime (Qt mag das)
if "XDG_RUNTIME_DIR" not in os.environ:
    tmp_run = f"/tmp/runtime-{os.getuid()}" if hasattr(os, "getuid") else "/tmp/runtime-unknown"
    os.makedirs(tmp_run, exist_ok=True)
    os.environ["XDG_RUNTIME_DIR"] = tmp_run

# VTK Logging
try:
    from vtkmodules.vtkCommonCore import vtkFileOutputWindow, vtkOutputWindow

    vtk_log = os.path.join(LOG_DIR, "vtk.log")
    fow = vtkFileOutputWindow()
    fow.SetFileName(vtk_log)
    vtkOutputWindow.SetInstance(fow)
    _LOG.info("VTK log -> %s", vtk_log)
except Exception:
    _LOG.exception("VTK logging setup failed")

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import QApplication, QSplashScreen, QMessageBox

# Jetzt Qt-Message-Handler aktivieren (falls vorhanden)
try:
    from app.utils.warnings_setup import enable_all_warnings

    enable_all_warnings(qt=True, pywarnings="always")
except Exception:
    pass

# ==== App-Imports ====
from app.startup_fsm import StartupMachine
from app.main_window import MainWindow


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
        pm.fill(Qt.GlobalColor.black)
    splash = QSplashScreen(pm)
    splash.setEnabled(False)
    splash.showMessage("Lade…", Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignBottom, Qt.GlobalColor.white)
    splash.show()
    return splash


def _nonblocking_logging_shutdown():
    try:
        logging.shutdown()
    except Exception:
        pass


def _install_signal_handlers(app: QApplication) -> None:
    # Ctrl+C / docker stop sauber beenden
    def _quit(_sig=None, _frame=None):
        try:
            app.quit()
        except Exception:
            pass

    try:
        signal.signal(signal.SIGINT, _quit)
    except Exception:
        pass
    try:
        signal.signal(signal.SIGTERM, _quit)
    except Exception:
        pass


class ExceptionApp(QApplication):
    def notify(self, receiver, event):
        try:
            return super().notify(receiver, event)
        except Exception:
            _excepthook(*sys.exc_info())
            return False


def main():
    app = ExceptionApp(sys.argv)
    _install_signal_handlers(app)

    # ---- Tripwire: wer beendet die App? (aboutToQuit / lastWindowClosed) ----
    QUIT_LOG = os.path.join(LOG_DIR, "quit.log")

    def _append_quit_log(tag: str):
        try:
            with open(QUIT_LOG, "a", encoding="utf-8") as f:
                f.write(f"\n=== {tag} ===\n")
                f.write("".join(traceback.format_stack(limit=80)))
        except Exception:
            pass

    app.aboutToQuit.connect(lambda: _append_quit_log("aboutToQuit"))
    app.lastWindowClosed.connect(lambda: _append_quit_log("lastWindowClosed"))

    # allow SIGINT handling while Qt event loop runs
    try:
        timer = QTimer()
        timer.timeout.connect(lambda: None)
        timer.start(200)
    except Exception:
        pass

    app.aboutToQuit.connect(lambda: QTimer.singleShot(0, _nonblocking_logging_shutdown))

    splash = _make_splash()
    app.processEvents()

    fsm = StartupMachine(
        startup_yaml_path=_startup_path_strict(),
        abort_on_error=False,
    )

    splash_msg = lambda t: splash.showMessage(
        t, Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignBottom, Qt.GlobalColor.white
    )
    fsm.progress.connect(lambda p: splash_msg(f"{p} …"))
    fsm.warning.connect(lambda w: splash_msg(f"⚠ {w}"))
    fsm.error.connect(lambda e: splash_msg(f"✖ {e}"))

    # ready liefert (ctx, bridge_shadow, bridge_live, plc)
    def _on_ready(ctx, bridge_shadow, bridge_live, plc):
        if ctx is None:
            QMessageBox.critical(None, "Startup fehlgeschlagen", "Kein gültiger AppContext. Siehe Log.")
            return

        # Shadow bevorzugt (UI default), dann Live, sonst None (UI läuft trotzdem)
        bridge = bridge_shadow or bridge_live

        if bridge is None:
            splash_msg("⚠ Keine ROS-Bridge verfügbar (shadow/live beide fehlgeschlagen).")

        win = MainWindow(ctx=ctx, bridge=bridge, plc=plc)

        # WICHTIG: Referenz halten, sonst kann Python-Objekt GC'ed werden -> Fenster weg -> App quit
        app._main_window = win  # noqa: SLF001

        # Optional: Logge auch, wenn das Window zerstört wird (hilft bei "plötzlich weg")
        try:
            win.destroyed.connect(lambda: _append_quit_log("MainWindow destroyed"))
        except Exception:
            pass

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
