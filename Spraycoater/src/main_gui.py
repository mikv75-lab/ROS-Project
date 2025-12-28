#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File: src/app/main_gui.py

from __future__ import annotations

import os
import sys
import signal
import traceback
import atexit
import logging
import faulthandler

# ============================================================
# ===============  Qt / PyVista Defaults (EARLY) ==============
# ============================================================

os.environ["QT_API"] = "PyQT6"
os.environ.setdefault("PYVISTA_QT_API", "pyqt6")
os.environ.setdefault("QT_X11_NO_MITSHM", "1")
os.environ["MPLBACKEND"] = "Agg"

os.environ.setdefault("QT_OPENGL", "software")
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

try:
    if hasattr(os, "geteuid") and os.geteuid() == 0:
        os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")
except Exception:
    pass

# ============================================================
# ==================  Project Root Discovery =================
# ============================================================

HERE = os.path.abspath(os.path.dirname(__file__))


def find_project_root(start: str) -> str:
    cur = os.path.abspath(start)
    while True:
        candidate = os.path.join(cur, "resource", "config", "startup.yaml")
        if os.path.exists(candidate):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    raise FileNotFoundError("PROJECT_ROOT nicht gefunden (resource/config/startup.yaml fehlt)")


PROJECT_ROOT = find_project_root(HERE)

# ✅ Variante B: Loader soll relative Pfade (data/..., config/...) relativ zum Projektroot auflösen
os.environ["SC_PROJECT_ROOT"] = PROJECT_ROOT

SRC_ROOT = os.path.join(PROJECT_ROOT, "src")
RES_ROOT = os.path.join(PROJECT_ROOT, "resource")

for p in (SRC_ROOT, RES_ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

# ============================================================
# ======================== Logging ============================
# ============================================================

LOG_DIR = os.path.join(PROJECT_ROOT, "data", "logs")
os.makedirs(LOG_DIR, exist_ok=True)

try:
    from utils.logging_setup import init_logging
    init_logging(LOG_DIR)
except Exception:
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname).1s %(name)s: %(message)s",
    )

_LOG = logging.getLogger("app.entry")

# ============================================================
# ======================== Crashdump =========================
# ============================================================

CRASH_PATH = os.path.join(LOG_DIR, "crash.dump")

try:
    if os.path.exists(CRASH_PATH):
        with open(CRASH_PATH, "r", encoding="utf-8") as f:
            print("\n=== Previous crash.dump ===", file=sys.stderr)
            print(f.read(), file=sys.stderr)
        os.remove(CRASH_PATH)
except Exception:
    pass

try:
    _CRASH_FH = open(CRASH_PATH, "w", buffering=1, encoding="utf-8")
    faulthandler.enable(file=_CRASH_FH, all_threads=True)
    for sig in (signal.SIGSEGV, signal.SIGABRT, signal.SIGBUS, signal.SIGILL, signal.SIGFPE):
        try:
            faulthandler.register(sig, file=_CRASH_FH, all_threads=True)
        except Exception:
            pass
except Exception:
    _CRASH_FH = None


def _excepthook(exc_type, exc, tb):
    traceback.print_exception(exc_type, exc, tb)
    try:
        if _CRASH_FH:
            traceback.print_exception(exc_type, exc, tb, file=_CRASH_FH)
            _CRASH_FH.flush()
    except Exception:
        pass


sys.excepthook = _excepthook


@atexit.register
def _flush_crashlog():
    try:
        if _CRASH_FH:
            _CRASH_FH.flush()
    except Exception:
        pass


# ============================================================
# ======================== Qt Setup ===========================
# ============================================================

from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import QApplication, QSplashScreen, QMessageBox

# ============================================================
# ======================== App Imports =======================
# ============================================================

from startup_fsm import StartupMachine
from main_window import MainWindow


# ============================================================
# ======================== Helpers ===========================
# ============================================================

def resource_path(*parts: str) -> str:
    return os.path.join(RES_ROOT, *parts)


def startup_yaml_path() -> str:
    p = resource_path("config", "startup.yaml")
    if not os.path.exists(p):
        raise FileNotFoundError(f"startup.yaml nicht gefunden: {p}")
    return p


def make_splash() -> QSplashScreen:
    img = resource_path("images", "splash.png")
    pm = QPixmap(img) if os.path.exists(img) else QPixmap(640, 360)
    if pm.isNull():
        pm = QPixmap(640, 360)
        pm.fill(Qt.GlobalColor.black)

    splash = QSplashScreen(pm)
    splash.showMessage(
        "Lade …",
        Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignBottom,
        Qt.GlobalColor.white,
    )
    splash.show()
    return splash


class ExceptionApp(QApplication):
    def notify(self, receiver, event):
        try:
            return super().notify(receiver, event)
        except Exception:
            _excepthook(*sys.exc_info())
            return False


# ============================================================
# ============================ Main ==========================
# ============================================================

def main():
    app = ExceptionApp(sys.argv)

    def _quit(*_):
        app.quit()

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, _quit)
        except Exception:
            pass

    splash = make_splash()
    app.processEvents()

    fsm = StartupMachine(
        startup_yaml_path=startup_yaml_path(),
        abort_on_error=False,
    )

    fsm.progress.connect(
        lambda t: splash.showMessage(
            f"{t} …",
            Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignBottom,
            Qt.GlobalColor.white,
        )
    )

    def on_ready(ctx, bridge_shadow, bridge_live, plc):
        if ctx is None:
            QMessageBox.critical(None, "Startup fehlgeschlagen", "Kein gültiger AppContext")
            return

        ros = bridge_shadow or bridge_live

        win = MainWindow(ctx=ctx, ros=ros, plc=plc)
        app._main_window = win  # GC-Schutz

        splash.finish(win)
        win.show()

    fsm.ready.connect(on_ready)
    QTimer.singleShot(0, fsm.start)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
