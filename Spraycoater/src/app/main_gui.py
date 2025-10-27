# -*- coding: utf-8 -*-
# --- EARLY CRASH/DUMP SETUP (MUSS *GANZ OBEN* STEHEN) ---
import os, sys, signal, traceback, atexit, logging
import faulthandler

HERE         = os.path.abspath(os.path.dirname(__file__))       # .../src/app
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))  # .../
SRC_ROOT     = os.path.join(PROJECT_ROOT, "src")
RES_ROOT     = os.path.join(PROJECT_ROOT, "resource")

# Pfade für Imports & Logs
for p in (SRC_ROOT, RES_ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)
LOG_DIR = os.path.join(PROJECT_ROOT, "data", "logs")
os.makedirs(LOG_DIR, exist_ok=True)
CRASH_PATH = os.path.join(LOG_DIR, "crash.dump")

# Ungepufferte/zeilenweise Log-Datei für faulthandler
try:
    _CRASH_FH = open(CRASH_PATH, "w", buffering=1, encoding="utf-8")
    # 1) Python-Tracebacks aller Threads
    faulthandler.enable(file=_CRASH_FH, all_threads=True)
    # 2) Native Crashes (SIGSEGV, SIGABRT, …) + TERM/INT
    for _sig in (
        signal.SIGSEGV, signal.SIGABRT, signal.SIGBUS, signal.SIGILL, signal.SIGFPE,
        signal.SIGTERM, signal.SIGINT,
    ):
        try:
            faulthandler.register(_sig, file=_CRASH_FH, all_threads=True)
        except Exception:
            pass
    # 3) Optionaler Watchdog: regelmäßig Thread-Dump bei Hängern
    try:
        faulthandler.dump_traceback_later(15.0, repeat=True, file=_CRASH_FH)
    except Exception:
        pass
except Exception:
    _CRASH_FH = None  # notfalls ohne faulthandler weiterlaufen

# 4) Ungefangene Python-Exceptions -> Log + crash.dump + Thread-Dump
def _excepthook(exc_type, exc, tb):
    try:
        logging.critical("UNCAUGHT EXCEPTION", exc_info=(exc_type, exc, tb))
    except Exception:
        pass
    try:
        if _CRASH_FH:
            traceback.print_exception(exc_type, exc, tb, file=_CRASH_FH)
            # kompletter Thread-Dump hilft bei Qt/Matplotlib-Problemen
            faulthandler.dump_traceback(file=_CRASH_FH, all_threads=True)
            _CRASH_FH.flush()
    except Exception:
        pass
    # nicht re-raisen, damit Qt sauber schließen kann
sys.excepthook = _excepthook

# 5) Beim Exit alles flushen + letzter Thread-Dump (auch bei „sauberem“ Exit)
def _on_exit_flush():
    try:
        if _CRASH_FH:
            faulthandler.dump_traceback(file=_CRASH_FH, all_threads=True)
            _CRASH_FH.flush()
    except Exception:
        pass
    try:
        logging.shutdown()
    except Exception:
        pass
    try:
        if _CRASH_FH:
            _CRASH_FH.close()
    except Exception:
        pass
atexit.register(_on_exit_flush)
# --- ENDE EARLY CRASH/DUMP SETUP ---

# Qt-/Umgebungs-Prep
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QSplashScreen, QMessageBox

# Qt-Runtime dir fix (verhindert Warnungen & seltene Hänger in Containern)
if "XDG_RUNTIME_DIR" not in os.environ:
    tmp_run = f"/tmp/runtime-{os.getuid()}"
    os.makedirs(tmp_run, exist_ok=True)
    os.environ["XDG_RUNTIME_DIR"] = tmp_run

# FastDDS SHM sicherheitshalber deaktivieren (verhindert Hänger in Containern)
os.environ.setdefault("FASTDDS_SHM_TRANSPORT_DISABLE", "1")

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

        tabs = QTabWidget()
        tabs.addTab(ProcessTab(ctx=self.ctx, bridge=self.bridge), "Process")
        tabs.addTab(RecipeTab(ctx=self.ctx,  bridge=self.bridge), "Recipe")
        tabs.addTab(ServiceTab(ctx=self.ctx, bridge=self.bridge), "Service")
        tabs.addTab(SystemTab(ctx=self.ctx,  bridge=self.bridge), "System")
        self.setCentralWidget(tabs)

    def closeEvent(self, event):
        try:
            if self.bridge and getattr(self.bridge, "is_connected", False):
                self.bridge.shutdown()
        except Exception:
            pass
        from ros.ros_launcher import BRINGUP_RUNNING, shutdown_bringup
        try:
            if BRINGUP_RUNNING():
                shutdown_bringup()
        except Exception:
            pass
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)

    # bei Qt-Exit noch einen Thread-Dump erzwingen (falls Qt „leise“ schließt)
    if _CRASH_FH:
        app.aboutToQuit.connect(lambda: faulthandler.dump_traceback(file=_CRASH_FH, all_threads=True))

    splash = _make_splash()
    app.processEvents()

    fsm = StartupMachine(
        startup_yaml_path=_startup_path_strict(),
        logging_yaml_path=resource_path("config", "logging.yaml"),
        abort_on_error=True,
    )

    splash_msg = lambda t: splash.showMessage(t, Qt.AlignHCenter | Qt.AlignBottom, Qt.white)
    fsm.progress.connect(lambda p: splash_msg(f"{p} …"))
    fsm.warning.connect(lambda w: splash_msg(f"⚠ {w}"))
    fsm.error.connect(lambda e: splash_msg(f"✖ {e}"))

    def _on_ready(ctx, bridge):
        if ctx is None:
            QMessageBox.critical(None, "Startup fehlgeschlagen", "Kein gültiger AppContext. Siehe Log.")
            sys.exit(2)
        win = MainWindow(ctx=ctx, bridge=bridge)
        splash.finish(win)
        win.show()

    fsm.ready.connect(_on_ready)
    fsm.start()

    # Qt-Eventloop robust beenden (auch bei Exceptions wird geflusht)
    rc = 0
    try:
        rc = app.exec_()
    finally:
        try:
            logging.shutdown()
        except Exception:
            pass
        try:
            if _CRASH_FH:
                faulthandler.dump_traceback(file=_CRASH_FH, all_threads=True)
                _CRASH_FH.flush()
        except Exception:
            pass
    sys.exit(rc)


if __name__ == "__main__":
    main()
