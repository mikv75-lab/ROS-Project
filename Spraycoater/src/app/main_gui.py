# Spraycoater/src/app/main_gui.py
# -*- coding: utf-8 -*-
"""
Main GUI für den Spraycoater.

- striktes Laden von startup.yaml (resource/config/startup.yaml)
- Logging via resource/config/logging.yaml (Rotating Files + Console)
- Sammeln von Python-Warnings in warnings.log
- Abfangen von unbehandelten Exceptions in stderr.log + faulthandler dump
- ROS Bringup (ros2 launch ...) inkl. ENV für node-Logging
- UIBridge wird erstellt und den Tabs durchgereicht (connect nach Bringup)
"""

import os
import sys
import argparse
import logging
import platform

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QMessageBox

# --- nur 'src' und 'resource' in sys.path aufnehmen ---
HERE         = os.path.abspath(os.path.dirname(__file__))              # .../Spraycoater/src/app
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))         # .../Spraycoater
SRC_ROOT     = os.path.join(PROJECT_ROOT, "src")
RES_ROOT     = os.path.join(PROJECT_ROOT, "resource")
for p in (SRC_ROOT, RES_ROOT):
    if p not in sys.path:
        sys.path.insert(0, p)

# Tabs direkt aus Moduldateien
from app.tabs.process.process_tab import ProcessTab
from app.tabs.recipe.recipe_tab import RecipeTab
from app.tabs.service.service_tab import ServiceTab
from app.tabs.system.system_tab import SystemTab

# Strict Loader (Rezepte nur auf Abruf validieren)
from config.startup import load_startup, validate_recipes_for_load

# ROS Launcher API (mit make_bringup_cmd)
from ros.ros_launcher import (
    ensure_clean_graph_then_launch,
    shutdown_bringup,
    BRINGUP_RUNNING,
    make_bringup_cmd,
)

# UI<->ROS Bridge
from ros.ui_bridge import get_ui_bridge, UIBridge  # Bridge wird erstellt und später verbunden

# Logging/Warnungen-Setup
from app.utils.logging_setup import configure_logging_from_yaml
from app.utils.warnings_setup import enable_all_warnings


def resource_path(*parts: str) -> str:
    """Pfade relativ zu Spraycoater/resource bauen."""
    return os.path.join(RES_ROOT, *parts)


def _startup_path_strict() -> str:
    """IMMER dieselbe startup.yaml unter resource/config verwenden."""
    cfg = resource_path("config", "startup.yaml")
    if not os.path.exists(cfg):
        raise FileNotFoundError(f"startup.yaml nicht gefunden: {cfg}")
    return cfg


class MainWindow(QMainWindow):
    def __init__(self, *, validate_on_start: bool = False, parent=None):
        super().__init__(parent)
        self.setWindowTitle("SprayCoater UI")
        self.resize(1280, 800)

        # 1) Strict load (keine Fallbacks)
        try:
            self.ctx = load_startup(_startup_path_strict())
        except Exception as e:
            QMessageBox.critical(self, "Startup-Fehler", str(e))
            sys.exit(2)

        # 2) Logging initialisieren (benötigt self.ctx.paths.log_dir)
        logging_yaml = resource_path("config", "logging.yaml")
        configure_logging_from_yaml(
            logging_yaml_path=logging_yaml,
            log_dir=self.ctx.paths.log_dir,
        )

        # 3) Warnings -> zentrales warnings.log routen
        enable_all_warnings(qt=True, pywarnings="always")

        # 4) STDERR/Crash-Dumps aktivieren
        self._install_exception_logging()

        # 5) Startup-Banner in alle Logs schreiben
        self._log_startup_banner()

        # 6) Bridge erzeugen (noch nicht verbinden) und Tabs erstellen
        self.bridge: UIBridge = get_ui_bridge(self.ctx)

        tabs = QTabWidget()
        # Alle Tabs bekommen ctx + bridge
        tabs.addTab(ProcessTab(ctx=self.ctx, bridge=self.bridge), "Process")
        tabs.addTab(RecipeTab(ctx=self.ctx,  bridge=self.bridge), "Recipe")
        tabs.addTab(ServiceTab(ctx=self.ctx, bridge=self.bridge), "Service")
        tabs.addTab(SystemTab(ctx=self.ctx,  bridge=self.bridge), "System")
        self.setCentralWidget(tabs)

        # 7) ROS ggf. aus dem MainWindow heraus starten (nachdem die UI steht)
        QTimer.singleShot(0, self._maybe_start_ros)
        # 8) Bridge etwas später verbinden (Graph sollte stehen)
        QTimer.singleShot(1500, self._connect_bridge)

        # Optional: Rezepte prüfen (syntaktisch)
        if validate_on_start:
            try:
                errs = validate_recipes_for_load(self.ctx.recipes_yaml)
            except Exception as e:
                logging.getLogger("app").exception("Validierungsfehler: %s", e)
                QMessageBox.critical(self, "Validierungsfehler", str(e))
                sys.exit(2)
            if errs:
                msg = "• " + "\n• ".join(errs)
                logging.getLogger("app").error("Fehlerhafte Rezepte:\n%s", msg)
                QMessageBox.critical(self, "Fehlerhafte Rezepte", msg)
                sys.exit(2)

    # --- ROS-Start ist Teil des MainWindow ---
    def _maybe_start_ros(self):
        try:
            if getattr(self.ctx, "ros", None) and self.ctx.ros.launch_ros:
                # Kommando + Launch-Args aus Helper
                cmd, extra = make_bringup_cmd(self.ctx.ros.sim_robot)

                # ENV für Nodes/Launch (optional; unschädlich wenn ungenutzt)
                env = {
                    "SPRAY_LOG_DIR": self.ctx.paths.log_dir,
                    "SPRAY_LOG_CFG": resource_path("config", "logging.yaml"),
                }
                ensure_clean_graph_then_launch(
                    cmd,
                    extra_launch_args=extra,
                    log_path=self.ctx.paths.bringup_log,  # Log-Ziel aus startup.yaml
                    env=env,
                    # ros_setup/ws_setup können später optional aus YAML kommen;
                    # ros_launcher wertet auch ROS_SETUP/WS_SETUP aus der ENV aus.
                )
                if not BRINGUP_RUNNING():
                    logging.getLogger("ros").error(
                        "Bringup-Prozess ist nicht gestartet. Log: %s",
                        self.ctx.paths.bringup_log,
                    )
                    QMessageBox.critical(
                        self,
                        "ROS Bringup fehlgeschlagen",
                        f"Bringup-Prozess ist nicht gestartet.\nLog: {self.ctx.paths.bringup_log}",
                    )
            else:
                logging.getLogger("ros").info("ROS-Launch in startup.yaml deaktiviert.")
        except Exception as e:
            logging.getLogger("ros").exception("ROS-Startfehler: %s", e)
            QMessageBox.critical(self, "ROS-Startfehler", str(e))

    def _connect_bridge(self):
        """Versucht, die UIBridge mit rclpy zu verbinden (non-fatal bei Fehlern)."""
        try:
            self.bridge.connect()
            logging.getLogger("ros").info("UIBridge verbunden (node=%s).", self.bridge.node_name)
        except Exception as e:
            # Nicht fatal: UI bleibt benutzbar, ROS-Features ausgrauen
            logging.getLogger("ros").warning("UIBridge konnte nicht verbinden: %s", e)

    def _install_exception_logging(self):
        """
        Loggt unbehandelte Exceptions (main + Threads) nach 'stderr' -> stderr.log.
        Zusätzlich: unraisable-Hook und faulthandler-Dump.
        """
        import threading
        logger = logging.getLogger("stderr")

        # Uncaught exceptions im Main-Thread
        def _excepthook(exc_type, exc, tb):
            logger.critical("UNCAUGHT EXCEPTION", exc_info=(exc_type, exc, tb))
        sys.excepthook = _excepthook

        # Uncaught exceptions in Threads (Py3.8+)
        def _threadhook(args: threading.ExceptHookArgs):
            logger.critical(
                "UNCAUGHT THREAD EXCEPTION (thread=%s)",
                getattr(args.thread, "name", "?"),
                exc_info=(args.exc_type, args.exc_value, args.exc_traceback),
            )
        threading.excepthook = _threadhook

        # Unraisable (z.B. __del__-Fehler)
        def _unraisablehook(unraisable):
            logger.error(
                "UNRAISABLE in %r",
                getattr(unraisable, "object", None),
                exc_info=(unraisable.exc_type, unraisable.exc_value, unraisable.exc_traceback),
            )
        sys.unraisablehook = _unraisablehook

        # Crash-Dumps bei harten Fehlern (Segfault, SIGABRT, etc.)
        try:
            import faulthandler
            crash_path = os.path.join(self.ctx.paths.log_dir, "crash.dump")
            self._faulthandler_file = open(crash_path, "a", encoding="utf-8")
            faulthandler.enable(self._faulthandler_file)
            logger.info("faulthandler enabled -> %s", crash_path)
        except Exception as e:
            logger.warning("faulthandler enable failed: %s", e)

    def _log_startup_banner(self):
        """
        Schreibt beim Appstart eine Zeile in alle definierten Logfiles,
        damit jedes File pro Start mindestens einen Eintrag hat.
        """
        logger_names = [
            "app",
            "app.tabs.process",
            "app.tabs.recipe",
            "app.tabs.service",
            "app.tabs.system",
            "ros",
            "ros.ros_launcher",
            "ros.kill_all_ros",
            "ros.launch",
            "py.warnings",
            "qt",
        ]
        banner = (
            "=== SprayCoater START === "
            f"pid={os.getpid()} "
            f"python={platform.python_version()} "
            f"cwd={os.getcwd()} "
            f"recipe_file={self.ctx.paths.recipe_file} "
            f"recipe_dir={self.ctx.paths.recipe_dir} "
            f"log_dir={self.ctx.paths.log_dir} "
            f"bringup_log={self.ctx.paths.bringup_log} "
            f"ros.launch_ros={self.ctx.ros.launch_ros} "
            f"ros.sim_robot={self.ctx.ros.sim_robot}"
        )
        for name in logger_names:
            logging.getLogger(name).info(banner)
        logging.getLogger().info(banner)  # Root (Konsole)

    def closeEvent(self, event):
        # Bridge sauber beenden
        try:
            if self.bridge and self.bridge.is_connected:
                self.bridge.shutdown()
        except Exception:
            pass
        # ROS-Bringup stoppen
        try:
            if BRINGUP_RUNNING():
                shutdown_bringup()
        except Exception:
            pass
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(prog="SprayCoater UI", add_help=True)
    parser.add_argument(
        "--validate-on-start",
        action="store_true",
        help="Rezepte beim Start validieren (standardmäßig AUS).",
    )
    args = parser.parse_args()

    app = QApplication(sys.argv)
    win = MainWindow(validate_on_start=args.validate_on_start)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
