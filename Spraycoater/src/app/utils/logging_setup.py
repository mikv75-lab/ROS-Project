# -*- coding: utf-8 -*-
# app/utils/logging_setup.py

from __future__ import annotations

import os
import logging
from logging.handlers import RotatingFileHandler
from typing import Optional

DEFAULT_MAX_BYTES = 1 * 1024 * 1024  # 1 MB
DEFAULT_BACKUP_COUNT = 5

_INITIALIZED = False
_LOG_DIR: Optional[str] = None
_CONSOLE_INFO: Optional[logging.Handler] = None
_CONSOLE_ERR: Optional[logging.Handler] = None


# ---------------------------------------------------------------------------
# interne Helfer
# ---------------------------------------------------------------------------

def _ensure_log_dir() -> None:
    global _LOG_DIR
    if not _LOG_DIR:
        raise RuntimeError("LOG_DIR ist noch nicht gesetzt. Erst init_logging(log_dir) aufrufen.")
    os.makedirs(_LOG_DIR, exist_ok=True)


def _make_brief_formatter() -> logging.Formatter:
    return logging.Formatter("%(levelname).1s %(name)s: %(message)s")


def _make_default_formatter() -> logging.Formatter:
    return logging.Formatter(
        "%(asctime)s %(levelname)s [%(process)d] %(name)s:%(lineno)d - %(message)s",
        "%Y-%m-%d %H:%M:%S",
    )


def _derive_file_name(logger_name: str) -> str:
    """
    Leitet einen sinnvollen Log-Dateinamen aus dem Loggernamen ab.

    Beispiele:
      "app.tabs.process.run_statemachine" -> "process.log"
      "app.tabs.recipe.saver"            -> "recipe.log"
      "ros.ros_launcher"                 -> "ros_launcher.log"
      "ros.launch"                       -> "launch.log"
      "irgendwas.custom.logger"          -> "logger.log"
    """
    parts = logger_name.split(".")

    # app.tabs.<tab>.[...]
    if len(parts) >= 3 and parts[0] == "app" and parts[1] == "tabs":
        return f"{parts[2]}.log"

    # ros.<something>
    if len(parts) >= 2 and parts[0] == "ros":
        return f"{parts[1]}.log"

    # fallback: letzter Teil
    return f"{parts[-1]}.log"


# ---------------------------------------------------------------------------
# öffentlich: init_logging + add_file_logger
# ---------------------------------------------------------------------------

def init_logging(log_dir: str) -> None:
    """
    Basis-Logging initialisieren:

      - Root: INFO, Konsole (stdout + stderr)
      - stderr-Logger: ERROR → stderr.log + Konsole (stderr)
      - registriert logging.add_file_logger(...) als Convenience-Funktion

    Muss genau EINMAL (früh) aufgerufen werden, z.B. im Startup.
    """
    global _INITIALIZED, _LOG_DIR, _CONSOLE_INFO, _CONSOLE_ERR

    if _INITIALIZED:
        return

    if not log_dir:
        raise ValueError("log_dir ist leer.")
    _LOG_DIR = os.path.abspath(log_dir)
    _ensure_log_dir()

    brief_fmt = _make_brief_formatter()
    default_fmt = _make_default_formatter()

    # Konsole
    console_info = logging.StreamHandler()
    console_info.setLevel(logging.DEBUG)
    console_info.setFormatter(brief_fmt)

    console_err = logging.StreamHandler()
    console_err.setLevel(logging.ERROR)
    console_err.setFormatter(default_fmt)

    _CONSOLE_INFO = console_info
    _CONSOLE_ERR = console_err

    # Root-Logger
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    root.handlers.clear()
    root.addHandler(console_info)
    root.addHandler(console_err)

    # stderr-Filehandler
    stderr_path = os.path.join(_LOG_DIR, "stderr.log")
    stderr_handler = RotatingFileHandler(
        stderr_path,
        maxBytes=DEFAULT_MAX_BYTES,
        backupCount=DEFAULT_BACKUP_COUNT,
        encoding="utf-8",
    )
    stderr_handler.setLevel(logging.ERROR)
    stderr_handler.setFormatter(default_fmt)

    stderr_logger = logging.getLogger("stderr")
    stderr_logger.setLevel(logging.ERROR)
    stderr_logger.handlers.clear()
    stderr_logger.addHandler(stderr_handler)
    stderr_logger.addHandler(console_err)
    stderr_logger.propagate = False

    # logging.add_file_logger am logging-Modul registrieren
    if not hasattr(logging, "add_file_logger"):
        setattr(logging, "add_file_logger", add_file_logger)  # type: ignore[attr-defined]

    _INITIALIZED = True
    logging.getLogger("app").info(
        "Basis-Logging konfiguriert (LOG_DIR=%s)", _LOG_DIR
    )


def add_file_logger(
    logger_name: str,
    file_name: Optional[str] = None,
    level: int = logging.DEBUG,
) -> logging.Logger:
    """
    Fügt einem Logger einen RotatingFileHandler hinzu.

    Default-Usage (so wie du willst):

        import logging
        logging.add_file_logger("app.tabs.process.run_statemachine")

    → Level = DEBUG  
    → Dateiname wird automatisch abgeleitet (z.B. "process.log")

    Optional:

        logging.add_file_logger("app.tabs.process.run_statemachine",
                                file_name="process_debug.log",
                                level=logging.INFO)
    """
    global _LOG_DIR, _CONSOLE_INFO, _CONSOLE_ERR

    if not _INITIALIZED:
        raise RuntimeError("init_logging(log_dir) wurde noch nicht aufgerufen.")

    if not logger_name:
        raise ValueError("logger_name ist leer.")

    _ensure_log_dir()
    logger = logging.getLogger(logger_name)
    logger.setLevel(level)

    if file_name is None:
        file_name = _derive_file_name(logger_name)

    target_path = os.path.join(_LOG_DIR, file_name)

    # Falls schon ein identischer Filehandler hängt, nichts tun
    for h in logger.handlers:
        if isinstance(h, RotatingFileHandler) and os.path.abspath(getattr(h, "baseFilename", "")) == os.path.abspath(target_path):
            return logger

    fh = RotatingFileHandler(
        target_path,
        maxBytes=DEFAULT_MAX_BYTES,
        backupCount=DEFAULT_BACKUP_COUNT,
        encoding="utf-8",
    )
    fh.setLevel(level)
    fh.setFormatter(_make_default_formatter())

    logger.addHandler(fh)

    # Optional: Konsolenhandler wie Root anhängen
    if _CONSOLE_INFO is not None:
        logger.addHandler(_CONSOLE_INFO)
    if _CONSOLE_ERR is not None:
        logger.addHandler(_CONSOLE_ERR)

    logger.propagate = False
    logger.debug(
        "File-Logger initialisiert (file=%s, level=%s)",
        target_path,
        logging.getLevelName(level),
    )
    return logger
