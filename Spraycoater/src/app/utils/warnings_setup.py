# -*- coding: utf-8 -*-
import logging
import warnings

def _install_qt_message_handler() -> None:
    """
    Leitet Qt-Logs (PyQt6) in den Logger 'qt'.
    Tut nichts, wenn PyQt6 nicht verfügbar ist.
    """
    try:
        from PyQt6 import QtCore
    except Exception:
        return  # Qt6 nicht verfügbar

    qtlog = logging.getLogger("qt")

    # Qt6-Enums liegen unter QtCore.QtMsgType
    levels = {
        QtCore.QtMsgType.QtDebugMsg:    logging.DEBUG,
        QtCore.QtMsgType.QtInfoMsg:     logging.INFO,
        QtCore.QtMsgType.QtWarningMsg:  logging.WARNING,
        QtCore.QtMsgType.QtCriticalMsg: logging.ERROR,
        QtCore.QtMsgType.QtFatalMsg:    logging.CRITICAL,
    }

    # Qt6-Signatur: handler(msg_type, context, message)
    def handler(msg_type, context, message):
        lvl = levels.get(msg_type, logging.INFO)
        where = ""
        try:
            file = getattr(context, "file", None)
            line = getattr(context, "line", None)
            if file and line:
                where = f"{file}:{line} "
        except Exception:
            pass
        try:
            qtlog.log(lvl, "%s%s", where, str(message))
        except Exception:
            # Fallback ohne Formatierung
            qtlog.log(lvl, str(message))

    # Installiert den globalen Qt-Message-Handler (überschreibt evtl. vorhandene)
    QtCore.qInstallMessageHandler(handler)

def enable_all_warnings(*, qt: bool = True, pywarnings: str = "always") -> None:
    """
    - Aktiviert logging.captureWarnings -> 'py.warnings' Logger.
    - Setzt den Python-Warnfilter (z.B. 'always', 'default', 'ignore' ...).
    - Optional: routed Qt-Meldungen nach 'qt'.
    """
    logging.captureWarnings(True)
    if pywarnings:
        warnings.simplefilter(pywarnings)

    if qt:
        _install_qt_message_handler()

    # kleines Lebenszeichen ins Log
    logging.getLogger("py.warnings").info("warnings capture aktiv (filter=%s)", pywarnings)
    if qt:
        logging.getLogger("qt").info("Qt message handler aktiv")
