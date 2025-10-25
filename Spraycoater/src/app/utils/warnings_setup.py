# -*- coding: utf-8 -*-
import warnings
import logging

def _install_qt_message_handler():
    """
    Leitet Qt-Logs (PyQt5) in den Logger 'qt'.
    Wird nur installiert, wenn PyQt5 verfügbar ist.
    """
    try:
        from PyQt5 import QtCore
    except Exception:
        return  # Qt nicht verfügbar / noch nicht geladen

    qtlog = logging.getLogger("qt")

    # Mapping Qt -> logging
    level_map = {
        QtCore.QtDebugMsg:    logging.DEBUG,
        getattr(QtCore, "QtInfoMsg", None): logging.INFO,
        QtCore.QtWarningMsg:  logging.WARNING,
        QtCore.QtCriticalMsg: logging.ERROR,
        QtCore.QtFatalMsg:    logging.CRITICAL,
    }

    def handler(mode, context, message):
        lvl = level_map.get(mode, logging.INFO)
        # Kontext optional anhängen
        where = ""
        try:
            if context and context.file and context.line:
                where = f"{context.file}:{context.line} "
        except Exception:
            pass
        qtlog.log(lvl, "%s%s", where, message)

    # Installieren (überschreibt evtl. existierende Handler)
    QtCore.qInstallMessageHandler(handler)

def enable_all_warnings(*, qt: bool = True, pywarnings: str = "default") -> None:
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
