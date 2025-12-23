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

    levels = {
        QtCore.QtMsgType.QtDebugMsg:    logging.DEBUG,
        QtCore.QtMsgType.QtInfoMsg:     logging.INFO,
        QtCore.QtMsgType.QtWarningMsg:  logging.WARNING,
        QtCore.QtMsgType.QtCriticalMsg: logging.ERROR,
        QtCore.QtMsgType.QtFatalMsg:    logging.CRITICAL,
    }

    def _safe_str(x) -> str:
        try:
            return "" if x is None else str(x)
        except Exception:
            return ""

    def _flush_all_handlers():
        try:
            root = logging.getLogger()
            for lg in (root, qtlog):
                for h in getattr(lg, "handlers", []) or []:
                    try:
                        h.flush()
                    except Exception:
                        pass
        except Exception:
            pass

    # Qt6: handler(msg_type, context, message)
    def handler(msg_type, context, message):
        lvl = levels.get(msg_type, logging.INFO)

        where = ""
        extra = ""

        try:
            file = getattr(context, "file", None)
            line = getattr(context, "line", None)
            function = getattr(context, "function", None)
            category = getattr(context, "category", None)

            if file and line:
                where = f"{_safe_str(file)}:{int(line)} "
            if function:
                extra += f"{_safe_str(function)} "
            if category:
                extra += f"[{_safe_str(category)}] "
        except Exception:
            pass

        msg = _safe_str(message)

        try:
            if extra:
                qtlog.log(lvl, "%s%s%s", where, extra, msg)
            else:
                qtlog.log(lvl, "%s%s", where, msg)
        except Exception:
            # super-fallback
            try:
                qtlog.log(lvl, msg)
            except Exception:
                pass

        # Bei Fatal: flushen (Qt kann danach sofort aborten)
        try:
            if msg_type == QtCore.QtMsgType.QtFatalMsg:
                _flush_all_handlers()
        except Exception:
            pass

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

    logging.getLogger("py.warnings").info("warnings capture aktiv (filter=%s)", pywarnings)
    if qt:
        logging.getLogger("qt").info("Qt message handler aktiv")
