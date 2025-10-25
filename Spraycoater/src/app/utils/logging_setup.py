# -*- coding: utf-8 -*-
import os
import io
import yaml
import logging
import logging.config
from typing import Any, Dict

def _expand_env(obj: Any) -> Any:
    """Ersetzt $VARS in allen Strings rekursiv (auch ${VAR})."""
    if isinstance(obj, dict):
        return {k: _expand_env(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_expand_env(x) for x in obj]
    if isinstance(obj, str):
        return os.path.expandvars(obj)
    return obj

def _ensure_log_dirs(cfg: Dict[str, Any]) -> None:
    """Prüft, dass alle Handler-Dateipfade existieren (nur Verzeichnisse!)."""
    handlers = cfg.get("handlers", {})
    for hname, hcfg in handlers.items():
        if isinstance(hcfg, dict) and "filename" in hcfg:
            fn = hcfg["filename"]
            d = os.path.dirname(fn) or "."
            if not os.path.isdir(d):
                raise FileNotFoundError(f"Log-Verzeichnis existiert nicht für Handler '{hname}': {d}")

def configure_logging_from_yaml(logging_yaml_path: str, *, log_dir: str) -> None:
    """
    Lädt logging.yaml strikt, ersetzt ${LOG_DIR}, prüft Verzeichnisse
    und wendet dictConfig an. Keine stillen Fallbacks.
    """
    if not logging_yaml_path:
        raise ValueError("logging_yaml_path ist leer.")
    if not os.path.exists(logging_yaml_path):
        raise FileNotFoundError(f"logging.yaml nicht gefunden: {logging_yaml_path}")
    if not os.path.isdir(log_dir):
        raise FileNotFoundError(f"log_dir existiert nicht: {log_dir}")

    # LOG_DIR als Env exportieren, damit ${LOG_DIR} expandiert werden kann
    os.environ["LOG_DIR"] = log_dir

    with io.open(logging_yaml_path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f)
    if not isinstance(raw, dict) or not raw:
        raise ValueError(f"logging.yaml leer oder ungültig: {logging_yaml_path}")

    cfg = _expand_env(raw)
    _ensure_log_dirs(cfg)

    logging.config.dictConfig(cfg)
    logging.getLogger("app").info(
        "Logging konfiguriert (yaml=%s, LOG_DIR=%s)", logging_yaml_path, log_dir
    )
