from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Optional

# ------------------------------------------------------------
# Kanonische Frames
# ------------------------------------------------------------
FRAMES_BASE: Dict[str, str] = {
    "tcp": "tcp",
    "world": "world",
    "meca_mount": "meca_mount",
    "home": "home",
    "predispense": "predispense",
    "service": "service",
    "scene": "scene",
    "substrate": "substrate",
    "substrate_mount": "substrate_mount",
    "cage": "cage",
    "tool_mount": "tool_mount",
}

# ------------------------------------------------------------
# Helper
# ------------------------------------------------------------
def _join(prefix: Optional[str], name: str) -> str:
    name = name.strip().strip("/")
    if not prefix:
        return name
    return f"{prefix.strip().strip('/')}/{name}"

# ------------------------------------------------------------
# Namespace-Ansicht auf Frames
# ------------------------------------------------------------
@dataclass(frozen=True)
class FramesNS:
    """
    Namespace-Ansicht auf Frames mit optionalem Prefix (z. B. 'robot1').

    Beispiel:
        f = FramesNS(prefix='robot1')
        f['tcp']  -> 'robot1/tcp'
        f.all()   -> dict mit allen geprefixten Frames
    """
    prefix: Optional[str] = None

    def __getitem__(self, key: str) -> str:
        base = FRAMES_BASE.get(key)
        if base is None:
            raise KeyError(f"Unknown frame key: {key!r}")
        return _join(self.prefix, base)

    def get(self, key: str, default: Optional[str] = None) -> Optional[str]:
        base = FRAMES_BASE.get(key)
        return _join(self.prefix, base) if base is not None else default

    def all(self) -> Dict[str, str]:
        return {k: _join(self.prefix, v) for k, v in FRAMES_BASE.items()}


# Bequemer Default ohne Prefix
FRAMES = FramesNS()

__all__ = ["FRAMES_BASE", "FramesNS", "FRAMES"]
