"""
pyCub Standalone Cross-Platform Desktop Application.

Provides 5 integrated modes:
  1. Kid GUI (Native)
  2. Kid GUI "Scratch" (Offline Blockly)
  3. Kid GUI "Programming" (High-level Python API)
  4. Normal (Full pyCub IDE & Script Runner)
  5. Web GUI Selector (Classroom Multi-Instance Server Manager)
"""

from pathlib import Path as _Path


def _load_version() -> str:
    """Read Studio version from ``studio/VERSION``."""
    ver_file = _Path(__file__).resolve().parent / "VERSION"
    try:
        text = ver_file.read_text(encoding="utf-8").strip()
        if text:
            return text
    except OSError:
        pass
    return "1.4.0"


__version__ = _load_version()
