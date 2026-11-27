"""
Bundled Font Discovery & Fallback Registration for pyCub Studio.

Linux systems (especially minimal containers, VMs, or lightweight desktops) may lack
emoji or symbol fonts such as ``Noto Color Emoji`` or ``DejaVu Sans``. This module
locates the fonts bundled under ``studio/assets/fonts/``, registers them with
``fontconfig`` (when available on Linux) and Qt's ``QFontDatabase``, and configures
application-wide font fallbacks so UI icons and symbols render without relying on
system-installed fonts.
"""
from __future__ import annotations

import ctypes
import ctypes.util
import os
import sys
from pathlib import Path
from typing import Iterable, List, Optional

_FONT_EXTENSIONS = (".ttf", ".otf", ".ttc")
_FONTCONFIG_REGISTERED_DIRS: set[str] = set()
_QT_REGISTERED_FONTS: dict[str, List[str]] = {}

_SANS_FALLBACK_TARGETS = (
    "Sans Serif",
    "sans-serif",
    "DejaVu Sans",
    "Ubuntu",
    "Noto Sans",
    "Liberation Sans",
    "Cantarell",
    "Helvetica",
    "Arial",
)

_MONO_FALLBACK_TARGETS = (
    "monospace",
    "Monospace",
    "DejaVu Sans Mono",
    "JetBrains Mono",
    "Fira Code",
    "Consolas",
    "Ubuntu Mono",
    "Liberation Mono",
)


def _dedup_preserve_order(items: Iterable[str]) -> List[str]:
    """Return unique non-empty strings while preserving original order."""
    seen: set[str] = set()
    out: List[str] = []
    for item in items:
        clean = str(item or "").strip()
        if not clean or clean in seen:
            continue
        seen.add(clean)
        out.append(clean)
    return out


def locate_studio_fonts_dir() -> Optional[Path]:
    """Resolve ``studio/assets/fonts`` across source checkouts, wheel installs, and frozen builds."""
    studio_dir = Path(__file__).resolve().parent
    candidates: List[Path] = [studio_dir / "assets" / "fonts"]
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        candidates.append(Path(meipass) / "studio" / "assets" / "fonts")
    if getattr(sys, "frozen", False):
        exe_dir = Path(sys.executable).resolve().parent
        candidates.append(exe_dir / "_internal" / "studio" / "assets" / "fonts")
        candidates.append(exe_dir / "studio" / "assets" / "fonts")
    for cand in candidates:
        if cand.is_dir() and any(
            p.is_file() and p.suffix.lower() in _FONT_EXTENSIONS for p in cand.iterdir()
        ):
            return cand
    return None


def iter_bundled_font_files(fonts_dir: Optional[Path] = None) -> List[Path]:
    """Return sorted font file paths inside the bundled fonts directory."""
    root = fonts_dir if fonts_dir is not None else locate_studio_fonts_dir()
    if root is None or not root.is_dir():
        return []
    return sorted(
        (p for p in root.iterdir() if p.is_file() and p.suffix.lower() in _FONT_EXTENSIONS),
        key=lambda p: p.name,
    )


def register_fontconfig_app_fonts(fonts_dir: Optional[Path] = None) -> bool:
    """Register bundled fonts with Linux ``fontconfig`` via ``FcConfigAppFontAddDir``."""
    if not sys.platform.startswith("linux"):
        return False
    root = fonts_dir if fonts_dir is not None else locate_studio_fonts_dir()
    if root is None or not root.is_dir():
        return False

    key = os.path.abspath(str(root))
    if key in _FONTCONFIG_REGISTERED_DIRS:
        return True

    try:
        lib_name = ctypes.util.find_library("fontconfig") or "libfontconfig.so.1"
        fc = ctypes.CDLL(lib_name)
        fc.FcInit.restype = ctypes.c_int
        fc.FcConfigAppFontAddDir.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        fc.FcConfigAppFontAddDir.restype = ctypes.c_int
        fc.FcInit()
        ok = bool(fc.FcConfigAppFontAddDir(None, os.fsencode(key)))
        if ok:
            _FONTCONFIG_REGISTERED_DIRS.add(key)
        return ok
    except Exception:
        return False


def register_qt_bundled_fonts(fonts_dir: Optional[Path] = None) -> List[str]:
    """Register bundled Linux fonts in ``QFontDatabase`` and return loaded font family names."""
    if not sys.platform.startswith("linux"):
        return []
    root = fonts_dir if fonts_dir is not None else locate_studio_fonts_dir()
    font_files = iter_bundled_font_files(root)
    if not font_files:
        return []

    register_fontconfig_app_fonts(root)

    from PySide6.QtGui import QFontDatabase

    families: List[str] = []
    for path in font_files:
        key = os.path.abspath(str(path))
        cached = _QT_REGISTERED_FONTS.get(key)
        if cached is not None:
            families.extend(cached)
            continue
        font_id = QFontDatabase.addApplicationFont(str(path))
        if font_id >= 0:
            loaded = list(QFontDatabase.applicationFontFamilies(font_id))
            _QT_REGISTERED_FONTS[key] = loaded
            families.extend(loaded)
    return _dedup_preserve_order(families)


def configure_application_font_fallbacks(app=None) -> List[str]:
    """Ensure Qt on Linux uses bundled ``DejaVu Sans`` and ``Noto Color Emoji`` as fallbacks."""
    if not sys.platform.startswith("linux"):
        return []

    families = register_qt_bundled_fonts()
    if not families:
        return []

    from PySide6.QtGui import QFont
    from PySide6.QtWidgets import QApplication

    target_app = app if app is not None else QApplication.instance()

    sans_fallbacks = _dedup_preserve_order(
        [
            *(f for f in ("DejaVu Sans", "Noto Color Emoji", "Noto Emoji") if f in families),
            *families,
        ]
    )
    mono_fallbacks = _dedup_preserve_order(
        [
            *(f for f in ("DejaVu Sans Mono", "Noto Color Emoji", "Noto Emoji", "DejaVu Sans") if f in families),
            *families,
        ]
    )

    sans_targets = list(_SANS_FALLBACK_TARGETS)
    if target_app is not None:
        base_font = target_app.font()
        primary = base_font.family()
        if primary:
            sans_targets.insert(0, primary)
        existing_families = list(base_font.families()) if hasattr(base_font, "families") else [primary]
        merged = _dedup_preserve_order([*existing_families, *sans_fallbacks])
        if hasattr(base_font, "setFamilies"):
            base_font.setFamilies(merged)
        target_app.setFont(base_font)

    for target in _dedup_preserve_order(sans_targets):
        QFont.insertSubstitutions(target, sans_fallbacks)
    for target in _MONO_FALLBACK_TARGETS:
        QFont.insertSubstitutions(target, mono_fallbacks)

    return families
