"""Resource path resolution for Studio across source checkouts, wheel installs, and frozen builds."""
from __future__ import annotations

import os
import site
import sys
import sysconfig
from pathlib import Path
from typing import Iterable, List, Optional


def _dedup_paths(paths: Iterable[Optional[str | Path]]) -> List[Path]:
    """Return unique ``Path`` entries while preserving order."""
    seen: set[str] = set()
    out: List[Path] = []
    for item in paths:
        if not item:
            continue
        p = Path(item)
        key = os.path.normcase(os.path.abspath(str(p)))
        if key not in seen:
            seen.add(key)
            out.append(p)
    return out


def iter_studio_data_roots() -> List[Path]:
    """Return candidate ``share/pycub-studio`` data directories for pip/wheel installs."""
    bases: List[Optional[str]] = []
    try:
        bases.append(sysconfig.get_path("data"))
    except Exception:
        pass
    bases.extend([sys.prefix, getattr(sys, "base_prefix", None)])
    try:
        bases.append(site.getuserbase())
    except Exception:
        pass
    return [base / "share" / "pycub-studio" for base in _dedup_paths(bases)]


def locate_studio_asset(*rel_parts: str) -> Optional[Path]:
    """Resolve a file inside ``studio/assets/`` across source, wheel, and frozen builds."""
    studio_dir = Path(__file__).resolve().parent
    candidates: List[Path] = [studio_dir.joinpath("assets", *rel_parts)]
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass:
        candidates.append(Path(meipass).joinpath("studio", "assets", *rel_parts))
    if getattr(sys, "frozen", False):
        exe_dir = Path(sys.executable).resolve().parent
        candidates.append(exe_dir.joinpath("_internal", "studio", "assets", *rel_parts))
        candidates.append(exe_dir.joinpath("studio", "assets", *rel_parts))
    for cand in candidates:
        if cand.is_file():
            return cand
    return None
