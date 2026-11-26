"""
Persistent Studio Settings & Extra Python Packages / External Interpreter Helpers.
"""
from __future__ import annotations

import glob
import importlib.util
import json
import os
import shutil
import subprocess
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

from icub_pybullet.runtime_paths import get_extra_packages_dir, is_frozen, user_data_dir


@dataclass
class StudioSettings:
    """User-configurable runtime preferences stored across sessions."""

    extra_python_paths: List[str] = field(default_factory=list)
    custom_python_exe: str = ""
    language: str = "en"
    theme: str = "light"


def get_settings_path() -> Path:
    """Return the JSON settings file path, honoring ``PYCUB_STUDIO_SETTINGS`` if set."""
    override = os.environ.get("PYCUB_STUDIO_SETTINGS")
    if override:
        return Path(override).expanduser().resolve()
    cfg_dir = Path(user_data_dir("pycub", False))
    cfg_dir.mkdir(parents=True, exist_ok=True)
    return cfg_dir / "studio_settings.json"


def load_studio_settings() -> StudioSettings:
    """Load persisted :class:`StudioSettings` from disk, returning defaults on error or missing file."""
    p = get_settings_path()
    if not p.is_file():
        return StudioSettings()
    try:
        with open(p, "r", encoding="utf-8") as f:
            raw = json.load(f) or {}
        paths = [str(x) for x in raw.get("extra_python_paths", []) if isinstance(x, str) and x.strip()]
        py_exe = str(raw.get("custom_python_exe") or "").strip()
        lang = str(raw.get("language") or "en")
        raw_theme = str(raw.get("theme") or "light").strip().lower()
        theme = raw_theme if raw_theme in ("light", "dark") else "light"
        return StudioSettings(extra_python_paths=paths, custom_python_exe=py_exe, language=lang, theme=theme)
    except Exception:
        return StudioSettings()


def save_studio_settings(settings: StudioSettings) -> None:
    """Persist ``settings`` as formatted JSON to the settings file.

    :param settings: Settings dataclass instance to serialize.
    """
    p = get_settings_path()
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(asdict(settings), f, indent=2)


def effective_extra_paths(settings: Optional[StudioSettings] = None) -> List[str]:
    """Return deduplicated list of existing extra package folders (always including extra_packages)."""
    if settings is None:
        settings = load_studio_settings()
    candidates: List[str] = [str(get_extra_packages_dir())]
    env_extra = os.environ.get("PYCUB_EXTRA_PYTHON_PATHS", "")
    if env_extra:
        candidates.extend(p for p in env_extra.split(os.pathsep) if p.strip())
    candidates.extend(settings.extra_python_paths)

    out: List[str] = []
    seen = set()
    for raw in candidates:
        ap = os.path.abspath(os.path.expanduser(raw))
        if ap not in seen and os.path.isdir(ap):
            seen.add(ap)
            out.append(ap)
    return out


def discover_site_packages_in_venv(venv_or_python_path: str) -> List[str]:
    """Given a venv root or python executable path, locate any `site-packages` directories inside it."""
    p = Path(os.path.expanduser(venv_or_python_path)).resolve()
    if p.is_file():
        if p.parent.name.lower() in ("bin", "scripts"):
            venv_root = p.parent.parent
        else:
            venv_root = p.parent
    else:
        venv_root = p

    found: List[str] = []
    patterns = [
        str(venv_root / "Lib" / "site-packages"),
        str(venv_root / "lib" / "python*" / "site-packages"),
        str(venv_root / "lib64" / "python*" / "site-packages"),
    ]
    for pat in patterns:
        for match in sorted(glob.glob(pat)):
            if os.path.isdir(match) and match not in found:
                found.append(match)
    return found


def has_bundled_pip() -> bool:
    """Return True if `pip._internal.cli.main` is available inside the current interpreter/bundle."""
    return importlib.util.find_spec("pip._internal.cli.main") is not None


def find_system_python_for_pip() -> Optional[str]:
    """Find a runnable Python executable (outside frozen app) that has `pip` for package installation."""
    candidates: List[str] = []
    if not is_frozen():
        candidates.append(sys.executable)
    for name in ("python3", "python", "py"):
        exe = shutil.which(name)
        if exe:
            candidates.append(exe)

    seen = set()
    for exe in candidates:
        ap = os.path.abspath(exe)
        if ap in seen:
            continue
        seen.add(ap)
        try:
            res = subprocess.run(
                [ap, "-m", "pip", "--version"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if res.returncode == 0:
                return ap
        except Exception:
            continue
    return None


def build_pip_install_command(
    packages: List[str],
    target_dir: str,
    custom_python_exe: str = "",
) -> Optional[Tuple[str, List[str], str]]:
    """
    Build `(program, args, mode_label)` to install PyPI packages into `target_dir`.

    Priority:
      1. User-specified `custom_python_exe` (if valid file) -> `<exe> -m pip install --target ...`
      2. Bundled `pip` inside the frozen binary -> `<exe> --worker-pip install --prefer-binary --no-build-isolation --target ...`
      3. Current non-frozen Python or system Python with pip -> `<exe> -m pip install --target ...`
    """
    exe_clean = (custom_python_exe or "").strip()
    if exe_clean and os.path.isfile(exe_clean):
        return (
            exe_clean,
            ["-m", "pip", "install", "--upgrade", "--target", target_dir, *packages],
            "custom",
        )

    if is_frozen():
        if has_bundled_pip():
            return (
                sys.executable,
                [
                    "--worker-pip",
                    "install",
                    "--prefer-binary",
                    "--no-build-isolation",
                    "--upgrade",
                    "--target",
                    target_dir,
                    *packages,
                ],
                "bundled",
            )
    else:
        if has_bundled_pip():
            return (
                sys.executable,
                ["-m", "pip", "install", "--upgrade", "--target", target_dir, *packages],
                "venv",
            )

    sys_py = find_system_python_for_pip()
    if sys_py:
        return (
            sys_py,
            ["-m", "pip", "install", "--upgrade", "--target", target_dir, *packages],
            "system",
        )
    return None
