"""Setuptools packaging script for the ``pycub-studio`` desktop application distribution."""
from __future__ import annotations

import os
import shutil
import sys
import urllib.request
from pathlib import Path
from typing import List, Sequence, Tuple

from setuptools import find_packages, setup
from setuptools.command.egg_info import egg_info as _egg_info
from setuptools.command.sdist import sdist as _sdist

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
SDIST_DATA_DIR = HERE / "_sdist_data"


def _read_version(path: Path, *env_vars: str, fallback: str = "1.4.0") -> str:
    for var in env_vars:
        val = os.environ.get(var, "").strip()
        if val:
            return val
    if path.is_file():
        text = path.read_text(encoding="utf-8").strip()
        if text:
            return text
    return fallback


def _read_long_description() -> str:
    for candidate in (SDIST_DATA_DIR / "README.md", REPO_ROOT / "README.md", HERE / "README.md"):
        if candidate.is_file():
            return candidate.read_text(encoding="utf-8")
    url = "https://raw.githubusercontent.com/rustlluk/pyCub/refs/heads/master/README.md"
    try:
        with urllib.request.urlopen(url, timeout=5) as response:
            return response.read().decode("utf-8")
    except Exception:
        return ""


def _collect_package_data(subdir: str) -> List[str]:
    """Return paths relative to ``HERE`` for all files under ``HERE/subdir``."""
    target = HERE / subdir
    files: List[str] = []
    if not target.is_dir():
        return files
    for dirpath, dirnames, filenames in os.walk(target):
        dirnames[:] = sorted(d for d in dirnames if d != "__pycache__")
        for fname in sorted(filenames):
            if fname.endswith((".pyc", ".pyo")) or fname == ".DS_Store":
                continue
            rel = Path(dirpath, fname).relative_to(HERE).as_posix()
            files.append(rel)
    return files


def _collect_data_files(
    src_root: Path,
    dest_prefix: str,
    *,
    exclude_filenames: Sequence[str] = (),
    only_suffixes: Sequence[str] = (),
) -> List[Tuple[str, List[str]]]:
    """Collect ``data_files`` pairs preserving relative subdirectories under ``dest_prefix``."""
    if not src_root.is_dir():
        return []
    excluded = set(exclude_filenames)
    suffixes = tuple(only_suffixes)
    entries: List[Tuple[str, List[str]]] = []
    for dirpath, dirnames, filenames in os.walk(src_root):
        dirnames[:] = sorted(d for d in dirnames if d != "__pycache__" and not d.startswith("."))
        files: List[str] = []
        for fname in sorted(filenames):
            if fname in excluded or fname.endswith((".pyc", ".pyo")) or fname == ".DS_Store":
                continue
            if suffixes and not fname.endswith(suffixes):
                continue
            rel = os.path.relpath(Path(dirpath, fname), HERE).replace(os.sep, "/")
            files.append(rel)
        if files:
            rel_dir = Path(dirpath).relative_to(src_root).as_posix()
            target_dir = dest_prefix if rel_dir == "." else f"{dest_prefix}/{rel_dir}"
            entries.append((target_dir, files))
    return entries


def _copy_tree_filtered(
    src_root: Path,
    dst_root: Path,
    *,
    exclude_filenames: Sequence[str] = (),
    only_suffixes: Sequence[str] = (),
) -> None:
    """Copy a directory tree into ``dst_root`` while filtering caches and unwanted files."""
    if not src_root.is_dir():
        return
    excluded = set(exclude_filenames)
    suffixes = tuple(only_suffixes)
    for dirpath, dirnames, filenames in os.walk(src_root):
        dirnames[:] = sorted(d for d in dirnames if d != "__pycache__" and not d.startswith("."))
        rel_dir = Path(dirpath).relative_to(src_root)
        for fname in sorted(filenames):
            if fname in excluded or fname.endswith((".pyc", ".pyo")) or fname == ".DS_Store":
                continue
            if suffixes and not fname.endswith(suffixes):
                continue
            target_dir = dst_root / rel_dir
            target_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(Path(dirpath, fname), target_dir / fname)


class _StudioEggInfo(_egg_info):
    """Keep parent-relative ``data_files`` out of ``SOURCES.txt`` so ``sdist`` stays self-contained."""

    def find_sources(self) -> None:
        manifest_filename = os.path.join(self.egg_info, "SOURCES.txt")
        if os.path.isfile(manifest_filename):
            os.remove(manifest_filename)
        orig_data_files = self.distribution.data_files
        self.distribution.data_files = [
            (dest, [f for f in files if not f.startswith("../")])
            for dest, files in (orig_data_files or ())
        ]
        try:
            super().find_sources()
        finally:
            self.distribution.data_files = orig_data_files


class _StudioSdist(_sdist):
    """Custom ``sdist`` that stages shared repository assets inside ``_sdist_data/``."""

    def make_release_tree(self, base_dir: str, files: List[str]) -> None:
        safe_files = [f for f in files if not f.startswith("../")]
        super().make_release_tree(base_dir, safe_files)
        stage_root = Path(base_dir) / "_sdist_data"
        if SDIST_DATA_DIR.is_dir():
            if not stage_root.exists():
                shutil.copytree(SDIST_DATA_DIR, stage_root)
            return

        stage_root.mkdir(parents=True, exist_ok=True)
        _copy_tree_filtered(REPO_ROOT / "docs" / "handbook", stage_root / "handbook")
        _copy_tree_filtered(
            REPO_ROOT / "exercises",
            stage_root / "exercises",
            exclude_filenames=("exercises.zip",),
        )
        _copy_tree_filtered(
            REPO_ROOT / "icub_pybullet" / "examples",
            stage_root / "examples",
            only_suffixes=(".py",),
        )

        readme_src = REPO_ROOT / "README.md"
        if readme_src.is_file():
            shutil.copy2(readme_src, stage_root / "README.md")

        license_src = REPO_ROOT / "LICENSE"
        if license_src.is_file():
            shutil.copy2(license_src, stage_root / "LICENSE")
            shutil.copy2(license_src, Path(base_dir) / "LICENSE")

        icub_ver_src = REPO_ROOT / "icub_pybullet" / "VERSION"
        if icub_ver_src.is_file():
            shutil.copy2(icub_ver_src, stage_root / "ICUB_VERSION")


studio_version = _read_version(HERE / "VERSION", "STUDIO_VERSION", "PYCUB_VERSION")
icub_version_file = (
    SDIST_DATA_DIR / "ICUB_VERSION"
    if (SDIST_DATA_DIR / "ICUB_VERSION").is_file()
    else REPO_ROOT / "icub_pybullet" / "VERSION"
)
icub_min_version = _read_version(icub_version_file, "ICUB_VERSION")

req_path = HERE / "requirements.txt"
studio_reqs = [
    line.strip()
    for line in req_path.read_text(encoding="utf-8").splitlines()
    if line.strip() and not line.lstrip().startswith("#") and not line.lower().startswith("pyinstaller")
]
install_requires = [f"icub_pybullet>={icub_min_version}", *studio_reqs]

long_description = _read_long_description()

if "sdist" in sys.argv:
    # External shared assets are staged into _sdist_data/ by _StudioSdist.make_release_tree
    # so sdist manifests never contain ``../`` paths outside the package directory.
    data_files: List[Tuple[str, List[str]]] = []
else:
    data_base = SDIST_DATA_DIR if SDIST_DATA_DIR.is_dir() else None
    handbook_src = (data_base / "handbook") if data_base is not None else (REPO_ROOT / "docs" / "handbook")
    exercises_src = (data_base / "exercises") if data_base is not None else (REPO_ROOT / "exercises")
    examples_src = (data_base / "examples") if data_base is not None else (REPO_ROOT / "icub_pybullet" / "examples")
    data_files = [
        *_collect_data_files(handbook_src, "share/pycub-studio/handbook"),
        *_collect_data_files(exercises_src, "share/pycub-studio/exercises", exclude_filenames=("exercises.zip",)),
        *_collect_data_files(examples_src, "share/pycub-studio/examples", only_suffixes=(".py",)),
    ]

subpackages = find_packages(where=".", exclude=["tests", "tests.*", "_sdist_data", "_sdist_data.*"])
packages = ["studio", *(f"studio.{pkg}" for pkg in subpackages)]

setup(
    name="pycub-studio",
    version=studio_version,
    description="pyCub Studio - Cross-platform GUI, Blockly & Python IDE for iCub in PyBullet",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Lukas Rustler",
    author_email="lukas.rustler@fel.cvut.cz",
    url="https://www.lukasrustler.cz/pycub",
    license="Creative Commons Attribution 4.0 International (CC BY 4.0)",
    packages=packages,
    package_dir={"studio": "."},
    package_data={"studio": ["VERSION", *_collect_package_data("assets")]},
    include_package_data=True,
    data_files=data_files,
    cmdclass={"egg_info": _StudioEggInfo, "sdist": _StudioSdist},
    install_requires=install_requires,
    extras_require={
        "build": ["pyinstaller>=6.10.0"],
    },
    entry_points={
        "console_scripts": [
            "pycub-studio=studio.main:main",
        ],
    },
    platforms=["any"],
    python_requires=">=3.10, <3.13",
)
