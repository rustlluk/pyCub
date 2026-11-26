"""
Cross-platform runtime paths, user workspace/cache helpers, config/mesh resolution,
and OS-agnostic process/network utilities for pyCub.
"""
from __future__ import annotations

import hashlib
import os
import socket
import sys
import tempfile
import time
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import psutil

try:
    from platformdirs import user_cache_dir, user_data_dir, user_documents_dir
except ImportError:  # pragma: no cover - lightweight fallback
    def user_documents_dir() -> str:
        """Return the fallback path to the user's Documents directory."""
        return os.path.join(os.path.expanduser("~"), "Documents")

    def user_cache_dir(appname: str = "pycub", appauthor: bool = False) -> str:
        """Return the fallback path to the platform cache directory for ``appname``."""
        if sys.platform == "win32":
            base = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~\\AppData\\Local")
            return os.path.join(base, appname, "Cache")
        if sys.platform == "darwin":
            return os.path.join(os.path.expanduser("~/Library/Caches"), appname)
        base = os.environ.get("XDG_CACHE_HOME") or os.path.expanduser("~/.cache")
        return os.path.join(base, appname)

    def user_data_dir(appname: str = "pycub", appauthor: bool = False) -> str:
        """Return the fallback path to the platform application data directory for ``appname``."""
        if sys.platform == "win32":
            base = os.environ.get("LOCALAPPDATA") or os.path.expanduser("~\\AppData\\Local")
            return os.path.join(base, appname, "Data")
        if sys.platform == "darwin":
            return os.path.join(os.path.expanduser("~/Library/Application Support"), appname)
        base = os.environ.get("XDG_DATA_HOME") or os.path.expanduser("~/.local/share")
        return os.path.join(base, appname)


SKIP_IFACE_PREFIXES = ("lo", "docker", "br-", "veth", "virbr", "lxc", "tailscale", "utun", "awdl", "llw")


def is_frozen() -> bool:
    """Return True when running inside a frozen bundle (e.g. PyInstaller)."""
    return bool(getattr(sys, "frozen", False))


def to_posix(path: str | os.PathLike) -> str:
    """Normalize a path and convert separators to forward slashes (safe inside URDF XML on Windows)."""
    return os.path.normpath(os.fspath(path)).replace("\\", "/")


def is_writable_dir(directory: str | os.PathLike) -> bool:
    """Return True only if directory is writable and NOT inside a frozen bundle directory."""
    d = os.path.abspath(os.fspath(directory))
    meipass = getattr(sys, "_MEIPASS", None)
    if meipass and d.startswith(os.path.abspath(meipass)):
        return False
    if is_frozen():
        exe_dir = os.path.dirname(os.path.abspath(sys.executable))
        if d.startswith(exe_dir):
            return False
    if not os.path.isdir(d):
        return False
    try:
        fd, probe = tempfile.mkstemp(prefix=".pycub_write_test_", dir=d)
        os.close(fd)
        os.unlink(probe)
        return True
    except OSError:
        return False


def get_user_workspace_dir() -> Path:
    """Return the user's writable pyCub workspace directory (default: ~/Documents/pyCub)."""
    override = os.environ.get("PYCUB_WORKSPACE_DIR")
    if override:
        return Path(override).expanduser().resolve()
    return Path(user_documents_dir()) / "pyCub"


def ensure_user_workspace() -> Path:
    """Ensure standard user workspace subdirectories exist and return root path."""
    root = get_user_workspace_dir()
    for sub in ("worlds", "configs", "meshes", "scripts", "projects", "images", "extra_packages"):
        (root / sub).mkdir(parents=True, exist_ok=True)
    return root


def get_extra_packages_dir() -> Path:
    """Return the writable extra Python packages directory (`~/Documents/pyCub/extra_packages`)."""
    p = get_user_workspace_dir() / "extra_packages"
    p.mkdir(parents=True, exist_ok=True)
    return p


def get_user_cache_dir() -> Path:
    """Return writable cache directory for generated URDFs and dynamic VHACD collision meshes."""
    override = os.environ.get("PYCUB_CACHE_DIR")
    base = Path(override).expanduser().resolve() if override else Path(user_cache_dir("pycub", False))
    for sub in ("vhacd", "fixed_urdf", "generated_urdf"):
        (base / sub).mkdir(parents=True, exist_ok=True)
    return base


def get_user_log_dir(package_dir: Optional[str] = None) -> Path:
    """Return writable log directory (falls back to user data dir when package_dir/logs is read-only)."""
    override = os.environ.get("PYCUB_LOG_DIR")
    if override:
        p = Path(override).expanduser().resolve()
        p.mkdir(parents=True, exist_ok=True)
        return p
    if package_dir and not is_frozen():
        pkg_logs = os.path.join(package_dir, "logs")
        if is_writable_dir(pkg_logs):
            return Path(pkg_logs)
    p = Path(user_data_dir("pycub", False)) / "logs"
    p.mkdir(parents=True, exist_ok=True)
    return p


def write_text_atomic(target_path: str | os.PathLike, content: str) -> str:
    """Write text atomically via temp file + os.replace so concurrent instances never read partial URDFs."""
    target = Path(target_path)
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{target.stem}_", suffix=target.suffix, dir=str(target.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(content)
        os.replace(tmp_name, str(target))
    except Exception:
        try:
            if os.path.exists(tmp_name):
                os.unlink(tmp_name)
        except OSError:
            pass
        raise
    return str(target)


def resolve_config_path(config: str, package_dir: str, caller_file: Optional[str] = None) -> str:
    """
    Resolve a YAML config path across:
      1. Direct / absolute path or relative to CWD
      2. Relative to caller script directory
      3. User workspace (~/Documents/pyCub/worlds, configs, root)
      4. Bundled icub_pybullet/configs
    """
    if os.path.isabs(config) and os.path.isfile(config):
        return os.path.abspath(config)

    candidates: List[str] = [os.path.abspath(config)]

    if caller_file:
        caller_dir = os.path.dirname(os.path.abspath(caller_file))
        candidates.append(os.path.join(caller_dir, config))
        candidates.append(os.path.join(caller_dir, "configs", config))

    ws = get_user_workspace_dir()
    candidates.extend([
        str(ws / "worlds" / config),
        str(ws / "configs" / config),
        str(ws / config),
    ])

    candidates.extend([
        os.path.join(package_dir, "configs", config),
    ])

    for c_path in candidates:
        if os.path.isfile(c_path):
            return os.path.abspath(c_path)

    raise FileNotFoundError(
        f"Could not find config '{config}'. Searched current directory, user workspace ({ws}), "
        f"and built-in configs ({os.path.join(package_dir, 'configs')})."
    )


def resolve_robot_urdf_path(robot_urdf_rel: str, package_dir: str, config_dir: Optional[str] = None) -> str:
    """Resolve robot_urdf_path across absolute path, config dir, user workspace, and bundled package dir."""
    if os.path.isabs(robot_urdf_rel) and os.path.isfile(robot_urdf_rel):
        return os.path.abspath(robot_urdf_rel)

    candidates: List[str] = []
    if config_dir:
        candidates.append(os.path.join(config_dir, robot_urdf_rel))
    ws = get_user_workspace_dir()
    candidates.extend([
        str(ws / robot_urdf_rel),
        str(ws / "meshes" / robot_urdf_rel),
        os.path.join(package_dir, robot_urdf_rel),
    ])
    for c in candidates:
        if os.path.isfile(c):
            return os.path.abspath(c)
    return os.path.abspath(os.path.join(package_dir, robot_urdf_rel))


def resolve_mesh_or_urdf_path(obj_path: str, package_dir: str, config_dir: Optional[str] = None) -> str:
    """
    Resolve an object .obj or .urdf path referenced in config.urdfs.paths across:
      1. Absolute path
      2. Relative to the loaded config file's directory
      3. Relative to CWD
      4. User workspace (~/Documents/pyCub/meshes and root)
      5. Bundled icub_pybullet/other_meshes and package root
    """
    if os.path.isabs(obj_path) and os.path.isfile(obj_path):
        return os.path.abspath(obj_path)

    candidates: List[str] = []
    if config_dir:
        candidates.append(os.path.join(config_dir, obj_path))
        candidates.append(os.path.join(config_dir, "other_meshes", obj_path))
        candidates.append(os.path.join(config_dir, "meshes", obj_path))

    candidates.append(os.path.abspath(obj_path))

    ws = get_user_workspace_dir()
    candidates.extend([
        str(ws / "meshes" / obj_path),
        str(ws / obj_path),
    ])

    candidates.extend([
        os.path.join(package_dir, "other_meshes", obj_path),
        os.path.join(package_dir, obj_path),
    ])

    for c in candidates:
        if os.path.isfile(c):
            return os.path.abspath(c)

    raise FileNotFoundError(
        f"Could not find object mesh/URDF '{obj_path}'. Searched config dir, user workspace ({ws / 'meshes'}), "
        f"and built-in other_meshes."
    )


def file_fingerprint(path: str) -> str:
    """Fast fingerprint combining canonical path, size, and mtime_ns."""
    ap = os.path.abspath(path)
    try:
        st = os.stat(ap)
        raw = f"{ap}|{st.st_size}|{st.st_mtime_ns}".encode("utf-8")
    except OSError:
        raw = ap.encode("utf-8")
    return hashlib.sha256(raw).hexdigest()[:12]


def get_vhacd_cache_path(col_path: str, force_vhacd: bool = False) -> Tuple[str, bool]:
    """
    Locate an existing pre-baked ``*_vhacd.obj`` or return a writable cache path.
    Returns (vhacd_mesh_path, needs_compute).
    """
    col_abs = os.path.abspath(col_path)
    prebaked_candidates = [
        col_abs.replace(".obj", "_vhacd.obj").replace(f"{os.sep}visual{os.sep}", f"{os.sep}vhacd{os.sep}"),
        col_abs.replace(".obj", "_vhacd.obj"),
    ]
    if not force_vhacd:
        for cand in prebaked_candidates:
            if os.path.isfile(cand):
                return cand, False

    stem = Path(col_abs).stem
    fp = file_fingerprint(col_abs)
    cached = get_user_cache_dir() / "vhacd" / f"{stem}_{fp}_vhacd.obj"
    needs_compute = force_vhacd or not cached.is_file()
    return str(cached), needs_compute


def get_cached_urdf_path(stem: str, key: str, subdir: str = "generated_urdf") -> str:
    """Return a deterministic path in the user cache for a generated/patched URDF."""
    digest = hashlib.sha256(key.encode("utf-8")).hexdigest()[:12]
    safe_stem = "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in stem)
    return str(get_user_cache_dir() / subdir / f"{safe_stem}_{digest}.urdf")


def local_ipv4_addresses() -> List[str]:
    """Cross-platform IPv4 LAN addresses discovery (Windows, macOS, Linux)."""
    addresses: List[str] = []
    try:
        stats = psutil.net_if_stats()
        for iface, addrs in psutil.net_if_addrs().items():
            low = iface.lower()
            if low.startswith(SKIP_IFACE_PREFIXES) or "loopback" in low:
                continue
            if iface in stats and not stats[iface].isup:
                continue
            for snic in addrs:
                if snic.family == socket.AF_INET:
                    ip = snic.address
                    if ip and not ip.startswith("127.") and not ip.startswith("169.254."):
                        if ip not in addresses:
                            addresses.append(ip)
    except Exception:
        pass

    if not addresses:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("8.8.8.8", 80))
                ip = s.getsockname()[0]
                if ip and not ip.startswith("127."):
                    addresses.append(ip)
        except OSError:
            pass
    return addresses


def terminate_process_tree(procs: Iterable, timeout: float = 5.0) -> None:
    """Cross-platform termination of child processes and all their descendants using psutil."""
    ps_roots: List[psutil.Process] = []
    for p in procs:
        if p is None:
            continue
        pid = p.pid if hasattr(p, "pid") else int(p)
        if pid is None:
            continue
        try:
            ps_roots.append(psutil.Process(pid))
        except psutil.NoSuchProcess:
            pass

    all_procs: List[psutil.Process] = []
    for root in ps_roots:
        try:
            all_procs.extend(root.children(recursive=True))
        except psutil.NoSuchProcess:
            pass
        all_procs.append(root)

    seen = set()
    unique_procs: List[psutil.Process] = []
    for pr in all_procs:
        if pr.pid not in seen:
            seen.add(pr.pid)
            unique_procs.append(pr)

    for pr in unique_procs:
        try:
            pr.terminate()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

    _, alive = psutil.wait_procs(unique_procs, timeout=timeout)
    for pr in alive:
        try:
            pr.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    if alive:
        psutil.wait_procs(alive, timeout=2.0)
