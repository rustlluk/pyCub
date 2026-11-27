# -*- mode: python ; coding: utf-8 -*-
"""
Cross-platform PyInstaller specification for pyCub Studio (Windows, macOS arm64 & Intel x86_64, Linux).

Run from repository root:
    pyinstaller --noconfirm packaging/pycub.spec
"""
import os
import sys
from PyInstaller.utils.hooks import collect_all, collect_submodules, copy_metadata

SPEC_DIR = os.path.dirname(os.path.abspath(SPEC))
REPO_ROOT = os.path.abspath(os.path.join(SPEC_DIR, ".."))

def _resolve_studio_version():
    ver = (os.environ.get("PYCUB_VERSION") or os.environ.get("STUDIO_VERSION") or "").strip()
    if ver:
        return ver
    ver_file = os.path.join(REPO_ROOT, "studio", "VERSION")
    if os.path.isfile(ver_file):
        with open(ver_file, encoding="utf-8") as f:
            return f.read().strip() or "1.4.0"
    return "1.4.0"

STUDIO_VERSION = _resolve_studio_version()

datas = [
    (os.path.join(REPO_ROOT, "icub_pybullet", "configs"), os.path.join("icub_pybullet", "configs")),
    (os.path.join(REPO_ROOT, "icub_pybullet", "iCub"), os.path.join("icub_pybullet", "iCub")),
    (os.path.join(REPO_ROOT, "icub_pybullet", "other_meshes"), os.path.join("icub_pybullet", "other_meshes")),
    (os.path.join(REPO_ROOT, "icub_pybullet", "examples"), os.path.join("icub_pybullet", "examples")),
    (os.path.join(REPO_ROOT, "exercises"), "exercises"),
    (os.path.join(REPO_ROOT, "studio", "assets"), os.path.join("studio", "assets")),
    (os.path.join(REPO_ROOT, "studio", "VERSION"), "studio"),
    (os.path.join(REPO_ROOT, "docs", "handbook"), os.path.join("docs", "handbook")),
    (os.path.join(REPO_ROOT, "LICENSE"), "."),
    (os.path.join(REPO_ROOT, "THIRD_PARTY_LICENSES.md"), "."),
    (os.path.join(REPO_ROOT, "licenses"), "licenses"),
]
# Jedi resolves simulator definitions from source inside the installer.
for module in ("__init__", "pycub", "utils", "visualizer", "kid_api", "runtime_paths"):
    datas.append((os.path.join(REPO_ROOT, "icub_pybullet", module + ".py"), "icub_pybullet"))
binaries = []
hiddenimports = [
    "icub_pybullet",
    "icub_pybullet.pycub",
    "icub_pybullet.utils",
    "icub_pybullet.visualizer",
    "icub_pybullet.runtime_paths",
    "icub_pybullet.kid_api",
    "studio.kid_ui",
    "studio.kid_ui.kid_ui",
    "studio.kid_ui.kid_ui_multi",
    "studio.kid_ui.kid_ui_landing",
    "pybullet",
    "pybullet_utils.bullet_client",
    "OpenGL",
    "OpenGL.GL",
    "OpenGL.GL.shaders",
    "PySide6.QtOpenGL",
    "PySide6.QtOpenGLWidgets",
    "PySide6.QtQuick",
    "PySide6.QtQuickWidgets",
    "PySide6.QtWebChannel",
    "PySide6.QtWebEngineCore",
    "PySide6.QtWebEngineWidgets",
    "flask",
    "werkzeug.serving",
    "qrcode",
    "PIL",
    "psutil",
    "platformdirs",
    "yaml",
    "matplotlib",
    "matplotlib.backends.backend_agg",
    "pip",
]

for pkg in ("open3d", "pip"):
    try:
        pkg_datas, pkg_bins, pkg_hidden = collect_all(pkg)
        datas += pkg_datas
        binaries += pkg_bins
        hiddenimports += pkg_hidden
    except Exception:
        pass

# roboticstoolbox only needs its Python/C-extension modules for compute_jacobian();
# skip rtbdata/robot_descriptions/cpp-extensions (~157 MB of unused meshes, models, headers).
try:
    datas += copy_metadata("xacrodoc")
except Exception:
    pass

for pkg in ("spatialmath", "spatialgeometry", "xacrodoc"):
    try:
        hiddenimports += collect_submodules(pkg)
    except Exception:
        pass

try:
    hiddenimports += collect_submodules(
        "roboticstoolbox",
        filter=lambda name: not name.startswith((
            "roboticstoolbox.blocks",
            "roboticstoolbox.demo",
            "roboticstoolbox.backends.swift",
            "roboticstoolbox.backends.PyPlot",
        )),
    )
except Exception:
    pass

hiddenimports += collect_submodules("OpenGL.platform")

if sys.platform == "win32":
    icon_file = os.path.join(REPO_ROOT, "studio", "assets", "icons", "pycub.ico")
elif sys.platform == "darwin":
    icon_file = os.path.join(REPO_ROOT, "studio", "assets", "icons", "pycub.icns")
else:
    icon_file = os.path.join(REPO_ROOT, "studio", "assets", "icons", "pycub.png")

a = Analysis(
    [os.path.join(REPO_ROOT, "studio", "main.py")],
    pathex=[REPO_ROOT],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[
        "tkinter",
        "IPython",
        "notebook",
        "jupyter",
        "PyQt5",
        "PyQt6",
        "PySide2",
        "torch",
        "torchvision",
        "torchaudio",
        "nvidia",
        "tensorflow",
        "tensorboard",
        "rtbdata",
        "robot_descriptions",
        "roboticstoolbox.blocks",
        "roboticstoolbox.demo",
        "roboticstoolbox.backends.swift",
        "roboticstoolbox.backends.PyPlot",
    ],
    noarchive=False,
)

if sys.platform.startswith("linux"):
    # Exclude host-ABI system libraries from the PyInstaller bundle so builds
    # created on older distros (e.g. Ubuntu 22.04) use the target host's C++
    # runtime, Mesa/DRI/GBM/GL stack, GLib/GIO modules, and GTK/font stack
    # instead of colliding with newer system plugins on Ubuntu 24.04+.
    _LINUX_EXCLUDED_PREFIXES = (
        "libstdc++.so",
        "libgcc_s.so",
        "libglib-2.0.so",
        "libgio-2.0.so",
        "libgobject-2.0.so",
        "libgmodule-2.0.so",
        "libgthread-2.0.so",
        "libgbm.so",
        "libdrm",
        "libglapi.so",
        "libGL.so",
        "libEGL.so",
        "libGLX.so",
        "libGLdispatch.so",
        "libOpenGL.so",
        "libwayland-",
        "libexpat.so",
        "libz.so",
        "libzstd.so",
        "libfontconfig.so",
        "libfreetype.so",
        "libharfbuzz.so",
        "libgtk-3.so",
        "libgdk-3.so",
        "libgdk_pixbuf-2.0.so",
        "libpango-1.0.so",
        "libpangocairo-1.0.so",
        "libpangoft2-1.0.so",
        "libcairo.so",
        "libcairo-gobject.so",
        "libatk-1.0.so",
        "libatk-bridge-2.0.so",
        "libatspi.so",
        "libX11.so",
        "libX11-xcb.so",
        "libxcb.so",
        "libxcb-glx.so",
        "libxcb-dri3.so",
        "libdbus-1.so",
        "libsystemd.so",
        "libudev.so",
    )

    def _keep_linux_binary(entry):
        dest = entry[0].replace("\\", "/")
        if "/" in dest:
            return True
        return not dest.startswith(_LINUX_EXCLUDED_PREFIXES)

    a.binaries = [entry for entry in a.binaries if _keep_linux_binary(entry)]

pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="pyCub",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    icon=icon_file,
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name="pyCub",
)

if sys.platform == "darwin":
    app = BUNDLE(
        coll,
        name="pyCub.app",
        icon=icon_file,
        bundle_identifier="org.icub.pycub.studio",
        info_plist={
            "CFBundleDisplayName": "pyCub Studio",
            "CFBundleShortVersionString": STUDIO_VERSION,
            "CFBundleVersion": STUDIO_VERSION,
            "NSHighResolutionCapable": True,
        },
    )
