"""
Unified Entrypoint for pyCub Standalone Desktop Application & Frozen Subprocess Workers.

Usage:
    python -m studio.main                         # Open Home Mode Launcher
    python -m studio.main --mode kid_native       # Open directly in a mode
    python -m studio.main --run-script my_code.py # Open Normal Mode & run .py script
    python -m studio.main --lang cs               # Start in Czech

Internal worker flags (used when packaged as a single frozen OS executable):
    --worker-web-instance [kid_ui args...]
    --worker-web-multi    [kid_ui_multi args...]
    --worker-pip          [pip args...]
"""
from __future__ import annotations

import argparse
import ctypes
import multiprocessing as mp
import os
import sys


def _ensure_repo_on_path() -> None:
    """Insert the repository root into ``sys.path`` if not already present."""
    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(here)
    if repo_root not in sys.path:
        sys.path.insert(0, repo_root)


def _is_headless_qt_platform() -> bool:
    """Return True if Qt is configured to use a headless QPA platform ('offscreen' or 'minimal')."""
    return os.environ.get("QT_QPA_PLATFORM") in ("offscreen", "minimal")


def _configure_platform_qt_environment() -> None:
    """Configure Qt/WebEngine flags and bundled Linux fonts before QApplication & QtWebEngine init."""
    if sys.platform.startswith("linux"):
        from studio.fonts import register_fontconfig_app_fonts

        register_fontconfig_app_fonts()

    if sys.platform == "win32":
        # Ensure Windows taskbar shows the pyCub icon instead of a generic Python icon
        try:
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("org.icub.pycub.studio")
        except Exception:
            pass

    if sys.platform in ("win32", "darwin"):
        # On Windows (D3D11) and macOS (Metal), Qt 6's QWebEngineView uses Qt Quick with a
        # non-OpenGL RHI backend by default, which conflicts with QOpenGLWidget in the same
        # QMainWindow (causing a black QWebEngineView in Scratch/Handbook or context teardown
        # when switching stacked pages). Force Qt Quick / RHI to OpenGL before QApplication init.
        if not _is_headless_qt_platform():
            os.environ.setdefault("QSG_RHI_BACKEND", "opengl")
        # Prevent Chromium GPU compositor (ANGLE/D3D11 on Windows, ANGLE/Metal IOSurface on macOS)
        # from failing to composite into Qt Quick's OpenGL scene graph, and disable the Chromium
        # sandbox inside frozen desktop bundles.
        flags = os.environ.get("QTWEBENGINE_CHROMIUM_FLAGS", "")
        for flag in ("--disable-gpu", "--disable-gpu-compositing"):
            if flag not in flags:
                flags = f"{flags} {flag}".strip()
        os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = flags
        os.environ.setdefault("QTWEBENGINE_DISABLE_SANDBOX", "1")


def _configure_lightweight_surface_format() -> None:
    """Configure default QSurfaceFormat without importing heavy numpy/OpenGL modules before splash."""
    if _is_headless_qt_platform():
        return
    from PySide6.QtGui import QSurfaceFormat

    fmt = QSurfaceFormat()
    fmt.setDepthBufferSize(24)
    fmt.setStencilBufferSize(8)
    fmt.setSwapBehavior(QSurfaceFormat.SwapBehavior.DoubleBuffer)
    if sys.platform == "darwin":
        # macOS native Core Profile is OpenGL 4.1 (supports GLSL 330 core shaders and QRhiGles2).
        # Avoid setting global MSAA on macOS/Windows so QQuickWidget/QWebEngineView, _GLSurfaceAnchor,
        # and QOpenGLWidget share identical pixel format attributes under AA_ShareOpenGLContexts.
        fmt.setRenderableType(QSurfaceFormat.RenderableType.OpenGL)
        fmt.setVersion(4, 1)
        fmt.setProfile(QSurfaceFormat.OpenGLContextProfile.CoreProfile)
    elif sys.platform != "win32":
        fmt.setSamples(4)
    QSurfaceFormat.setDefaultFormat(fmt)


def main(argv: list[str] | None = None) -> int:
    """Launch pyCub Studio GUI or dispatch frozen internal worker modes.

    :param argv: Optional CLI argument list; defaults to ``sys.argv[1:]``.
    """
    mp.freeze_support()
    _ensure_repo_on_path()

    raw_args = list(sys.argv[1:] if argv is None else argv)

    # Dispatch frozen web-instance workers before importing Qt GUI modules
    if raw_args and raw_args[0] == "--worker-web-instance":
        from studio.kid_ui import kid_ui

        sys.argv = [sys.argv[0]] + raw_args[1:]
        kid_ui.main()
        return 0

    if raw_args and raw_args[0] == "--worker-web-multi":
        from studio.kid_ui import kid_ui_multi

        kid_ui_multi.main(raw_args[1:])
        return 0

    if raw_args and raw_args[0] == "--worker-pip":
        from pip._internal.cli.main import main as pip_main

        try:
            return int(pip_main(raw_args[1:]) or 0)
        except SystemExit as exc:
            if exc.code is None:
                return 0
            if isinstance(exc.code, int):
                return exc.code
            print(exc.code, file=sys.stderr)
            return 1

    parser = argparse.ArgumentParser(description="pyCub Standalone Simulator & Studio")
    parser.add_argument(
        "--mode",
        choices=["home", "kid_native", "kid_scratch", "kid_code", "normal", "web"],
        default="home",
        help="Initial mode to open (default: home launcher)",
    )
    parser.add_argument(
        "--run-script",
        default=None,
        help="Open Normal IDE mode and immediately execute the specified .py script",
    )
    parser.add_argument(
        "--lang",
        choices=["en", "cs"],
        default="en",
        help="Initial UI language ('en' or 'cs')",
    )
    parser.add_argument(
        "--theme",
        choices=["light", "dark"],
        default=None,
        help="Initial UI theme ('light' or 'dark'); defaults to saved Studio settings",
    )
    args = parser.parse_args(raw_args)

    _configure_platform_qt_environment()

    from PySide6.QtCore import QCoreApplication, Qt
    from PySide6.QtGui import QIcon
    from PySide6.QtWidgets import QApplication

    QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)
    _configure_lightweight_surface_format()

    if sys.platform in ("win32", "darwin") and not _is_headless_qt_platform():
        try:
            from PySide6.QtQuick import QQuickWindow, QSGRendererInterface

            QQuickWindow.setGraphicsApi(QSGRendererInterface.GraphicsApi.OpenGL)
        except Exception:
            pass
        # Import QtWebEngineWidgets before constructing QApplication so WebEngine initializes
        # its shared OpenGL context hooks cleanly at startup.
        try:
            import PySide6.QtWebEngineWidgets  # noqa: F401
        except Exception:
            pass

    app = QApplication(sys.argv[:1])
    app.setApplicationName("pyCub Studio")
    app.setOrganizationName("pyCub")

    from studio.fonts import configure_application_font_fallbacks

    configure_application_font_fallbacks(app)

    from studio.widgets.splash import StudioSplashScreen, locate_app_icon

    icon_path = locate_app_icon()
    if icon_path:
        app.setWindowIcon(QIcon(icon_path))

    splash = StudioSplashScreen(icon_path=icon_path, lang=args.lang)
    splash.show()
    app.processEvents()

    is_cs = args.lang == "cs"
    splash.set_progress(18, "Načítám vzhled aplikace…" if is_cs else "Loading application theme…")
    from studio.settings import load_studio_settings
    from studio.theme import apply_studio_theme

    saved_settings = load_studio_settings()
    initial_theme = args.theme if args.theme is not None else saved_settings.theme
    apply_studio_theme(app, initial_theme)

    splash.set_progress(40, "Načítám hlavní okno Studia…" if is_cs else "Loading Studio interface…")
    from studio.launcher import PyCubMainWindow

    win = PyCubMainWindow(
        initial_lang=args.lang,
        initial_theme=initial_theme,
        icon_path=icon_path,
        progress_cb=splash.set_progress,
    )
    win.showMaximized()

    if args.run_script:
        win.switch_mode("normal")
        win.normal_ide.open_and_run_script(args.run_script)
    elif args.mode != "home":
        win.switch_mode(args.mode)

    splash.finish(win)
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
