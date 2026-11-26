"""Tests for Light/Dark Studio theming, settings persistence, and spinbox/combo arrow assets."""
from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


def _ensure_qapp():
    """Return an active ``QApplication`` instance, creating one if needed."""
    try:
        import PySide6.QtWebEngineWidgets  # noqa: F401
    except Exception:
        pass
    from PySide6.QtCore import QCoreApplication, Qt
    from PySide6.QtWidgets import QApplication

    app = QApplication.instance()
    if app is None:
        QCoreApplication.setAttribute(Qt.ApplicationAttribute.AA_ShareOpenGLContexts, True)
        app = QApplication([])
    return app


class StudioThemeTests(unittest.TestCase):
    """Verify Light/Dark theme switching, persistence, and arrow icon stylesheet rules."""

    def test_settings_theme_roundtrip_and_invalid_fallback(self):
        """Ensure ``StudioSettings.theme`` persists across saves and defaults invalid values to ``light``."""
        from studio.settings import StudioSettings, load_studio_settings, save_studio_settings

        with tempfile.TemporaryDirectory() as tmp_dir:
            cfg = Path(tmp_dir) / "studio_settings.json"
            with patch.dict(os.environ, {"PYCUB_STUDIO_SETTINGS": str(cfg)}):
                self.assertEqual(load_studio_settings().theme, "light")

                save_studio_settings(StudioSettings(language="cs", theme="dark"))
                loaded = load_studio_settings()
                self.assertEqual(loaded.theme, "dark")
                self.assertEqual(loaded.language, "cs")

                cfg.write_text('{"theme": "neon_purple"}', encoding="utf-8")
                self.assertEqual(load_studio_settings().theme, "light")

    def test_arrow_and_checkbox_icon_assets_and_qss_rules_exist(self):
        """Ensure QSpinBox/QComboBox arrow PNGs and QCheckBox indicator icon exist and appear in QSS."""
        from studio.resource_paths import locate_studio_asset
        from studio.theme import build_base_qss

        for fname in (
            "spin_up_light.png",
            "spin_down_light.png",
            "spin_up_dark.png",
            "spin_down_dark.png",
            "check_white.png",
        ):
            asset = locate_studio_asset("icons", fname)
            self.assertIsNotNone(asset, msg=f"Missing icon asset: {fname}")
            self.assertTrue(asset.is_file(), msg=f"Asset path is not a file: {asset}")

        light_qss = build_base_qss("light")
        self.assertIn("QSpinBox::up-arrow", light_qss)
        self.assertIn("QSpinBox::down-arrow", light_qss)
        self.assertIn("QComboBox::down-arrow", light_qss)
        self.assertIn("spin_up_light.png", light_qss)
        self.assertIn("spin_down_light.png", light_qss)
        self.assertIn("check_white.png", light_qss)

        dark_qss = build_base_qss("dark")
        self.assertIn("spin_up_dark.png", dark_qss)
        self.assertIn("spin_down_dark.png", dark_qss)

    def test_main_window_toggle_theme_updates_and_persists(self):
        """Verify ``PyCubMainWindow.toggle_theme()`` updates palette, button text, modes, and settings."""
        app = _ensure_qapp()
        from PySide6.QtGui import QPalette
        from studio.launcher import PyCubMainWindow
        from studio.settings import StudioSettings, load_studio_settings, save_studio_settings
        from studio.theme import apply_studio_theme, get_active_theme
        from studio.widgets.packages_dialog import PackagesDialog

        with tempfile.TemporaryDirectory() as tmp_dir:
            cfg = Path(tmp_dir) / "studio_settings.json"
            with patch.dict(os.environ, {"PYCUB_STUDIO_SETTINGS": str(cfg)}):
                save_studio_settings(StudioSettings(theme="light"))
                apply_studio_theme(app, "light")
                win = PyCubMainWindow(initial_lang="en", initial_theme="light")
                try:
                    win.supervisor.start_interactive = lambda *_a, **_kw: None
                    win.supervisor.stop_worker = lambda *_a, **_kw: None

                    self.assertEqual(win.current_theme, "light")
                    self.assertEqual(get_active_theme(), "light")
                    self.assertIn("Dark", win.theme_btn.text())
                    self.assertEqual(app.palette().color(QPalette.ColorRole.Window).name(), "#f1f5f9")

                    win.toggle_theme()
                    self.assertEqual(win.current_theme, "dark")
                    self.assertEqual(get_active_theme(), "dark")
                    self.assertIn("Light", win.theme_btn.text())
                    self.assertEqual(app.palette().color(QPalette.ColorRole.Window).name(), "#0f172a")
                    self.assertEqual(load_studio_settings().theme, "dark")

                    # Czech translation updates theme button cleanly
                    win.i18n.set_language("cs")
                    self.assertIn("Světlý", win.theme_btn.text())

                    # Newly constructed PackagesDialog picks up active dark theme
                    dlg = PackagesDialog(win.i18n, win)
                    self.assertIn("#17253b", dlg._banner.styleSheet())
                    dlg.close()

                    # Toggle back to light
                    win.toggle_theme()
                    self.assertEqual(win.current_theme, "light")
                    self.assertIn("Tmavý", win.theme_btn.text())
                    self.assertEqual(load_studio_settings().theme, "light")
                finally:
                    win.close()
                    apply_studio_theme(app, "light")

    def test_main_honors_saved_and_cli_theme(self):
        """Ensure ``studio.main.main`` loads saved theme and lets ``--theme`` override it."""
        app = _ensure_qapp()
        from studio import main as app_main
        import studio.launcher  # noqa: F401
        import studio.widgets.splash  # noqa: F401
        from studio.settings import StudioSettings, save_studio_settings

        seen = []

        class _FakeSplash:
            def __init__(self, *args, **kwargs):
                pass

            def show(self):
                pass

            def set_progress(self, *_args, **_kwargs):
                pass

            def finish(self, _win):
                pass

        class _FakeWindow:
            def __init__(self, *args, initial_theme=None, **kwargs):
                seen.append(("win_theme", initial_theme))

            def showMaximized(self):
                pass

            def switch_mode(self, _mode):
                pass

        def _fake_apply(_app, theme="light"):
            seen.append(("apply_theme", theme))
            return theme

        with tempfile.TemporaryDirectory() as tmp_dir:
            cfg = Path(tmp_dir) / "studio_settings.json"
            with (
                patch.dict(os.environ, {"PYCUB_STUDIO_SETTINGS": str(cfg)}),
                patch("studio.widgets.splash.locate_app_icon", return_value=None),
                patch("studio.widgets.splash.StudioSplashScreen", _FakeSplash),
                patch("studio.theme.apply_studio_theme", _fake_apply),
                patch("studio.launcher.PyCubMainWindow", _FakeWindow),
                patch("PySide6.QtWidgets.QApplication", return_value=app),
                patch.object(app, "exec", return_value=0),
            ):
                save_studio_settings(StudioSettings(theme="dark"))
                rc1 = app_main.main([])
                self.assertEqual(rc1, 0)
                self.assertEqual(seen, [("apply_theme", "dark"), ("win_theme", "dark")])

                seen.clear()
                rc2 = app_main.main(["--theme", "light"])
                self.assertEqual(rc2, 0)
                self.assertEqual(seen, [("apply_theme", "light"), ("win_theme", "light")])


if __name__ == "__main__":
    unittest.main()
