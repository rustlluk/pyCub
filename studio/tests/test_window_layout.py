"""Tests for maximized Studio startup, FlowLayout wrapping, and compact window resizing."""

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


class WindowLayoutTests(unittest.TestCase):
    """Verify maximized launch and compact resizability of pyCub Studio."""

    def test_main_launches_window_maximized(self):
        """Ensure ``studio.main.main`` displays ``PyCubMainWindow`` via ``showMaximized``."""
        app = _ensure_qapp()
        from studio import main as app_main
        import studio.launcher  # noqa: F401
        import studio.widgets.splash  # noqa: F401

        called = []

        class _FakeSplash:
            def __init__(self, *args, **kwargs):
                pass

            def show(self):
                pass

            def set_progress(self, *_args, **_kwargs):
                pass

            def finish(self, win):
                called.append(("finish", win))

        class _FakeWindow:
            def __init__(self, *args, **kwargs):
                pass

            def show(self):
                called.append(("show", None))

            def showMaximized(self):
                called.append(("showMaximized", None))

            def switch_mode(self, mode):
                called.append(("switch_mode", mode))

        with tempfile.TemporaryDirectory() as tmp_dir:
            cfg = Path(tmp_dir) / "studio_settings.json"
            with (
                patch.dict(os.environ, {"PYCUB_STUDIO_SETTINGS": str(cfg)}),
                patch("studio.widgets.splash.locate_app_icon", return_value=None),
                patch("studio.widgets.splash.StudioSplashScreen", _FakeSplash),
                patch("studio.theme.apply_studio_theme", lambda _app, _theme="light": "light"),
                patch("studio.launcher.PyCubMainWindow", _FakeWindow),
                patch("PySide6.QtWidgets.QApplication", return_value=app),
                patch.object(app, "exec", return_value=0),
            ):
                rc = app_main.main([])

        self.assertEqual(rc, 0)
        self.assertIn(("showMaximized", None), called)
        self.assertNotIn(("show", None), called)

    def test_main_window_resizable_below_640x480_in_both_languages(self):
        """Ensure ``PyCubMainWindow`` minimum size hint allows resizing down to 640x480."""
        app = _ensure_qapp()
        from studio.launcher import PyCubMainWindow

        for lang in ("en", "cs"):
            win = PyCubMainWindow(initial_lang=lang)
            try:
                win.supervisor.start_interactive = lambda *_a, **_kw: None
                win.supervisor.stop_worker = lambda *_a, **_kw: None

                min_hint = win.minimumSizeHint()
                self.assertLessEqual(min_hint.width(), 640, msg=f"min width too large for {lang}: {min_hint}")
                self.assertLessEqual(min_hint.height(), 480, msg=f"min height too large for {lang}: {min_hint}")

                win.show()
                win.resize(640, 480)
                app.processEvents()
                self.assertEqual(win.size().width(), 640)
                self.assertEqual(win.size().height(), 480)
            finally:
                win.close()

    def test_flow_layout_wraps_when_narrow_and_expands_stretch_when_wide(self):
        """Verify ``FlowLayout`` keeps items on one row with stretch when wide and wraps when narrow."""
        app = _ensure_qapp()
        from PySide6.QtWidgets import QPushButton, QWidget
        from studio.widgets.flow_layout import FlowLayout

        host = QWidget()
        lay = FlowLayout(host, margin=0, h_spacing=8, v_spacing=6)
        left_btn = QPushButton("Left Button")
        right_btn = QPushButton("Right Button")
        lay.addWidget(left_btn)
        lay.addStretch(1)
        lay.addWidget(right_btn)

        host.show()
        # Wide enough for a single row: both widgets share y == 0 and right_btn is aligned to right edge.
        host.resize(600, 100)
        app.processEvents()
        self.assertEqual(left_btn.geometry().y(), right_btn.geometry().y())
        self.assertGreater(right_btn.geometry().x(), left_btn.geometry().right() + 20)

        # Narrower than combined width: right_btn wraps onto a second row.
        narrow_w = max(left_btn.sizeHint().width(), right_btn.sizeHint().width()) + 10
        self.assertGreater(lay.heightForWidth(narrow_w), lay.heightForWidth(600))
        host.resize(narrow_w, lay.heightForWidth(narrow_w))
        app.processEvents()
        self.assertGreater(right_btn.geometry().y(), left_btn.geometry().y())
        host.close()

    def test_kid_native_and_home_reflow_cards_for_narrow_widths(self):
        """Verify KidNativeMode and LauncherHomeWidget adapt columns and jog rows without overflow."""
        app = _ensure_qapp()
        from PySide6.QtCore import Qt
        from studio.launcher import PyCubMainWindow

        win = PyCubMainWindow(initial_lang="cs")
        try:
            win.supervisor.start_interactive = lambda *_a, **_kw: None
            win.supervisor.stop_worker = lambda *_a, **_kw: None
            win.show()

            # LauncherHomeWidget: 3 -> 2 -> 1 columns
            win.switch_mode("home")
            win.resize(1200, 720)
            app.processEvents()
            self.assertEqual(win.home_widget._grid_cols, 3)
            win.resize(640, 720)
            app.processEvents()
            self.assertEqual(win.home_widget._grid_cols, 2)
            win.resize(500, 720)
            app.processEvents()
            self.assertEqual(win.home_widget._grid_cols, 1)

            # KidNativeMode: 2 cols at wide, 1 col + stacked jogs at narrow, vertical splitter at <560px
            win.switch_mode("kid_native")
            kn = win.kid_native
            for width, exp_orient, exp_cols, exp_stacked in (
                (1340, Qt.Orientation.Horizontal, 2, False),
                (900, Qt.Orientation.Horizontal, 1, False),
                (640, Qt.Orientation.Horizontal, 1, True),
                (520, Qt.Orientation.Vertical, 1, False),
            ):
                win.resize(width, 720)
                app.processEvents()
                app.processEvents()
                self.assertEqual(kn.splitter.orientation(), exp_orient, msg=f"width={width}")
                self.assertEqual(kn._grid_cols, exp_cols, msg=f"width={width}")
                self.assertEqual(kn._jog_stacked, exp_stacked, msg=f"width={width}")
                self.assertLessEqual(
                    kn._right_scroll.widget().width(),
                    kn._right_scroll.viewport().width(),
                    msg=f"horizontal overflow at width={width}",
                )
        finally:
            win.close()


if __name__ == "__main__":
    unittest.main()
