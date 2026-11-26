"""Tests for Windows OpenGL RHI configuration, WebEngine flags, and GL context lifecycle."""

import os
import sys
import unittest
from unittest.mock import patch

import numpy as np


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


class WindowsGraphicsSetupTests(unittest.TestCase):
    """Test platform Qt/OpenGL environment configuration and GL surface anchoring."""

    def test_win32_sets_opengl_rhi_and_chromium_flags(self):
        """Ensure Windows sets OpenGL RHI backend and software WebEngine flags."""
        from studio import main as app_main

        with patch.object(sys, "platform", "win32"), patch.dict(os.environ, {}, clear=True):
            app_main._configure_platform_qt_environment()
            self.assertEqual(os.environ.get("QSG_RHI_BACKEND"), "opengl")
            flags = os.environ.get("QTWEBENGINE_CHROMIUM_FLAGS", "")
            self.assertIn("--disable-gpu", flags)
            self.assertIn("--disable-gpu-compositing", flags)
            self.assertEqual(os.environ.get("QTWEBENGINE_DISABLE_SANDBOX"), "1")

    def test_win32_offscreen_skips_rhi_override(self):
        """Ensure offscreen Windows runs do not override ``QSG_RHI_BACKEND``."""
        from studio import main as app_main

        with (
            patch.object(sys, "platform", "win32"),
            patch.dict(os.environ, {"QT_QPA_PLATFORM": "offscreen"}, clear=True),
        ):
            app_main._configure_platform_qt_environment()
            self.assertNotIn("QSG_RHI_BACKEND", os.environ)

    def test_gl_surface_anchor_predicate(self):
        """Verify GL surface anchoring is enabled only on native Windows platforms."""
        _ensure_qapp()
        from studio import launcher

        with (
            patch.object(launcher.sys, "platform", "win32"),
            patch.object(launcher.QGuiApplication, "platformName", return_value="windows"),
            patch.dict(os.environ, {}, clear=True),
        ):
            self.assertTrue(launcher._should_use_gl_surface_anchor())

        with (
            patch.object(launcher.sys, "platform", "win32"),
            patch.object(launcher.QGuiApplication, "platformName", return_value="offscreen"),
            patch.dict(os.environ, {}, clear=True),
        ):
            self.assertFalse(launcher._should_use_gl_surface_anchor())

        with (
            patch.object(launcher.sys, "platform", "linux"),
            patch.object(launcher.QGuiApplication, "platformName", return_value="xcb"),
            patch.dict(os.environ, {}, clear=True),
        ):
            self.assertFalse(launcher._should_use_gl_surface_anchor())

    def test_hardware_gl_canvas_resets_handles_on_context_destroy(self):
        """Verify destroying the OpenGL context resets shader, VAO, and VBO handles."""
        _ensure_qapp()
        from studio.obj_loader import LoadedMeshData
        from studio.widgets.viewport import _HardwareGLCanvas, _SceneMeshItem, _SceneState

        st = _SceneState()
        mesh = _SceneMeshItem(
            key="part",
            mesh_data=LoadedMeshData(
                path="part.obj",
                scale=1.0,
                interleaved_vn=np.zeros((3, 6), dtype=np.float32),
                center=np.zeros(3, dtype=np.float64),
                bbox_min=np.zeros(3, dtype=np.float64),
                bbox_max=np.ones(3, dtype=np.float64),
                vertex_count=3,
            ),
            color=(1.0, 1.0, 1.0, 1.0),
            model=np.eye(4, dtype=np.float32),
            is_robot=True,
            vao=11,
            vbo=22,
        )
        st.meshes["part"] = mesh
        canvas = _HardwareGLCanvas(st)
        canvas._gl_ready = True
        canvas._mesh_prog = 3
        canvas._line_prog = 4
        canvas._dyn_vao = 5
        canvas._dyn_vbo = 6

        canvas._on_context_about_to_be_destroyed()

        self.assertFalse(canvas._gl_ready)
        self.assertEqual(canvas._mesh_prog, 0)
        self.assertEqual(canvas._line_prog, 0)
        self.assertEqual(canvas._dyn_vao, 0)
        self.assertEqual(canvas._dyn_vbo, 0)
        self.assertEqual(mesh.vao, 0)
        self.assertEqual(mesh.vbo, 0)

    def test_win32_main_window_creates_gl_anchor_and_switches_handbook(self):
        """Verify main window anchors GL surfaces on Windows and toggles the handbook view."""
        _ensure_qapp()
        from studio.launcher import PyCubMainWindow, _GLSurfaceAnchor

        with patch("studio.launcher._should_use_gl_surface_anchor", return_value=True):
            win = PyCubMainWindow()
        try:
            win.supervisor.start_interactive = lambda *_a, **_kw: None
            win.supervisor.stop_worker = lambda *_a, **_kw: None
            self.assertIsInstance(win._gl_anchor, _GLSurfaceAnchor)
            win.switch_mode("kid_scratch")
            self.assertEqual(win.stack.currentWidget(), win.kid_scratch)
            win.open_handbook()
            self.assertEqual(win.stack.currentWidget(), win.handbook_viewer)
            win.close_handbook()
            self.assertEqual(win.stack.currentWidget(), win.kid_scratch)
        finally:
            win.close()


if __name__ == "__main__":
    unittest.main()
