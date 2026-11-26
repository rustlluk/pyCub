"""Unit tests for wheel/data-dir resource discovery outside a repository checkout."""
from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


class ResourcePathsTests(unittest.TestCase):
    """Verify `studio` locates handbook, exercises, and examples via `share/pycub-studio`."""

    def test_locate_handbook_falls_back_to_installed_data_dir(self):
        """Ensure `locate_handbook_dir()` finds `share/pycub-studio/handbook` outside checkout."""
        from studio.widgets import handbook_viewer

        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp = Path(tmp_dir)
            fake_module_file = tmp / "site-packages" / "studio" / "widgets" / "handbook_viewer.py"
            fake_module_file.parent.mkdir(parents=True, exist_ok=True)
            fake_module_file.touch()

            data_handbook = tmp / "venv" / "share" / "pycub-studio" / "handbook"
            data_handbook.mkdir(parents=True, exist_ok=True)
            (data_handbook / "index.html").write_text("<html></html>", encoding="utf-8")

            with (
                patch.object(handbook_viewer, "__file__", str(fake_module_file)),
                patch.object(handbook_viewer, "iter_studio_data_roots", return_value=[tmp / "venv" / "share" / "pycub-studio"]),
            ):
                self.assertEqual(handbook_viewer.locate_handbook_dir(), data_handbook)

    def test_locate_exercises_and_examples_fall_back_to_installed_data_dir(self):
        """Ensure exercises and examples fall back to `share/pycub-studio/{exercises,examples}`."""
        from studio.modes import normal_ide

        with tempfile.TemporaryDirectory() as tmp_dir:
            tmp = Path(tmp_dir)
            fake_studio_file = tmp / "site-packages" / "studio" / "modes" / "normal_ide.py"
            fake_icub_file = tmp / "site-packages" / "icub_pybullet" / "__init__.py"
            fake_studio_file.parent.mkdir(parents=True, exist_ok=True)
            fake_icub_file.parent.mkdir(parents=True, exist_ok=True)
            fake_studio_file.touch()
            fake_icub_file.touch()

            data_root = tmp / "venv" / "share" / "pycub-studio"
            data_exercises = data_root / "exercises"
            (data_exercises / "exercise_1").mkdir(parents=True, exist_ok=True)
            data_examples = data_root / "examples"
            data_examples.mkdir(parents=True, exist_ok=True)
            (data_examples / "demo.py").write_text("print('ok')\n", encoding="utf-8")

            with (
                patch.object(normal_ide, "__file__", str(fake_studio_file)),
                patch.object(normal_ide.icub_pkg, "__file__", str(fake_icub_file)),
                patch.object(normal_ide, "iter_studio_data_roots", return_value=[data_root]),
            ):
                self.assertEqual(normal_ide._locate_bundled_exercises_root(), data_exercises)
                self.assertEqual(normal_ide._locate_bundled_examples_dir(), data_examples)

    def test_iter_studio_data_roots_deduplicates_sys_and_user_bases(self):
        """Ensure `iter_studio_data_roots()` preserves order and deduplicates equivalent paths."""
        from studio import resource_paths

        with tempfile.TemporaryDirectory() as tmp_dir:
            base = Path(tmp_dir) / "env"
            user_base = Path(tmp_dir) / "user"
            with (
                patch.object(resource_paths.sysconfig, "get_path", return_value=str(base)),
                patch.object(resource_paths.sys, "prefix", str(base)),
                patch.object(resource_paths.sys, "base_prefix", str(base)),
                patch.object(resource_paths.site, "getuserbase", return_value=str(user_base)),
            ):
                roots = resource_paths.iter_studio_data_roots()
                self.assertEqual(
                    roots,
                    [base / "share" / "pycub-studio", user_base / "share" / "pycub-studio"],
                )


if __name__ == "__main__":
    unittest.main()
