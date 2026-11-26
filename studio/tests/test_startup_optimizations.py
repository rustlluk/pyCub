"""Tests verifying lazy imports and binary mesh/point-cloud caching optimizations."""

import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]


class StartupOptimizationsTests(unittest.TestCase):
    """Test lazy heavy dependency loading, binary mesh caching, and deferred URDF parsing."""

    def test_pycub_defers_open3d_and_roboticstoolbox_until_needed(self):
        """Ensure ``open3d`` and ``roboticstoolbox`` are imported lazily only when requested."""
        code = """
import os, queue, sys, tempfile
os.environ["PYCUB_EMBEDDED_VIEWER"] = "1"
from icub_pybullet.pycub import pyCub, set_visualizer_factory
from studio.sim_bridge import EmbeddedVisualizerProxy, prepare_world_config

assert "open3d" not in sys.modules, "open3d imported at module load"
assert "roboticstoolbox" not in sys.modules, "roboticstoolbox imported at module load"

q = queue.Queue()
set_visualizer_factory(lambda c: EmbeddedVisualizerProxy(c, q, {}))
cfg = prepare_world_config("ball")
try:
    client = pyCub(config=cfg)
    assert "open3d" not in sys.modules, "open3d imported during embedded pyCub init"
    assert "roboticstoolbox" not in sys.modules, "roboticstoolbox imported before compute_jacobian"
    J, active = client.compute_jacobian("right_arm", end="r_hand")
    assert J.shape[0] == 6
    assert "roboticstoolbox" in sys.modules
    assert "open3d" not in sys.modules
    client.disconnect()
finally:
    if os.path.exists(cfg):
        os.unlink(cfg)
print("LAZY_OK")
"""
        env = dict(os.environ)
        env["PYTHONPATH"] = str(REPO_ROOT)
        proc = subprocess.run(
            [sys.executable, "-c", code],
            cwd=str(REPO_ROOT),
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stderr + "\n" + proc.stdout)
        self.assertIn("LAZY_OK", proc.stdout)

    def test_obj_bounds_and_meshbin_disk_cache_match_full_parse(self):
        """Ensure fast bounds parsing and disk mesh caches match full OBJ parsing."""
        from studio import obj_loader

        sample_objs = [
            REPO_ROOT / "icub_pybullet" / "other_meshes" / "ball" / "ball.obj",
            REPO_ROOT / "icub_pybullet" / "iCub" / "meshes" / "visual" / "full_l_hand.obj",
        ]
        with tempfile.TemporaryDirectory() as tmp_cache:
            with patch.dict(os.environ, {"PYCUB_CACHE_DIR": tmp_cache}):
                for obj_path in sample_objs:
                    obj_loader._MESH_CACHE.clear()
                    obj_loader._BOUNDS_CACHE.clear()

                    bounds = obj_loader.load_obj_bounds(str(obj_path), scale=0.001)
                    mesh_first = obj_loader.load_obj_mesh(str(obj_path), scale=0.001)

                    np.testing.assert_allclose(bounds.center, mesh_first.center, atol=1e-12)
                    np.testing.assert_allclose(bounds.bbox_min, mesh_first.bbox_min, atol=1e-12)
                    np.testing.assert_allclose(bounds.bbox_max, mesh_first.bbox_max, atol=1e-12)

                    # Clear in-memory caches and verify binary disk cache returns identical data
                    obj_loader._MESH_CACHE.clear()
                    obj_loader._BOUNDS_CACHE.clear()

                    bounds_disk = obj_loader.load_obj_bounds(str(obj_path), scale=0.001)
                    mesh_disk = obj_loader.load_obj_mesh(str(obj_path), scale=0.001)

                    np.testing.assert_array_equal(mesh_disk.interleaved_vn, mesh_first.interleaved_vn)
                    np.testing.assert_array_equal(mesh_disk.center, mesh_first.center)
                    np.testing.assert_array_equal(mesh_disk.bbox_min, mesh_first.bbox_min)
                    np.testing.assert_array_equal(mesh_disk.bbox_max, mesh_first.bbox_max)
                    np.testing.assert_array_equal(bounds_disk.center, bounds.center)
                    np.testing.assert_array_equal(bounds_disk.bbox_min, bounds.bbox_min)
                    np.testing.assert_array_equal(bounds_disk.bbox_max, bounds.bbox_max)

    def test_pcd_fast_loader_matches_open3d_without_importing_open3d(self):
        """Ensure binary PCD parsing avoids importing ``open3d`` and matches its coordinates."""
        code = """
import sys
from pathlib import Path
from studio.obj_loader import load_skin_pcd_points

root = Path(%r)
pts_arm = load_skin_pcd_points(str(root / "icub_pybullet" / "iCub" / "skin" / "point_clouds" / "left_arm.pcd"))
pts_foot = load_skin_pcd_points(str(root / "icub_pybullet" / "iCub" / "skin" / "point_clouds" / "left_foot.pcd"))
assert "open3d" not in sys.modules, "open3d imported during binary pcd load"
assert pts_arm.shape[1] == 3 and pts_arm.shape[0] > 0
assert pts_foot.shape[1] == 3 and pts_foot.shape[0] > 0
print("PCD_FAST_OK")
""" % str(REPO_ROOT)
        env = dict(os.environ)
        env["PYTHONPATH"] = str(REPO_ROOT)
        proc = subprocess.run(
            [sys.executable, "-c", code],
            cwd=str(REPO_ROOT),
            env=env,
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(proc.returncode, 0, msg=proc.stderr + "\n" + proc.stdout)
        self.assertIn("PCD_FAST_OK", proc.stdout)

        import open3d as o3d
        from studio import obj_loader

        obj_loader._PCD_CACHE.clear()
        for fname, do_scale in (("left_arm.pcd", True), ("left_foot.pcd", False)):
            pcd_path = REPO_ROOT / "icub_pybullet" / "iCub" / "skin" / "point_clouds" / fname
            fast_pts = obj_loader.load_skin_pcd_points(str(pcd_path))
            pc = o3d.io.read_point_cloud(str(pcd_path))
            if do_scale and len(pc.points) > 0:
                pc.scale(1.05, pc.get_center())
            o3d_pts = np.asarray(pc.points, dtype=np.float32)
            np.testing.assert_allclose(fast_pts, o3d_pts, atol=1e-5, rtol=1e-5)

    def test_urdf_deferred_find_root_tags_still_writes_valid_urdf(self):
        """Ensure deferred URDF root-tag discovery still writes complete valid XML."""
        from icub_pybullet.utils import URDF

        urdf_path = REPO_ROOT / "icub_pybullet" / "iCub" / "full.urdf"
        u = URDF(str(urdf_path), package_dir=str(REPO_ROOT / "icub_pybullet"))
        self.assertIsNone(u.ROOT_TAGS)
        u.expand_package_uris()
        u.write_urdf()
        self.assertIsNotNone(u.ROOT_TAGS)
        self.assertIn("mesh", u.ROOT_TAGS)
        self.assertIn("<robot name=\"iCub\">", u.new_urdf)
        self.assertIn("</robot>", u.new_urdf)

    def test_embedded_eye_save_images_uses_pil_without_open3d(self):
        """Verify embedded eye image saving writes PNGs via Pillow without ``open3d``."""
        from studio.sim_bridge import EmbeddedEyeWindow

        class _DummyParent:
            """Minimal parent stub providing ``file_dir`` for ``EmbeddedEyeWindow``."""

            def __init__(self, file_dir: str):
                """Store the mock package directory."""
                self.file_dir = file_dir

        with tempfile.TemporaryDirectory() as tmp_dir:
            with patch.dict(os.environ, {"PYCUB_WORKSPACE_DIR": tmp_dir}):
                ew = EmbeddedEyeWindow("l_eye", _DummyParent("/nonexistent/pkg"))
                ew.last_image = np.full((24, 32, 3), 120, dtype=np.uint8)
                ew.save_images()
                out_png = Path(tmp_dir) / "images" / "l_eye" / "RGB" / "0.png"
                self.assertTrue(out_png.is_file())
                self.assertGreater(out_png.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
