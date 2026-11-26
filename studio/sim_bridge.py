"""
Multiprocessing Simulation Bridge & EmbeddedVisualizerProxy for pyCub Standalone.

Connects PyBullet (`pyCub`) running in an isolated worker process to the Qt main window's
`EmbeddedGLViewer` and `EyeCameraPanel` widgets at ~30-60 FPS without Open3D GUI windows.

Supports two worker execution modes:
  1. Interactive Sandbox Worker (`start_interactive`): persistent physics world used by
     Mode 1 (Kid Native), Mode 2 (Scratch Blockly), Mode 3 (Kid Programming), and Mode 4's idle preview.
  2. Full Script Worker (`start_user_script`): executes arbitrary `.py` files (full `pyCub` API,
     examples, exercises, testers) in a clean spawned process while streaming 3D geometry,
     tactile skin activations, custom `LineSet` overlays, eye camera RGB/Depth feeds, and stdout/stderr.
"""
from __future__ import annotations

import glob
import io
import logging
import math
import multiprocessing as mp
from multiprocessing.connection import Client, Listener
import os
import queue
import runpy
import site
import subprocess
import sys
import tempfile
import threading
import time
import traceback
from typing import Any, Callable, Dict, Generator, List, Optional, Set, Tuple

import numpy as np
import yaml

from icub_pybullet.runtime_paths import (
    ensure_user_workspace,
    get_user_workspace_dir,
    is_writable_dir,
    resolve_config_path,
    resolve_mesh_or_urdf_path,
    resolve_robot_urdf_path,
    terminate_process_tree,
)
from studio.obj_loader import load_obj_bounds
from studio.settings import effective_extra_paths, load_studio_settings

try:
    from PySide6.QtCore import QObject, QTimer, Signal
    HAS_QT = True
except Exception:  # pragma: no cover
    QObject = object
    QTimer = None
    Signal = None
    HAS_QT = False


BUILTIN_KID_WORLDS: Dict[str, Tuple[Tuple[str, str], str]] = {
    "ball": (("Stůl s míčkem", "Table with a ball"), "with_ball.yaml"),
    "empty": (("Prázdný pokoj", "Empty room"), "default.yaml"),
    "skin": (("Dotyková kůže", "Tactile skin"), "skin_test.yaml"),
}


def discover_all_worlds() -> Dict[str, Tuple[Tuple[str, str], str]]:
    """Return built-in worlds plus any user `.yaml`/`.yml` worlds in `~/Documents/pyCub/worlds`."""
    worlds = dict(BUILTIN_KID_WORLDS)
    ws_worlds = get_user_workspace_dir() / "worlds"
    if ws_worlds.is_dir():
        for p in sorted(ws_worlds.iterdir()):
            if p.is_file() and p.suffix.lower() in (".yaml", ".yml"):
                wid = f"user:{p.stem}"
                label = p.stem.replace("_", " ").title()
                worlds[wid] = ((f"🌍 {label}", f"🌍 {label}"), str(p.resolve()))
    return worlds


def prepare_world_config(world_or_path: str) -> str:
    """
    Resolve a world ID or custom `.yaml` path and write a temporary config prepared for embedded rendering,
    resolving relative robot/object mesh paths against the original config's directory.
    """
    import icub_pybullet.pycub as pycub_mod

    pkg_dir = os.path.dirname(os.path.abspath(pycub_mod.__file__))
    worlds = discover_all_worlds()
    if world_or_path in worlds:
        target = worlds[world_or_path][1]
        if os.path.isabs(target) and os.path.isfile(target):
            src_yaml = target
        else:
            src_yaml = resolve_config_path(target, pkg_dir)
    else:
        src_yaml = resolve_config_path(world_or_path, pkg_dir)

    src_dir = os.path.dirname(os.path.abspath(src_yaml))
    with open(src_yaml, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}

    # Preserve parent reference resolution relative to original config dir if used
    if "parent" in cfg and isinstance(cfg["parent"], str) and not os.path.isabs(cfg["parent"]):
        cfg["parent"] = resolve_config_path(cfg["parent"], pkg_dir, caller_file=src_yaml)

    if "robot_urdf_path" in cfg and isinstance(cfg["robot_urdf_path"], str):
        cfg["robot_urdf_path"] = resolve_robot_urdf_path(cfg["robot_urdf_path"], pkg_dir, src_dir)
    if "urdfs" in cfg and isinstance(cfg["urdfs"], dict) and "paths" in cfg["urdfs"]:
        cfg["urdfs"]["paths"] = [
            resolve_mesh_or_urdf_path(p, pkg_dir, src_dir) for p in cfg["urdfs"]["paths"]
        ]

    gui_cfg = dict(cfg.get("gui") or {})
    gui_cfg["standard"] = True
    gui_cfg["web"] = False
    cfg["gui"] = gui_cfg
    cfg["debug"] = False

    fd, tmp_path = tempfile.mkstemp(prefix="pycub_embedded_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f)
    return tmp_path


# =================================================================================================
# EmbeddedVisualizerProxy (runs inside worker process, replaces Open3D Visualizer)
# =================================================================================================

class _EmbeddedSceneProxy:
    """
    Compatible stand-in for `visualizer.vis` (`Open3DScene`), supporting `add_geometry`,
    `remove_geometry`, `set_geometry_transform`, and `get_geometry_transform` (used e.g. by Exercise 3).
    """

    def __init__(self, parent: "EmbeddedVisualizerProxy"):
        """Initialize the embedded scene proxy with empty transforms and custom line overlays."""
        self._parent = parent
        self._transforms: Dict[str, np.ndarray] = {}
        self.custom_lines: Dict[str, Dict[str, Any]] = {}
        self.lines_dirty = False

    def set_geometry_transform(self, name: str, transform: np.ndarray) -> None:
        """Store the 4x4 homogeneous transform matrix for geometry ``name``.

        :param name: Geometry identifier key.
        :param transform: 4x4 homogeneous transformation matrix.
        """
        self._transforms[name] = np.asarray(transform, dtype=np.float64)

    def get_geometry_transform(self, name: str) -> np.ndarray:
        """Return the stored 4x4 transform for ``name``, or identity if unset.

        :param name: Geometry identifier key.
        """
        return self._transforms.get(name, np.eye(4, dtype=np.float64))

    def remove_geometry(self, name: str) -> None:
        """Remove custom line overlays and cached transforms associated with ``name``.

        :param name: Geometry identifier key to remove.
        """
        if name in self.custom_lines:
            del self.custom_lines[name]
            self.lines_dirty = True
        self._transforms.pop(name, None)

    def add_geometry(self, name: str, geometry: Any, material: Any = None) -> None:
        """Extract ``LineSet`` segments and colors from ``geometry`` for embedded rendering.

        :param name: Geometry identifier key.
        :param geometry: Geometry object exposing ``points`` and ``lines`` attributes.
        :param material: Optional material whose ``line_width`` attribute sets stroke width.
        """
        # Extract LineSet segments & colors (e.g. Exercise 3 gaze lines)
        if hasattr(geometry, "lines") and hasattr(geometry, "points"):
            pts = np.asarray(geometry.points, dtype=np.float32)
            lines = np.asarray(geometry.lines, dtype=np.int64)
            cols = np.asarray(geometry.colors, dtype=np.float32) if hasattr(geometry, "colors") else np.empty((0, 3))
            if pts.shape[0] >= 2 and lines.shape[0] >= 1:
                seg_verts = np.empty((lines.shape[0] * 2, 6), dtype=np.float32)
                for idx in range(lines.shape[0]):
                    i0, i1 = int(lines[idx, 0]), int(lines[idx, 1])
                    col = cols[idx] if cols.shape[0] > idx else (cols[0] if cols.shape[0] > 0 else np.array([1.0, 0.2, 0.2]))
                    seg_verts[2 * idx, :3] = pts[i0]
                    seg_verts[2 * idx, 3:] = col
                    seg_verts[2 * idx + 1, :3] = pts[i1]
                    seg_verts[2 * idx + 1, 3:] = col
                width = float(getattr(material, "line_width", 3.0)) if material is not None else 3.0
                self.custom_lines[name] = {"vertices": seg_verts.tolist(), "width": width}
                self.lines_dirty = True


class EmbeddedEyeWindow:
    """
    Drop-in replacement for `Visualizer.EyeWindow` using PyBullet's offscreen camera renderer
    and exact OpenGL View-Projection unprojection (`unproject(u, v, d)`).
    """

    WIDTH = 320
    HEIGHT = 240
    FOVY = 60.0
    NEAR = 0.03
    FAR = 5.0

    def __init__(self, eye: str, parent: "EmbeddedVisualizerProxy"):
        """Initialize offscreen camera state and image buffers for the specified eye."""
        self.eye = eye
        self.link_name = f"{eye}_pupil"
        self.parent = parent
        self.center = np.array([0.0, 0.0, 0.5], dtype=np.float64)
        self.dir = np.array([-0.1, 0.0, 0.0], dtype=np.float64)
        self.img_counter = 0
        self.save_images_bool = False
        self.last_image: np.ndarray = np.full((self.HEIGHT, self.WIDTH, 3), 255, dtype=np.uint8)
        self.last_depth_image: np.ndarray = np.ones((self.HEIGHT, self.WIDTH), dtype=np.float32)
        self._last_inv_vp: np.ndarray = np.eye(4, dtype=np.float64)

    def capture(self) -> None:
        """Render an offscreen RGB and depth frame from the eye pose via PyBullet TinyRenderer."""
        client = self.parent.client
        eye_pos = self.center.tolist()
        target_pos = (self.center + self.dir).tolist()
        view_mat = client.computeViewMatrix(
            cameraEyePosition=eye_pos,
            cameraTargetPosition=target_pos,
            cameraUpVector=[0.0, 0.0, 1.0],
        )
        proj_mat = client.computeProjectionMatrixFOV(
            fov=self.FOVY,
            aspect=float(self.WIDTH) / float(self.HEIGHT),
            nearVal=self.NEAR,
            farVal=self.FAR,
        )
        _, _, rgba, depth, _ = client.getCameraImage(
            self.WIDTH,
            self.HEIGHT,
            viewMatrix=view_mat,
            projectionMatrix=proj_mat,
            shadow=0,
            lightDirection=[-0.5, -0.3, 1.0],
            lightColor=[1.0, 1.0, 1.0],
            lightDistance=3.0,
            renderer=client.ER_TINY_RENDERER,
        )
        rgba_arr = np.asarray(rgba, dtype=np.uint8).reshape((self.HEIGHT, self.WIDTH, 4))
        self.last_image = rgba_arr[:, :, :3].copy()
        self.last_depth_image = np.asarray(depth, dtype=np.float32).reshape((self.HEIGHT, self.WIDTH)).copy()

        v_m = np.asarray(view_mat, dtype=np.float64).reshape((4, 4), order="F")
        p_m = np.asarray(proj_mat, dtype=np.float64).reshape((4, 4), order="F")
        try:
            self._last_inv_vp = np.linalg.inv(p_m @ v_m)
        except np.linalg.LinAlgError:
            self._last_inv_vp = np.eye(4, dtype=np.float64)

        if self.save_images_bool:
            self.save_images()

    def save_images(self) -> None:
        """Save the latest captured RGB eye frame to disk as a PNG image."""
        from PIL import Image

        parent_root = os.path.normpath(os.path.join(self.parent.file_dir, ".."))
        if is_writable_dir(parent_root):
            out_dir = os.path.join(parent_root, "images", self.eye, "RGB")
        else:
            out_dir = str(ensure_user_workspace() / "images" / self.eye / "RGB")
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, f"{self.img_counter}.png")
        self.img_counter += 1
        Image.fromarray(self.last_image).save(out_path)

    def unproject(self, u: float, v: float, d: float) -> np.ndarray:
        """
        Unproject pixel `(u, v)` and OpenGL non-linear depth buffer value `d` in `[0, 1]`
        into 3D world coordinates `[X, Y, Z]`, matching `Open3DScene.camera.unproject`.
        """
        x_ndc = (2.0 * (float(u) + 0.5) / float(self.WIDTH)) - 1.0
        y_ndc = 1.0 - (2.0 * (float(v) + 0.5) / float(self.HEIGHT))
        z_ndc = 2.0 * float(d) - 1.0
        clip = np.array([x_ndc, y_ndc, z_ndc, 1.0], dtype=np.float64)
        world_h = self._last_inv_vp @ clip
        if abs(world_h[3]) < 1e-12:
            return world_h[:3]
        return world_h[:3] / world_h[3]


class EmbeddedVisualizerProxy:
    """
    Worker-side replacement for `icub_pybullet.visualizer.Visualizer`.
    Streams scene manifest and 30-60 FPS compact state updates to `SimProcessSupervisor`.
    """

    def __init__(
        self,
        client,
        event_queue: "mp.Queue",
        runtime_cfg: Optional[Dict[str, Any]] = None,
    ):
        """Build the initial scene manifest, optional skin/eye state, and emit ``scene_init``.

        :param client: Active ``pyCub`` simulation client instance.
        :param event_queue: Queue used to stream scene and frame payloads to the UI.
        :param runtime_cfg: Optional shared mutable dictionary for dynamic UI flags.
        """
        self.client = client
        self._event_queue = event_queue
        self._runtime_cfg = runtime_cfg if runtime_cfg is not None else {}
        self.file_dir = client.file_dir

        self.vis = _EmbeddedSceneProxy(self)
        self.R_urdf: Dict[str, np.ndarray] = {}
        self._mesh_centers: Dict[str, np.ndarray] = {}
        self._mesh_scales: Dict[str, float] = {}
        self._fpath_to_skin: Dict[str, Optional[str]] = {}
        self.eye_windows: Dict[str, EmbeddedEyeWindow] = {}

        self._R = np.eye(4, dtype=np.float64)
        self._R_ori = self._R[:3, :3]

        self.positions: List[float] = []
        self.orientations: List[float] = []
        self.colors: List[float] = []
        self.paths: List[str] = []

        self.pycub_alpha = float(getattr(self.client.config.gui, "pycub_alpha", 1.0))

        manifest_meshes: List[Dict[str, Any]] = []

        # 1. Robot meshes & bounding-box default center
        self.read_info(self.client.robot)
        robot_entries = self._register_urdf_meshes("robot", is_robot=True)
        self._update_transforms_dict()

        all_mins: List[np.ndarray] = []
        all_maxs: List[np.ndarray] = []
        for entry in robot_entries:
            key = entry["key"]
            tf = self.vis.get_geometry_transform(key)
            entry["transform"] = tf.astype(np.float32).tolist()
            md = entry.pop("_mesh_data")
            # Transform AABB corners to compute default_center identically to Visualizer.__init__
            bmin, bmax = md.bbox_min, md.bbox_max
            corners = np.array(
                [
                    [bmin[0], bmin[1], bmin[2], 1.0],
                    [bmax[0], bmin[1], bmin[2], 1.0],
                    [bmin[0], bmax[1], bmin[2], 1.0],
                    [bmin[0], bmin[1], bmax[2], 1.0],
                    [bmax[0], bmax[1], bmin[2], 1.0],
                    [bmax[0], bmin[1], bmax[2], 1.0],
                    [bmin[0], bmax[1], bmax[2], 1.0],
                    [bmax[0], bmax[1], bmax[2], 1.0],
                ],
                dtype=np.float64,
            )
            world_corners = (corners @ tf.T)[:, :3]
            all_mins.append(np.min(world_corners, axis=0))
            all_maxs.append(np.max(world_corners, axis=0))
            manifest_meshes.append(entry)

        if all_mins:
            scene_min = np.min(np.vstack(all_mins), axis=0)
            scene_max = np.max(np.vstack(all_maxs), axis=0)
            self.default_center = 0.5 * (scene_min + scene_max)
        else:
            self.default_center = np.zeros(3, dtype=np.float64)

        # 2. Other scene objects (plane, table, ball, custom user meshes, etc.)
        floor_z: Optional[float] = None
        for obj_id, obj_name, _, _, _ in self.client.other_objects:
            self.read_info(obj_id)
            obj_entries = self._register_urdf_meshes(obj_name, is_robot=False)
            self._update_transforms_dict()
            for entry in obj_entries:
                key = entry["key"]
                tf = self.vis.get_geometry_transform(key)
                entry["transform"] = tf.astype(np.float32).tolist()
                md = entry.pop("_mesh_data")
                if "plane" in obj_name.lower():
                    floor_z = float((tf @ np.array([0.0, 0.0, md.bbox_max[2], 1.0]))[2])
                manifest_meshes.append(entry)

        # 3. Optional tactile skin point clouds
        skin_parts_manifest: List[Dict[str, str]] = []
        if self.client.config.skin.use:
            cfg_txt = os.path.join(self.file_dir, "iCub", "skin", "point_clouds", "config.txt")
            with open(cfg_txt, "r", encoding="utf-8") as f:
                skin_map = {ln.split(";")[0]: ln.split(";")[1] for ln in f.read().splitlines() if ";" in ln}
            if len(self.client.config.skin.skin_parts) == 0:
                pcd_paths = sorted(glob.glob(os.path.join(self.file_dir, "iCub", "skin", "point_clouds", "*.pcd")))
            else:
                pcd_paths = [
                    os.path.join(self.file_dir, "iCub", "skin", "point_clouds", f"{sp}.pcd")
                    for sp in self.client.config.skin.skin_parts
                ]
            for pcd_p in pcd_paths:
                stem = os.path.basename(pcd_p).split(".")[0]
                if stem in skin_map:
                    skin_parts_manifest.append({"part": skin_map[stem], "pcd_path": pcd_p})

        # 4. Eye cameras enabled in YAML config or requested dynamically by UI
        if self.client.config.eyes.l_eye:
            self.enable_eye("l_eye", True)
        if self.client.config.eyes.r_eye:
            self.enable_eye("r_eye", True)
        for req_eye in self._runtime_cfg.get("enabled_eyes", ()):
            self.enable_eye(req_eye, True)

        self.is_alive = True
        self._last_eye_stream_ts = 0.0

        # Emit initial scene manifest to UI
        self._put_event(
            {
                "type": "scene_init",
                "default_center": self.default_center.tolist(),
                "floor_z": floor_z,
                "meshes": manifest_meshes,
                "skin_parts": skin_parts_manifest,
            },
            drop_if_full=False,
        )

        # Capture initial eye images immediately if eyes are enabled so `get_camera_images` is never None
        if self.eye_windows:
            self._capture_eyes_and_emit(force_emit=True)

    def enable_eye(self, eye: str, enabled: bool = True) -> None:
        """Enable or disable offscreen rendering for ``eye`` (``'l_eye'`` or ``'r_eye'``).

        :param eye: Eye identifier (``'l_eye'`` or ``'r_eye'``).
        :param enabled: Whether the eye camera should be active.
        """
        if enabled:
            if eye not in self.eye_windows:
                self.eye_windows[eye] = EmbeddedEyeWindow(eye, self)
                self._refresh_eye_poses()
                self.eye_windows[eye].capture()
        else:
            if not getattr(self.client.config.eyes, eye, False):
                self.eye_windows.pop(eye, None)

    def reset_view(self) -> None:
        """Emit a ``reset_view`` event to recenter the UI OpenGL camera."""
        self._put_event({"type": "reset_view"}, drop_if_full=False)

    def _put_event(self, payload: Dict[str, Any], drop_if_full: bool = True) -> None:
        """Push an event payload onto the queue, optionally dropping when full."""
        if self._event_queue is None:
            return
        try:
            if drop_if_full:
                self._event_queue.put_nowait(payload)
            else:
                self._event_queue.put(payload, timeout=2.0)
        except queue.Full:
            pass
        except Exception:
            pass

    def _find_xyz_rpy(self, mesh_name: str, urdf_name: str = "robot") -> Tuple[list, list, float, str]:
        """Look up visual origin, scale, and link name for ``mesh_name`` in ``urdf_name``."""
        for link in self.client.urdfs[urdf_name].links:
            if hasattr(link, "visual"):
                fn = link.visual.geometry.mesh.filename
                if os.path.basename(fn) == mesh_name:
                    xyz = link.visual.origin.xyz
                    rpy = link.visual.origin.rpy
                    scale = link.visual.geometry.mesh.scale[0] if hasattr(link.visual.geometry.mesh, "scale") else 1.0
                    return xyz, rpy, float(scale), link.name
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1.0, ""

    def _register_urdf_meshes(self, urdf_name: str, is_robot: bool) -> List[Dict[str, Any]]:
        """Register URDF visual meshes, compute initial transforms/bounds, and return manifest entries."""
        entries: List[Dict[str, Any]] = []
        alpha = self.pycub_alpha if is_robot else 1.0
        for mesh_id in range(len(self.positions) // 3):
            col = self.colors[mesh_id * 3 : (mesh_id + 1) * 3]
            f_path = self.paths[mesh_id]
            init_xyz, init_rpy, scale, link_name = self._find_xyz_rpy(os.path.basename(f_path), urdf_name=urdf_name)

            r_urdf = np.eye(4, dtype=np.float64)
            r_urdf[:3, :3] = np.reshape(
                self.client.getMatrixFromQuaternion(self.client.getQuaternionFromEuler(init_rpy)),
                (3, 3),
            )
            r_urdf[:3, 3] = init_xyz
            self.R_urdf[f_path] = r_urdf
            self._mesh_scales[f_path] = scale

            md = load_obj_bounds(f_path, scale=scale)
            self._mesh_centers[f_path] = md.center

            if self.client.config.skin.use and is_robot:
                self._fpath_to_skin[f_path] = link_name if link_name in self.client.skin else None
            else:
                self._fpath_to_skin[f_path] = None

            entries.append(
                {
                    "key": f_path,
                    "path": f_path,
                    "scale": scale,
                    "color": [float(col[0]), float(col[1]), float(col[2]), alpha],
                    "is_robot": is_robot,
                    "_mesh_data": md,
                }
            )
        return entries

    def read_info(self, obj_id: int) -> int:
        """Query visual shape poses, colors, and mesh paths from PyBullet for ``obj_id``.

        :param obj_id: PyBullet body unique ID.
        """
        visual_data = self.client.getVisualShapeData(obj_id)
        num_j = self.client.getNumJoints(obj_id)
        link_states = (
            self.client.getLinkStates(obj_id, range(num_j), computeLinkVelocity=0, computeForwardKinematics=0)
            if num_j > 0
            else []
        )
        self.positions = []
        self.orientations = []
        self.colors = []
        self.paths = []

        for m in visual_data:
            f_path = m[self.client.visualShapeData["FILE"]].decode("utf-8")
            if not f_path:
                continue
            col = m[self.client.visualShapeData["COLOR"]]
            link = m[self.client.visualShapeData["LINK"]]
            if link != -1:
                ls = link_states[link]
                ori = ls[self.client.linkInfo["URDFORI"]]
                pos = ls[self.client.linkInfo["URDFPOS"]]
            else:
                pos, ori = self.client.getBasePositionAndOrientation(obj_id)
            self.positions.extend(pos)
            self.orientations.extend(ori)
            self.colors.extend(col[:-1])
            self.paths.append(f_path)
        return 0

    def _update_transforms_dict(self) -> Tuple[Dict[str, list], Dict[str, list]]:
        """Compute world transforms for meshes, eye cameras, and tactile skin links."""
        mesh_tfs: Dict[str, list] = {}
        skin_tfs: Dict[str, list] = {}
        for mesh_id in range(len(self.positions) // 3):
            pos = self.positions[mesh_id * 3 : (mesh_id + 1) * 3]
            ori = self.orientations[mesh_id * 4 : (mesh_id + 1) * 4]
            f_path = self.paths[mesh_id]
            r_urdf = self.R_urdf[f_path]

            self._R_ori.flat[:] = self.client.getMatrixFromQuaternion(ori)
            self._R[:3, 3] = pos
            tf = self._R @ r_urdf
            self.vis.set_geometry_transform(f_path, tf)
            mesh_tfs[f_path] = tf.astype(np.float32).tolist()

            for ew in self.eye_windows.values():
                if ew.link_name in f_path:
                    c_local = self._mesh_centers.get(f_path, np.zeros(3))
                    ew.center = (tf @ np.hstack((c_local, 1.0)))[:3]
                    rot_only = tf[:3, :3]
                    d_vec = rot_only @ np.array([0.0, 0.0, 1.0])
                    dn = np.linalg.norm(d_vec)
                    ew.dir = (0.1 * d_vec / dn) if dn > 1e-9 else np.array([-0.1, 0.0, 0.0])

            skin_part = self._fpath_to_skin.get(f_path)
            if skin_part:
                skin_tfs[skin_part] = self._R.astype(np.float32).tolist()
        return mesh_tfs, skin_tfs

    def _refresh_eye_poses(self) -> None:
        """Re-read robot link states and refresh eye camera positions and directions."""
        self.read_info(self.client.robot)
        self._update_transforms_dict()

    def _capture_eyes_and_emit(self, force_emit: bool = False) -> None:
        """Capture active eye cameras and emit RGB/depth payloads at up to 10 Hz."""
        for ew in self.eye_windows.values():
            ew.capture()
        now = time.time()
        if force_emit or (now - self._last_eye_stream_ts >= 0.10):
            self._last_eye_stream_ts = now
            eyes_payload = {
                eye: {"rgb": ew.last_image, "depth": ew.last_depth_image}
                for eye, ew in self.eye_windows.items()
            }
            self._put_event({"type": "eyes", "eyes": eyes_payload}, drop_if_full=True)

    def render(self) -> None:
        """Sync eye toggles, update scene transforms/skin/lines, and stream frame and eye events."""
        if self._runtime_cfg.get("stop_requested"):
            self.is_alive = False
            return

        # Sync dynamically toggled eye cameras from UI
        desired_eyes: Set[str] = set(self._runtime_cfg.get("enabled_eyes", ()))
        for eye_name in ("l_eye", "r_eye"):
            if eye_name in desired_eyes:
                self.enable_eye(eye_name, True)
            elif eye_name in self.eye_windows and not getattr(self.client.config.eyes, eye_name, False):
                self.enable_eye(eye_name, False)

        all_tfs: Dict[str, list] = {}
        all_skin_tfs: Dict[str, list] = {}

        self.read_info(self.client.robot)
        tfs, stfs = self._update_transforms_dict()
        all_tfs.update(tfs)
        all_skin_tfs.update(stfs)

        for obj_id, _, _, _, _ in self.client.other_objects:
            self.read_info(obj_id)
            otfs, _ = self._update_transforms_dict()
            all_tfs.update(otfs)

        skin_active: Optional[Dict[str, List[int]]] = None
        if self.client.config.skin.use:
            skin_active = {}
            for part, acts in self.client.skin_activations.items():
                idxs = np.flatnonzero(acts > 0)
                if idxs.size > 0:
                    skin_active[part] = idxs.astype(int).tolist()

        lines_payload: Optional[Dict[str, Any]] = None
        if self.vis.lines_dirty:
            lines_payload = dict(self.vis.custom_lines)
            self.vis.lines_dirty = False

        self._put_event(
            {
                "type": "frame",
                "transforms": all_tfs,
                "skin_transforms": all_skin_tfs if all_skin_tfs else None,
                "skin_active": skin_active,
                "lines": lines_payload,
            },
            drop_if_full=True,
        )

        if self.eye_windows:
            self._capture_eyes_and_emit(force_emit=False)


# =================================================================================================
# Stream Redirector for Worker Console Output
# =================================================================================================

class _QueueStream(io.TextIOBase):
    """Text I/O stream that forwards written console text into a worker event queue."""

    def __init__(self, event_queue: "mp.Queue", stream_name: str = "stdout"):
        """Initialize the stream wrapper targeting ``event_queue`` and ``stream_name``."""
        super().__init__()
        self._q = event_queue
        self._stream = stream_name

    def write(self, s: str) -> int:
        """Forward non-empty text ``s`` as a console event and return character count."""
        if s:
            try:
                self._q.put_nowait({"type": "console", "stream": self._stream, "text": s})
            except Exception:
                pass
        return len(s)

    def flush(self) -> None:
        """Implement required ``TextIOBase.flush`` as a no-op."""
        pass


class _QueueLogHandler(logging.Handler):
    """Logging handler that routes formatted records into a worker event queue."""

    def __init__(self, event_queue: "mp.Queue"):
        """Initialize the log handler targeting ``event_queue``."""
        super().__init__()
        self._q = event_queue

    def emit(self, record: logging.LogRecord) -> None:
        """Format ``record`` and enqueue it as a stdout or stderr console event."""
        try:
            msg = self.format(record) + "\n"
            stream = "stderr" if record.levelno >= logging.WARNING else "stdout"
            self._q.put_nowait({"type": "console", "stream": stream, "text": msg})
        except Exception:
            pass


# =================================================================================================
# Worker Entrypoints (top-level picklable functions for `spawn` multiprocessing)
# =================================================================================================

def _compute_ball_telemetry(
    client,
    ball_id: Optional[int],
    table_id: Optional[int],
    dt: float,
    tracker: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    """Compute ball-to-table distance, hit count, post-hit distance, and motion duration."""
    if ball_id is None:
        return None
    dist = float("nan")
    if table_id is not None:
        pts = client.getClosestPoints(ball_id, table_id, 10.0)
        dist = min(p[8] for p in pts) if pts else float("nan")
        steps = client.steps_done
        contact = bool(client.getContactPoints(client.robot, ball_id))
        if contact:
            last_cs = tracker["last_contact_step"]
            if not tracker["in_contact"] and (last_cs is None or (steps - last_cs) * dt >= 0.5):
                tracker["hits"] += 1
            tracker["in_contact"] = True
            tracker["last_contact_step"] = steps
            tracker["after_3s"] = float("nan")
            tracker["moving_time"] = 0.0
        else:
            tracker["in_contact"] = False
            last_cs = tracker["last_contact_step"]
            if last_cs is not None and math.isnan(tracker["after_3s"]) and (steps - last_cs) * dt >= 3.0:
                tracker["after_3s"] = dist
            if last_cs is not None:
                lin, ang = client.getBaseVelocity(ball_id)
                if np.linalg.norm(lin) > 0.01 or np.linalg.norm(ang) > 0.1:
                    tracker["moving_time"] = (steps - last_cs) * dt

    def _cm(v: float) -> Optional[float]:
        """Convert meters to rounded centimeters, returning ``None`` for ``NaN``."""
        return None if math.isnan(v) else round(v * 100.0, 1)

    mv = tracker["moving_time"]
    return {
        "distance": _cm(dist),
        "after": _cm(tracker["after_3s"]),
        "touched": bool(tracker["last_contact_step"] is not None),
        "moving": None if math.isnan(mv) else round(mv, 2),
        "hits": int(tracker["hits"]),
    }


def _inject_extra_python_paths() -> None:
    """Expose user's `~/Documents/pyCub/extra_packages` and configured venv site-packages."""
    for p in effective_extra_paths():
        if p not in sys.path:
            site.addsitedir(p)




def interactive_worker_main(
    initial_world: str,
    cmd_queue: "mp.Queue",
    event_queue: "mp.Queue",
    initial_eyes: List[str],
) -> None:
    """
    Persistent interactive physics worker for Modes 1, 2, 3, and Mode 4 idle preview.
    Supports instant button presets, joint jogs, world switching, eye toggles, and
    cooperative threaded execution of Kid Scratch / Kid Programming code.
    """
    os.environ["PYCUB_EMBEDDED_VIEWER"] = "1"
    os.environ["PYCUB_DISABLE_KILL_OPEN3D"] = "1"
    _inject_extra_python_paths()

    from studio.kid_ui.kid_ui import ACTIONS, jog_wait
    from icub_pybullet.pycub import pyCub, set_status_callback, set_visualizer_factory
    import icub_pybullet.kid_api as kid_api

    runtime_cfg: Dict[str, Any] = {"enabled_eyes": set(initial_eyes), "stop_requested": False}

    set_status_callback(
        lambda stage, msg: event_queue.put({"type": "status", "state": "loading", "detail": msg})
    )
    set_visualizer_factory(lambda c: EmbeddedVisualizerProxy(c, event_queue, runtime_cfg))

    # Redirect print() output from Kid Scratch / Kid Programming scripts
    orig_stdout, orig_stderr = sys.stdout, sys.stderr
    sys.stdout = _QueueStream(event_queue, "stdout")
    sys.stderr = _QueueStream(event_queue, "stderr")

    shutdown_all = False
    next_world: Optional[str] = initial_world

    try:
        while not shutdown_all and next_world is not None:
            world_to_load = next_world
            next_world = None
            runtime_cfg["stop_requested"] = False

            event_queue.put({"type": "status", "state": "loading", "world": world_to_load, "detail": ""})
            cfg_path = prepare_world_config(world_to_load)
            try:
                client = pyCub(config=cfg_path)
            except Exception:
                tb = traceback.format_exc()
                event_queue.put({"type": "status", "state": "error", "detail": tb})
                # Wait for next load_world or shutdown command
                while True:
                    cmd = cmd_queue.get()
                    if not isinstance(cmd, dict):
                        continue
                    if cmd.get("cmd") == "shutdown":
                        shutdown_all = True
                        break
                    if cmd.get("cmd") == "load_world":
                        next_world = str(cmd["world"])
                        break
                continue
            finally:
                if os.path.exists(cfg_path):
                    try:
                        os.unlink(cfg_path)
                    except OSError:
                        pass

            ball_id = next((o[0] for o in client.other_objects if o[1] == "ball"), None)
            table_id = next((o[0] for o in client.other_objects if o[1] == "table"), None)
            dt = client.getPhysicsEngineParameters()["fixedTimeStep"]
            ball_tracker: Dict[str, Any] = {
                "last_contact_step": None,
                "after_3s": float("nan"),
                "moving_time": float("nan"),
                "hits": 0,
                "in_contact": False,
            }
            latest_ball_dict: Dict[str, Any] = {}
            last_ball_emit = 0.0

            current_gen: Optional[Generator] = None
            current_name: Optional[str] = None
            jogging: Set[str] = set()

            # Kid code background thread state (physics stays ticking on main loop!)
            kid_stop_event = threading.Event()
            kid_thread: Optional[threading.Thread] = None

            def _stop_kid_thread() -> None:
                """Signal any running Kid script thread to stop and wait briefly for it to exit."""
                nonlocal kid_thread
                if kid_thread is not None and kid_thread.is_alive():
                    kid_stop_event.set()
                    kid_thread.join(timeout=1.5)
                kid_thread = None
                kid_stop_event.clear()

            def _start_kid_code(code_str: str, filename: str = "<kid_program>") -> None:
                """Stop prior motions and execute Kid Python code on a background daemon thread."""
                nonlocal kid_thread, current_gen, current_name
                _stop_kid_thread()
                current_gen, current_name = None, None
                client.stop_robot()
                kid_stop_event.clear()

                def _runner() -> None:
                    """Run compiled Kid user code with ``kid_api`` bindings and emit completion status."""
                    controller = kid_api.KidRobot(
                        client=client,
                        step_fn=lambda: time.sleep(0.01),
                        stop_event=kid_stop_event,
                        highlight_cb=lambda bid: event_queue.put({"type": "block_highlight", "block_id": bid}),
                        telemetry_getter=lambda: dict(latest_ball_dict),
                    )
                    kid_api._bind_active_controller(controller)
                    ns: Dict[str, Any] = {"__name__": "__main__", "math": math, "random": __import__("random")}
                    for sym in kid_api.__all__:
                        ns[sym] = getattr(kid_api, sym)
                    ns["_highlight_block"] = kid_api._highlight_block

                    event_queue.put({"type": "kid_script_state", "running": True})
                    exit_status = "ok"
                    err_info = None
                    try:
                        compiled = compile(code_str, filename, "exec")
                        exec(compiled, ns, ns)
                    except kid_api.KidScriptStopped:
                        exit_status = "stopped"
                    except Exception as exc:
                        exit_status = "error"
                        tb_lines = traceback.extract_tb(exc.__traceback__)
                        lineno = None
                        for frame in reversed(tb_lines):
                            if frame.filename == filename:
                                lineno = frame.lineno
                                break
                        if lineno is None and isinstance(exc, SyntaxError):
                            lineno = exc.lineno
                        err_info = {
                            "type": type(exc).__name__,
                            "message": str(exc),
                            "lineno": lineno,
                            "traceback": traceback.format_exc(),
                        }
                        sys.stderr.write(traceback.format_exc())
                    finally:
                        try:
                            client.stop_robot()
                        except Exception:
                            pass
                        kid_api._bind_active_controller(None)
                        event_queue.put({"type": "block_highlight", "block_id": None})
                        event_queue.put(
                            {
                                "type": "kid_script_state",
                                "running": False,
                                "status": exit_status,
                                "error": err_info,
                            }
                        )

                kid_thread = threading.Thread(target=_runner, daemon=True)
                kid_thread.start()

            event_queue.put({"type": "status", "state": "ready", "world": world_to_load, "detail": ""})

            # Main simulation loop for the loaded world
            while client.is_alive() and not shutdown_all and next_world is None:
                try:
                    while True:
                        msg = cmd_queue.get_nowait()
                        if not isinstance(msg, dict):
                            continue
                        kind = msg.get("cmd")
                        if kind == "shutdown":
                            shutdown_all = True
                            break
                        elif kind == "load_world":
                            next_world = str(msg["world"])
                            break
                        elif kind == "reset_view":
                            client.visualizer.reset_view()
                        elif kind == "set_eyes":
                            runtime_cfg["enabled_eyes"] = set(msg.get("eyes", []))
                        elif kind == "stop":
                            _stop_kid_thread()
                            client.stop_robot()
                            current_gen, current_name = None, None
                            jogging.clear()
                            event_queue.put({"type": "status", "state": "ready", "world": world_to_load, "detail": ""})
                        elif kind == "preset":
                            _stop_kid_thread()
                            action_name = str(msg["name"])
                            if action_name in ACTIONS:
                                client.stop_robot()
                                jogging.clear()
                                current_gen = ACTIONS[action_name](client)
                                current_name = action_name
                                event_queue.put(
                                    {"type": "status", "state": "busy", "action": action_name, "world": world_to_load}
                                )
                        elif kind == "jog":
                            _stop_kid_thread()
                            joint = str(msg["joint"])
                            delta = float(msg["delta"])
                            speed = float(msg.get("speed", 1.0))
                            if current_name != "jog":
                                client.stop_robot()
                                jogging.clear()
                            _, joint_id = client.find_joint_id(joint)
                            j_state = client.joints[joint_id]
                            base = (
                                j_state.set_point
                                if isinstance(j_state.set_point, float)
                                else client.get_joint_state(joint)[0]
                            )
                            target = float(np.clip(base + np.deg2rad(delta), j_state.lower_limit, j_state.upper_limit))
                            client.move_position(
                                joint, target, wait=False, velocity=speed, timeout=8.0, check_collision=False
                            )
                            jogging.add(joint)
                            if current_name != "jog":
                                current_name = "jog"
                                current_gen = jog_wait(client, jogging)
                                event_queue.put(
                                    {"type": "status", "state": "busy", "action": "jog", "world": world_to_load}
                                )
                        elif kind == "run_kid_code":
                            _start_kid_code(str(msg.get("code", "")), str(msg.get("filename", "<kid_program>")))
                        elif kind == "stop_kid_code":
                            _stop_kid_thread()
                            client.stop_robot()
                except queue.Empty:
                    pass

                if shutdown_all or next_world is not None:
                    break

                if current_gen is not None:
                    try:
                        next(current_gen)
                    except StopIteration:
                        current_gen, current_name = None, None
                        event_queue.put({"type": "status", "state": "ready", "world": world_to_load, "detail": ""})

                client.update_simulation(0.01)

                now = time.time()
                if ball_id is not None and (now - last_ball_emit >= 0.08):
                    last_ball_emit = now
                    b_info = _compute_ball_telemetry(client, ball_id, table_id, dt, ball_tracker)
                    if b_info is not None:
                        latest_ball_dict.clear()
                        latest_ball_dict.update(b_info)
                        event_queue.put({"type": "ball", "ball": b_info})

                time.sleep(0.002)

            _stop_kid_thread()
            try:
                client.disconnect()
            except Exception:
                pass
    finally:
        sys.stdout = orig_stdout
        sys.stderr = orig_stderr
        try:
            event_queue.put({"type": "status", "state": "stopped"})
        except Exception:
            pass


def script_worker_main(
    script_path: str,
    working_dir: str,
    script_args: List[str],
    cmd_queue: "mp.Queue",
    event_queue: "mp.Queue",
    initial_eyes: List[str],
) -> None:
    """
    Isolated Python script runner for Mode 4 (Normal IDE, Examples, and Exercise Testers).
    Intercepts `pyCub` visualizer creation so any `pyCub(...)` inside the user's `.py` script
    renders natively inside Mode 4's embedded OpenGL viewport!
    """
    os.environ["PYCUB_EMBEDDED_VIEWER"] = "1"
    os.environ["PYCUB_DISABLE_KILL_OPEN3D"] = "1"
    os.environ.setdefault("MPLBACKEND", "Agg")
    _inject_extra_python_paths()

    from icub_pybullet.pycub import set_status_callback, set_visualizer_factory

    runtime_cfg: Dict[str, Any] = {"enabled_eyes": set(initial_eyes), "stop_requested": False}

    def _poll_commands() -> None:
        """Drain pending supervisor commands to update stop flag and enabled eye cameras."""
        while True:
            try:
                msg = cmd_queue.get_nowait()
            except queue.Empty:
                break
            if isinstance(msg, dict):
                if msg.get("cmd") in ("stop", "shutdown"):
                    runtime_cfg["stop_requested"] = True
                elif msg.get("cmd") == "set_eyes":
                    runtime_cfg["enabled_eyes"] = set(msg.get("eyes", []))

    def _make_proxy(client):
        """Instantiate an :class:`EmbeddedVisualizerProxy` whose ``render`` polls commands."""
        _poll_commands()
        proxy = EmbeddedVisualizerProxy(client, event_queue, runtime_cfg)
        orig_render = proxy.render

        def _wrapped_render():
            """Poll queued commands before delegating to the proxy's ``render`` method."""
            _poll_commands()
            orig_render()

        proxy.render = _wrapped_render
        return proxy

    set_status_callback(
        lambda stage, msg: event_queue.put({"type": "status", "state": "loading", "detail": msg})
    )
    set_visualizer_factory(_make_proxy)

    orig_stdout, orig_stderr = sys.stdout, sys.stderr
    sys.stdout = _QueueStream(event_queue, "stdout")
    sys.stderr = _QueueStream(event_queue, "stderr")

    pycub_logger = logging.getLogger("pycub_logger")
    q_handler = _QueueLogHandler(event_queue)
    q_handler.setFormatter(logging.Formatter("%(module)s %(levelname)s: %(message)s"))
    pycub_logger.addHandler(q_handler)

    exit_code = 0
    try:
        abs_script = os.path.abspath(script_path)
        cwd = os.path.abspath(working_dir) if working_dir else os.path.dirname(abs_script)
        os.chdir(cwd)
        if cwd not in sys.path:
            sys.path.insert(0, cwd)
        script_dir = os.path.dirname(abs_script)
        if script_dir not in sys.path:
            sys.path.insert(0, script_dir)

        sys.argv = [abs_script] + list(script_args)
        event_queue.put({"type": "status", "state": "running_script", "detail": os.path.basename(abs_script)})
        runpy.run_path(abs_script, run_name="__main__")
    except SystemExit as exc:
        if isinstance(exc.code, int):
            exit_code = exc.code
        elif exc.code:
            sys.stderr.write(f"{exc.code}\n")
            exit_code = 1
    except Exception:
        exit_code = 1
        sys.stderr.write(traceback.format_exc())
    finally:
        pycub_logger.removeHandler(q_handler)
        sys.stdout = orig_stdout
        sys.stderr = orig_stderr
        try:
            event_queue.put({"type": "script_finished", "exit_code": exit_code})
        except Exception:
            pass


class _ConnEventSender:
    """Thread-safe queue.Queue lookalike sending worker events over a multiprocessing Connection."""

    def __init__(self, conn):
        """Initialize the thread-safe sender around multiprocessing connection ``conn``."""
        self._conn = conn
        self._lock = threading.Lock()

    def put(self, item: Any, *_args, **_kwargs) -> None:
        """Send ``item`` over the IPC connection under a thread lock."""
        with self._lock:
            try:
                self._conn.send(item)
            except Exception:
                pass

    def put_nowait(self, item: Any) -> None:
        """Alias for :meth:`put` matching ``queue.Queue.put_nowait``."""
        self.put(item)


def external_script_worker_entry(
    ipc_port: int,
    authkey_hex: str,
    script_path: str,
    working_dir: str,
    script_args: List[str],
    initial_eyes: List[str],
) -> None:
    """Entrypoint invoked by an external custom Python interpreter to run a script with embedded 3D."""
    conn = Client(("127.0.0.1", int(ipc_port)), authkey=bytes.fromhex(authkey_hex))
    cmd_q: queue.Queue = queue.Queue()
    evt_sender = _ConnEventSender(conn)

    def _cmd_reader() -> None:
        """Continuously receive commands from the IPC connection until closed."""
        try:
            while True:
                msg = conn.recv()
                cmd_q.put(msg)
        except (EOFError, OSError):
            cmd_q.put({"cmd": "shutdown"})

    reader = threading.Thread(target=_cmd_reader, daemon=True)
    reader.start()

    try:
        script_worker_main(
            script_path=script_path,
            working_dir=working_dir,
            script_args=script_args,
            cmd_queue=cmd_q,
            event_queue=evt_sender,
            initial_eyes=initial_eyes,
        )
    finally:
        try:
            conn.close()
        except Exception:
            pass


# =================================================================================================
# Qt-side Supervisor (SimProcessSupervisor)
# =================================================================================================

if HAS_QT:

    class SimProcessSupervisor(QObject):
        """
        Manages the spawned simulation / script worker process and dispatches worker events
        as thread-safe Qt signals at 60 Hz on the GUI thread.
        """

        sceneInitialized = Signal(dict)
        frameReady = Signal(dict)
        eyesUpdated = Signal(dict)
        statusChanged = Signal(dict)
        ballUpdated = Signal(dict)
        consoleOutput = Signal(str, str)        # (stream, text)
        blockHighlighted = Signal(object)       # str | None
        kidScriptStateChanged = Signal(dict)
        scriptFinished = Signal(int)
        resetViewRequested = Signal()

        def __init__(self, parent: Optional[QObject] = None):
            """Initialize worker process handles, IPC state, and the 60 Hz event poll timer."""
            super().__init__(parent)
            self._ctx = mp.get_context("spawn")
            self._proc: Optional[mp.Process] = None
            self._cmd_q: Optional[mp.Queue] = None
            self._evt_q: Optional[mp.Queue] = None
            self._ext_proc: Optional[subprocess.Popen] = None
            self._ext_listener: Optional[Listener] = None
            self._ext_conn: Any = None
            self._ext_lock = threading.Lock()
            self._enabled_eyes: Set[str] = set()
            self._mode: str = "stopped"
            self._current_world: str = "ball"

            self._poll_timer = QTimer(self)
            self._poll_timer.setInterval(16)  # ~60 FPS drain
            self._poll_timer.timeout.connect(self._drain_events)

        @property
        def current_world(self) -> str:
            """Return the identifier or path of the currently configured interactive world."""
            return self._current_world

        def is_running(self) -> bool:
            """Return True if either the spawned or external worker process is alive."""
            if self._ext_proc is not None:
                return self._ext_proc.poll() is None
            return self._proc is not None and self._proc.is_alive()

        def stop_worker(self) -> None:
            """Shut down any active worker process, terminate lingering child trees, and reset state."""
            self._poll_timer.stop()
            if self._proc is not None:
                if self._cmd_q is not None:
                    try:
                        self._cmd_q.put_nowait({"cmd": "shutdown"})
                    except Exception:
                        pass
                if self._proc.is_alive():
                    terminate_process_tree([self._proc], timeout=3.0)
                    self._proc.join(timeout=1.5)
            if self._ext_proc is not None:
                self.send_command({"cmd": "shutdown"})
                if self._ext_proc.poll() is None:
                    terminate_process_tree([self._ext_proc], timeout=3.0)
            for handle in (self._ext_conn, self._ext_listener):
                if handle is not None:
                    try:
                        handle.close()
                    except Exception:
                        pass
            self._ext_proc = None
            self._ext_conn = None
            self._ext_listener = None
            self._proc = None
            self._cmd_q = None
            self._evt_q = None
            self._mode = "stopped"

        def start_interactive(self, world: str = "ball") -> None:
            """Start (or hot-switch if already in interactive mode) the interactive world sandbox."""
            self._current_world = world
            if self._mode == "interactive" and self.is_running() and self._cmd_q is not None:
                self._cmd_q.put({"cmd": "load_world", "world": world})
                return

            self.stop_worker()
            self._mode = "interactive"
            self._cmd_q = self._ctx.Queue()
            self._evt_q = self._ctx.Queue(maxsize=120)
            self._proc = self._ctx.Process(
                target=interactive_worker_main,
                daemon=True,
                args=(world, self._cmd_q, self._evt_q, sorted(self._enabled_eyes)),
            )
            self._proc.start()
            self._poll_timer.start()

        def start_user_script(
            self,
            script_path: str,
            working_dir: Optional[str] = None,
            args: Optional[List[str]] = None,
        ) -> None:
            """Stop any running worker and launch `script_path` in an isolated Python process."""
            self.stop_worker()
            self._mode = "script"
            cwd = working_dir or os.path.dirname(os.path.abspath(script_path))
            custom_py = load_studio_settings().custom_python_exe.strip()
            if custom_py and os.path.isfile(custom_py):
                self._start_external_user_script(custom_py, script_path, cwd, list(args or []))
                return

            self._cmd_q = self._ctx.Queue()
            self._evt_q = self._ctx.Queue(maxsize=240)
            self._proc = self._ctx.Process(
                target=script_worker_main,
                daemon=True,
                args=(script_path, cwd, list(args or []), self._cmd_q, self._evt_q, sorted(self._enabled_eyes)),
            )
            self._proc.start()
            self._poll_timer.start()

        def _start_external_user_script(
            self,
            python_exe: str,
            script_path: str,
            working_dir: str,
            script_args: List[str],
        ) -> None:
            """Launch ``script_path`` using a custom Python executable over an authenticated local IPC socket."""
            authkey = os.urandom(16)
            listener = Listener(("127.0.0.1", 0), authkey=authkey)
            port = listener.address[1]
            self._ext_listener = listener
            evt_q: queue.Queue = queue.Queue()
            self._evt_q = evt_q

            repo_or_meipass = getattr(sys, "_MEIPASS", None) or os.path.dirname(
                os.path.dirname(os.path.abspath(__file__))
            )
            env = dict(os.environ)
            existing_pp = env.get("PYTHONPATH", "")
            extra_pp = [repo_or_meipass] + effective_extra_paths()
            if existing_pp:
                extra_pp.append(existing_pp)
            env["PYTHONPATH"] = os.pathsep.join(extra_pp)
            env["PYCUB_EMBEDDED_VIEWER"] = "1"

            runner_code = (
                "import sys; "
                "from studio.sim_bridge import external_script_worker_entry; "
                "external_script_worker_entry("
                f"{port}, {authkey.hex()!r}, {os.path.abspath(script_path)!r}, "
                f"{os.path.abspath(working_dir)!r}, {script_args!r}, {sorted(self._enabled_eyes)!r}"
                ")"
            )
            self._ext_proc = subprocess.Popen(
                [python_exe, "-u", "-c", runner_code],
                env=env,
                stderr=subprocess.PIPE,
                text=True,
            )

            def _accept_and_pump() -> None:
                """Accept the external worker IPC connection and forward events until process exit."""
                try:
                    conn = listener.accept()
                    self._ext_conn = conn
                    while True:
                        ev = conn.recv()
                        evt_q.put(ev)
                except (EOFError, OSError):
                    pass
                finally:
                    if self._ext_proc is not None and self._ext_proc.stderr is not None:
                        try:
                            err_tail = self._ext_proc.stderr.read()
                            if err_tail:
                                evt_q.put({"type": "console", "stream": "stderr", "text": err_tail})
                        except Exception:
                            pass

            threading.Thread(target=_accept_and_pump, daemon=True).start()
            self._poll_timer.start()

        def send_command(self, payload: Dict[str, Any]) -> bool:
            """Send a command dictionary to the running worker process.

            :param payload: Command payload dictionary to deliver.
            """
            if not self.is_running():
                return False
            if self._ext_conn is not None:
                with self._ext_lock:
                    try:
                        self._ext_conn.send(payload)
                        return True
                    except Exception:
                        return False
            if self._cmd_q is None:
                return False
            try:
                self._cmd_q.put_nowait(payload)
                return True
            except Exception:
                return False

        def set_enabled_eyes(self, eyes: Set[str]) -> None:
            """Update the active eye camera set and notify the running worker.

            :param eyes: Set of eye names to stream (e.g. ``{'l_eye', 'r_eye'}``).
            """
            self._enabled_eyes = set(eyes)
            self.send_command({"cmd": "set_eyes", "eyes": sorted(self._enabled_eyes)})

        def run_kid_code(self, code: str, filename: str = "<kid_program>") -> bool:
            """Send Kid Python source code to the interactive worker for execution.

            :param code: Python source string to execute.
            :param filename: Virtual filename used in tracebacks.
            """
            return self.send_command({"cmd": "run_kid_code", "code": code, "filename": filename})

        def stop_kid_code(self) -> bool:
            """Request cancellation of any currently running Kid script in the interactive worker."""
            return self.send_command({"cmd": "stop_kid_code"})

        def _drain_events(self) -> None:
            """Drain queued worker events on the GUI thread, coalescing frames and emitting Qt signals."""
            if self._evt_q is None:
                return
            latest_frame: Optional[Dict[str, Any]] = None
            merged_lines: Dict[str, Any] = {}
            has_lines = False

            for _ in range(80):
                try:
                    ev = self._evt_q.get_nowait()
                except queue.Empty:
                    break
                if not isinstance(ev, dict):
                    continue
                etype = ev.get("type")
                if etype == "scene_init":
                    latest_frame = None
                    merged_lines.clear()
                    has_lines = False
                    self.sceneInitialized.emit(ev)
                elif etype == "frame":
                    if ev.get("lines") is not None:
                        merged_lines = ev["lines"]
                        has_lines = True
                    latest_frame = ev
                elif etype == "eyes":
                    self.eyesUpdated.emit(ev.get("eyes", {}))
                elif etype == "status":
                    self.statusChanged.emit(ev)
                elif etype == "ball":
                    self.ballUpdated.emit(ev.get("ball") or {})
                elif etype == "console":
                    self.consoleOutput.emit(str(ev.get("stream", "stdout")), str(ev.get("text", "")))
                elif etype == "block_highlight":
                    self.blockHighlighted.emit(ev.get("block_id"))
                elif etype == "kid_script_state":
                    self.kidScriptStateChanged.emit(ev)
                elif etype == "reset_view":
                    self.resetViewRequested.emit()
                elif etype == "script_finished":
                    if latest_frame is not None:
                        if has_lines:
                            latest_frame["lines"] = merged_lines
                        self.frameReady.emit(latest_frame)
                        latest_frame = None
                    self.scriptFinished.emit(int(ev.get("exit_code", 0)))

            if latest_frame is not None:
                if has_lines:
                    latest_frame["lines"] = merged_lines
                self.frameReady.emit(latest_frame)

            # Detect unexpected worker exit in script mode
            proc_exited = (self._proc is not None and not self._proc.is_alive()) or (
                self._ext_proc is not None and self._ext_proc.poll() is not None
            )
            if proc_exited:
                # Drain any remaining messages before stopping poll timer
                try:
                    while True:
                        ev = self._evt_q.get_nowait()
                        if isinstance(ev, dict) and ev.get("type") == "console":
                            self.consoleOutput.emit(str(ev.get("stream", "stdout")), str(ev.get("text", "")))
                        elif isinstance(ev, dict) and ev.get("type") == "script_finished":
                            self.scriptFinished.emit(int(ev.get("exit_code", 0)))
                except queue.Empty:
                    pass
                self._poll_timer.stop()
