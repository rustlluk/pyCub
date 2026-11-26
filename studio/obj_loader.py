"""
Fast, zero-Open3D-GUI Wavefront .obj and .pcd geometry loader for EmbeddedGLViewer.

Matches Open3D Visualizer.show_first() geometry scaling and centering semantics:
  mesh.scale(scale, mesh.get_center())
  mesh.translate(mesh.get_center() * scale, relative=False)
which is algebraically equivalent to scaling vertices by `scale` around the origin
and setting `center = mean(unique_vertices) * scale`.
"""
from __future__ import annotations

import hashlib
import os
import struct
import tempfile
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from icub_pybullet.runtime_paths import get_user_cache_dir


@dataclass(frozen=True)
class LoadedMeshData:
    """Immutable container holding interleaved mesh vertex/normal data and bounding metadata."""

    path: str
    scale: float
    # Interleaved float32 array of shape (N_verts, 6): [x, y, z, nx, ny, nz]
    interleaved_vn: np.ndarray
    center: np.ndarray        # shape (3,), float64
    bbox_min: np.ndarray      # shape (3,), float64
    bbox_max: np.ndarray      # shape (3,), float64
    vertex_count: int


@dataclass(frozen=True)
class MeshBounds:
    """Immutable container holding scaled mesh center and axis-aligned bounding box."""

    path: str
    scale: float
    center: np.ndarray        # shape (3,), float64
    bbox_min: np.ndarray      # shape (3,), float64
    bbox_max: np.ndarray      # shape (3,), float64


_MESH_CACHE: Dict[Tuple[str, float, int], LoadedMeshData] = {}
_BOUNDS_CACHE: Dict[Tuple[str, float, int], MeshBounds] = {}
_PCD_CACHE: Dict[Tuple[str, bool, int], np.ndarray] = {}
_MESHBIN_MAGIC = b"PMB1"
_MESHBIN_HEADER = struct.Struct("<4sQ9d")


def _mesh_cache_file(abs_path: str, scale: float, mtime_ns: int) -> str:
    """Compute the deterministic binary cache file path for a scaled mesh and mtime."""
    stem = os.path.splitext(os.path.basename(abs_path))[0]
    safe_stem = "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in stem)
    digest = hashlib.sha256(f"{abs_path}|{scale:.9g}|{mtime_ns}".encode("utf-8")).hexdigest()[:16]
    return str(get_user_cache_dir() / "mesh_cache" / f"{safe_stem}_{digest}.meshbin")


def _read_meshbin_bounds(cache_path: str, abs_path: str, scale: float) -> Optional[MeshBounds]:
    """Read only the header bounds from a cached ``.meshbin`` file, or return ``None`` if invalid."""
    if not os.path.isfile(cache_path):
        return None
    try:
        with open(cache_path, "rb") as f:
            hdr = f.read(_MESHBIN_HEADER.size)
        if len(hdr) != _MESHBIN_HEADER.size:
            return None
        magic, _vcount, cx, cy, cz, minx, miny, minz, maxx, maxy, maxz = _MESHBIN_HEADER.unpack(hdr)
        if magic != _MESHBIN_MAGIC:
            return None
        return MeshBounds(
            path=abs_path,
            scale=scale,
            center=np.array([cx, cy, cz], dtype=np.float64),
            bbox_min=np.array([minx, miny, minz], dtype=np.float64),
            bbox_max=np.array([maxx, maxy, maxz], dtype=np.float64),
        )
    except OSError:
        return None


def _read_meshbin(cache_path: str, abs_path: str, scale: float) -> Optional[LoadedMeshData]:
    """Load full interleaved mesh data from a cached ``.meshbin`` file, or return ``None`` if invalid."""
    if not os.path.isfile(cache_path):
        return None
    try:
        with open(cache_path, "rb") as f:
            raw = f.read()
        if len(raw) < _MESHBIN_HEADER.size:
            return None
        magic, vcount, cx, cy, cz, minx, miny, minz, maxx, maxy, maxz = _MESHBIN_HEADER.unpack_from(raw, 0)
        expected_bytes = _MESHBIN_HEADER.size + int(vcount) * 6 * 4
        if magic != _MESHBIN_MAGIC or len(raw) != expected_bytes:
            return None
        if vcount == 0:
            interleaved = np.zeros((0, 6), dtype=np.float32)
        else:
            interleaved = np.frombuffer(raw, dtype=np.float32, offset=_MESHBIN_HEADER.size).reshape((int(vcount), 6)).copy()
        return LoadedMeshData(
            path=abs_path,
            scale=scale,
            interleaved_vn=interleaved,
            center=np.array([cx, cy, cz], dtype=np.float64),
            bbox_min=np.array([minx, miny, minz], dtype=np.float64),
            bbox_max=np.array([maxx, maxy, maxz], dtype=np.float64),
            vertex_count=int(vcount),
        )
    except Exception:
        return None


def _write_meshbin(cache_path: str, data: LoadedMeshData) -> None:
    """Atomically write ``data`` to the ``.meshbin`` disk cache, ignoring I/O errors."""
    try:
        cache_dir = os.path.dirname(cache_path)
        os.makedirs(cache_dir, exist_ok=True)
        hdr = _MESHBIN_HEADER.pack(
            _MESHBIN_MAGIC,
            int(data.vertex_count),
            float(data.center[0]),
            float(data.center[1]),
            float(data.center[2]),
            float(data.bbox_min[0]),
            float(data.bbox_min[1]),
            float(data.bbox_min[2]),
            float(data.bbox_max[0]),
            float(data.bbox_max[1]),
            float(data.bbox_max[2]),
        )
        arr = np.ascontiguousarray(data.interleaved_vn, dtype=np.float32)
        fd, tmp_name = tempfile.mkstemp(prefix=".meshbin_", suffix=".tmp", dir=cache_dir)
        try:
            with os.fdopen(fd, "wb") as f:
                f.write(hdr)
                f.write(arr.tobytes())
            os.replace(tmp_name, cache_path)
        except Exception:
            try:
                if os.path.exists(tmp_name):
                    os.unlink(tmp_name)
            except OSError:
                pass
    except Exception:
        pass


def _parse_face_token(tok: str, num_v: int, num_vn: int) -> Tuple[int, int]:
    """Parse an OBJ face vertex token (``v/vt/vn``) into 0-based vertex and normal indices."""
    parts = tok.split("/")
    vi = int(parts[0])
    vi = (num_v + vi) if vi < 0 else (vi - 1)
    ni = -1
    if len(parts) >= 3 and parts[2]:
        raw_ni = int(parts[2])
        ni = (num_vn + raw_ni) if raw_ni < 0 else (raw_ni - 1)
    return vi, ni


def load_obj_bounds(path: str, scale: float = 1.0) -> MeshBounds:
    """
    Fast vertex-only bounds & center loader for a Wavefront .obj file.
    Avoids parsing faces/normals when only AABB and center are required (e.g. worker proxy).
    """
    abs_path = os.path.abspath(path)
    try:
        mtime_ns = os.stat(abs_path).st_mtime_ns
    except OSError:
        mtime_ns = 0

    scale_f = float(scale)
    cache_key = (abs_path, scale_f, mtime_ns)
    full_cached = _MESH_CACHE.get(cache_key)
    if full_cached is not None:
        return MeshBounds(abs_path, scale_f, full_cached.center, full_cached.bbox_min, full_cached.bbox_max)
    cached = _BOUNDS_CACHE.get(cache_key)
    if cached is not None:
        return cached

    bin_bounds = _read_meshbin_bounds(_mesh_cache_file(abs_path, scale_f, mtime_ns), abs_path, scale_f)
    if bin_bounds is not None:
        _BOUNDS_CACHE[cache_key] = bin_bounds
        return bin_bounds

    verts: List[Tuple[float, float, float]] = []
    with open(abs_path, "r", encoding="utf-8", errors="ignore") as f:
        for raw_line in f:
            if raw_line.startswith("v "):
                parts = raw_line.split()
                if len(parts) >= 4:
                    verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif raw_line[:1] in (" ", "\t"):
                stripped = raw_line.lstrip()
                if stripped.startswith("v "):
                    parts = stripped.split()
                    if len(parts) >= 4:
                        verts.append((float(parts[1]), float(parts[2]), float(parts[3])))

    if not verts:
        z3 = np.zeros(3, dtype=np.float64)
        res = MeshBounds(abs_path, scale_f, z3, z3, z3)
        _BOUNDS_CACHE[cache_key] = res
        return res

    v_arr = np.asarray(verts, dtype=np.float64) * scale_f
    res = MeshBounds(
        path=abs_path,
        scale=scale_f,
        center=np.mean(v_arr, axis=0),
        bbox_min=np.min(v_arr, axis=0),
        bbox_max=np.max(v_arr, axis=0),
    )
    _BOUNDS_CACHE[cache_key] = res
    return res


def load_obj_mesh(path: str, scale: float = 1.0) -> LoadedMeshData:
    """
    Parse a Wavefront .obj file into an interleaved float32 (pos + normal) triangle array.
    Results are cached in-process by (abspath, scale, mtime_ns).
    """
    abs_path = os.path.abspath(path)
    try:
        mtime_ns = os.stat(abs_path).st_mtime_ns
    except OSError:
        mtime_ns = 0

    scale_f = float(scale)
    cache_key = (abs_path, scale_f, mtime_ns)
    cached = _MESH_CACHE.get(cache_key)
    if cached is not None:
        return cached

    cache_file = _mesh_cache_file(abs_path, scale_f, mtime_ns)
    disk_cached = _read_meshbin(cache_file, abs_path, scale_f)
    if disk_cached is not None:
        _MESH_CACHE[cache_key] = disk_cached
        return disk_cached

    verts: List[Tuple[float, float, float]] = []
    norms: List[Tuple[float, float, float]] = []
    tri_v: List[int] = []
    tri_n: List[int] = []

    with open(abs_path, "r", encoding="utf-8", errors="ignore") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line[0] == "#":
                continue
            if line.startswith("v "):
                parts = line.split()
                if len(parts) >= 4:
                    verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif line.startswith("vn "):
                parts = line.split()
                if len(parts) >= 4:
                    norms.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif line.startswith("f "):
                tokens = line.split()[1:]
                if len(tokens) < 3:
                    continue
                nv = len(verts)
                nn = len(norms)
                v0, n0 = _parse_face_token(tokens[0], nv, nn)
                prev_v, prev_n = _parse_face_token(tokens[1], nv, nn)
                for tok in tokens[2:]:
                    cur_v, cur_n = _parse_face_token(tok, nv, nn)
                    tri_v.extend((v0, prev_v, cur_v))
                    tri_n.extend((n0, prev_n, cur_n))
                    prev_v, prev_n = cur_v, cur_n

    if not verts or not tri_v:
        empty = np.zeros((0, 6), dtype=np.float32)
        z3 = np.zeros(3, dtype=np.float64)
        res = LoadedMeshData(abs_path, scale_f, empty, z3, z3, z3, 0)
        _MESH_CACHE[cache_key] = res
        _write_meshbin(cache_file, res)
        return res

    v_arr = np.asarray(verts, dtype=np.float64) * scale_f
    center = np.mean(v_arr, axis=0)
    bbox_min = np.min(v_arr, axis=0)
    bbox_max = np.max(v_arr, axis=0)

    tri_v_arr = np.asarray(tri_v, dtype=np.int64)
    tri_n_arr = np.asarray(tri_n, dtype=np.int64)
    pos_expanded = v_arr[tri_v_arr]  # (M, 3)

    has_valid_obj_normals = len(norms) > 0 and np.all((tri_n_arr >= 0) & (tri_n_arr < len(norms)))
    if has_valid_obj_normals:
        n_arr = np.asarray(norms, dtype=np.float64)
        norm_expanded = n_arr[tri_n_arr]
        lengths = np.linalg.norm(norm_expanded, axis=1, keepdims=True)
        bad = lengths[:, 0] < 1e-9
        lengths[bad] = 1.0
        norm_expanded = norm_expanded / lengths
        if np.any(bad):
            norm_expanded[bad] = np.array([0.0, 0.0, 1.0])
    else:
        # Compute smooth vertex normals accumulated from face area vectors (matching Open3D compute_vertex_normals)
        tris = pos_expanded.reshape((-1, 3, 3))
        e1 = tris[:, 1] - tris[:, 0]
        e2 = tris[:, 2] - tris[:, 0]
        face_n = np.cross(e1, e2)
        v_normals = np.zeros_like(v_arr)
        tri_idx3 = tri_v_arr.reshape((-1, 3))
        np.add.at(v_normals, tri_idx3[:, 0], face_n)
        np.add.at(v_normals, tri_idx3[:, 1], face_n)
        np.add.at(v_normals, tri_idx3[:, 2], face_n)
        v_len = np.linalg.norm(v_normals, axis=1, keepdims=True)
        zero_mask = v_len[:, 0] < 1e-12
        v_len[zero_mask] = 1.0
        v_normals = v_normals / v_len
        if np.any(zero_mask):
            v_normals[zero_mask] = np.array([0.0, 0.0, 1.0])
        norm_expanded = v_normals[tri_v_arr]

    interleaved = np.empty((pos_expanded.shape[0], 6), dtype=np.float32)
    interleaved[:, :3] = pos_expanded.astype(np.float32)
    interleaved[:, 3:] = norm_expanded.astype(np.float32)

    res = LoadedMeshData(
        path=abs_path,
        scale=scale_f,
        interleaved_vn=interleaved,
        center=center,
        bbox_min=bbox_min,
        bbox_max=bbox_max,
        vertex_count=int(interleaved.shape[0]),
    )
    _MESH_CACHE[cache_key] = res
    _write_meshbin(cache_file, res)
    return res


def _parse_pcd_xyz_fast(abs_path: str) -> Optional[np.ndarray]:
    """Parse standard ASCII or uncompressed binary .pcd XYZ points without importing Open3D."""
    fields: List[str] = []
    sizes: List[int] = []
    types: List[str] = []
    counts: List[int] = []
    points_count = -1
    data_mode = ""
    with open(abs_path, "rb") as f:
        while True:
            line_b = f.readline()
            if not line_b:
                return None
            line = line_b.decode("ascii", errors="ignore").strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            tag = parts[0].upper()
            if tag == "FIELDS":
                fields = parts[1:]
            elif tag == "SIZE":
                sizes = [int(x) for x in parts[1:]]
            elif tag == "TYPE":
                types = [x.upper() for x in parts[1:]]
            elif tag == "COUNT":
                counts = [int(x) for x in parts[1:]]
            elif tag == "POINTS":
                points_count = int(parts[1])
            elif tag == "DATA":
                data_mode = parts[1].lower() if len(parts) > 1 else ""
                payload = f.read()
                break

    if not fields or "x" not in fields or "y" not in fields or "z" not in fields:
        return None
    ix, iy, iz = fields.index("x"), fields.index("y"), fields.index("z")
    if not counts:
        counts = [1] * len(fields)
    if len(sizes) != len(fields) or len(types) != len(fields) or len(counts) != len(fields):
        return None

    if data_mode == "ascii":
        text = payload.decode("ascii", errors="ignore").strip()
        if not text:
            return np.zeros((0, 3), dtype=np.float64)
        arr = np.loadtxt(text.splitlines(), dtype=np.float64)
        if arr.ndim == 1:
            arr = arr.reshape((1, -1))
        return arr[:, [ix, iy, iz]]

    if data_mode == "binary":
        if any(c != 1 for c in counts):
            return None
        if not all(sizes[i] == 4 and types[i] == "F" for i in (ix, iy, iz)):
            return None
        # Only fast-path when all fields are 4-byte scalars (standard x y z normal_x normal_y normal_z rgb)
        if any(s != 4 for s in sizes):
            return None
        n_cols = len(fields)
        n_pts = points_count if points_count >= 0 else len(payload) // (n_cols * 4)
        need_bytes = n_pts * n_cols * 4
        if len(payload) < need_bytes:
            return None
        arr = np.frombuffer(payload, dtype="<f4", count=n_pts * n_cols).reshape((n_pts, n_cols))
        return arr[:, [ix, iy, iz]].astype(np.float64)

    return None


def load_skin_pcd_points(pcd_path: str) -> np.ndarray:
    """
    Load 3D points from an ASCII or binary .pcd file (matching pyCub skin scaling: 1.05x around center except foot).
    Returns float32 array of shape (N, 3).
    """
    abs_path = os.path.abspath(pcd_path)
    scale_non_foot = "foot" not in os.path.basename(abs_path)
    try:
        mtime_ns = os.stat(abs_path).st_mtime_ns
    except OSError:
        mtime_ns = 0
    key = (abs_path, scale_non_foot, mtime_ns)
    if key in _PCD_CACHE:
        return _PCD_CACHE[key]

    pts64 = _parse_pcd_xyz_fast(abs_path)
    if pts64 is None:
        import open3d as o3d

        pc = o3d.io.read_point_cloud(abs_path)
        if scale_non_foot and len(pc.points) > 0:
            pc.scale(1.05, pc.get_center())
        pts = np.asarray(pc.points, dtype=np.float32).copy()
    else:
        if scale_non_foot and pts64.shape[0] > 0:
            center = np.mean(pts64, axis=0)
            pts64 = (pts64 - center) * 1.05 + center
        pts = pts64.astype(np.float32)
    _PCD_CACHE[key] = pts
    return pts
