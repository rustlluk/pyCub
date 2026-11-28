"""
Native hardware-accelerated 3D viewport (QOpenGLWidget + PyOpenGL) and EyeCameraPanel.

Designed for 60 FPS native rendering across Windows, macOS (Apple Silicon & Intel), and Linux,
including older OpenGL 2.1 / 3.3 GPUs and clean QWidget software fallback when running headless/offscreen.
"""
from __future__ import annotations

import ctypes
import math
import os
import sys
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from PySide6.QtCore import QPoint, Qt, Signal
from PySide6.QtGui import (
    QColor,
    QFont,
    QGuiApplication,
    QImage,
    QMouseEvent,
    QPainter,
    QPen,
    QPixmap,
    QPolygonF,
    QSurfaceFormat,
    QWheelEvent,
)
from PySide6.QtOpenGLWidgets import QOpenGLWidget
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from icub_pybullet.runtime_paths import ensure_user_workspace
from studio.obj_loader import LoadedMeshData, load_obj_mesh, load_skin_pcd_points

try:
    from OpenGL import GL as gl
    from OpenGL.GL import shaders as gl_shaders
    HAS_PYOPENGL = True
except Exception:  # pragma: no cover
    gl = None
    gl_shaders = None
    HAS_PYOPENGL = False


# --- Shaders (OpenGL 3.3 Core) ---
_MESH_VERT_330 = """
#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_norm;

uniform mat4 u_mvp;
uniform mat4 u_model;
uniform mat3 u_normal_mat;

out vec3 v_world_pos;
out vec3 v_norm;

void main() {
    vec4 wp = u_model * vec4(a_pos, 1.0);
    v_world_pos = wp.xyz;
    v_norm = normalize(u_normal_mat * a_norm);
    gl_Position = u_mvp * vec4(a_pos, 1.0);
}
"""

_MESH_FRAG_330 = """
#version 330 core
in vec3 v_world_pos;
in vec3 v_norm;

uniform vec4 u_color;
uniform vec3 u_cam_pos;

out vec4 fragColor;

void main() {
    vec3 N = normalize(v_norm);
    if (!gl_FrontFacing) {
        N = -N;
    }
    vec3 V = normalize(u_cam_pos - v_world_pos);
    vec3 L1 = normalize(vec3(-0.65, -0.45, 0.85));
    vec3 L2 = normalize(vec3(0.55, 0.65, 0.35));

    float diff1 = max(dot(N, L1), 0.0);
    float diff2 = max(dot(N, L2), 0.0);
    float hemi = 0.5 * N.z + 0.5;

    vec3 H = normalize(L1 + V);
    float spec = pow(max(dot(N, H), 0.0), 28.0) * 0.18;

    vec3 ambient = mix(vec3(0.30, 0.32, 0.36), vec3(0.46, 0.48, 0.50), hemi);
    vec3 lit = u_color.rgb * (ambient + 0.58 * diff1 + 0.24 * diff2) + vec3(spec);
    fragColor = vec4(clamp(lit, 0.0, 1.0), u_color.a);
}
"""

_LINE_VERT_330 = """
#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_col;

uniform mat4 u_mvp;
uniform float u_point_size;

out vec3 v_col;

void main() {
    v_col = a_col;
    gl_PointSize = u_point_size;
    gl_Position = u_mvp * vec4(a_pos, 1.0);
}
"""

_LINE_FRAG_330 = """
#version 330 core
in vec3 v_col;
uniform float u_alpha;
out vec4 fragColor;

void main() {
    fragColor = vec4(v_col, u_alpha);
}
"""

# --- Shaders (OpenGL 2.1 / GLSL 120 fallback for older GPUs) ---
_MESH_VERT_120 = """
#version 120
attribute vec3 a_pos;
attribute vec3 a_norm;

uniform mat4 u_mvp;
uniform mat4 u_model;
uniform mat3 u_normal_mat;

varying vec3 v_world_pos;
varying vec3 v_norm;

void main() {
    vec4 wp = u_model * vec4(a_pos, 1.0);
    v_world_pos = wp.xyz;
    v_norm = normalize(u_normal_mat * a_norm);
    gl_Position = u_mvp * vec4(a_pos, 1.0);
}
"""

_MESH_FRAG_120 = """
#version 120
varying vec3 v_world_pos;
varying vec3 v_norm;

uniform vec4 u_color;
uniform vec3 u_cam_pos;

void main() {
    vec3 N = normalize(v_norm);
    if (!gl_FrontFacing) {
        N = -N;
    }
    vec3 V = normalize(u_cam_pos - v_world_pos);
    vec3 L1 = normalize(vec3(-0.65, -0.45, 0.85));
    vec3 L2 = normalize(vec3(0.55, 0.65, 0.35));

    float diff1 = max(dot(N, L1), 0.0);
    float diff2 = max(dot(N, L2), 0.0);
    float hemi = 0.5 * N.z + 0.5;
    vec3 H = normalize(L1 + V);
    float spec = pow(max(dot(N, H), 0.0), 28.0) * 0.18;

    vec3 ambient = mix(vec3(0.30, 0.32, 0.36), vec3(0.46, 0.48, 0.50), hemi);
    vec3 lit = u_color.rgb * (ambient + 0.58 * diff1 + 0.24 * diff2) + vec3(spec);
    gl_FragColor = vec4(clamp(lit, 0.0, 1.0), u_color.a);
}
"""

_LINE_VERT_120 = """
#version 120
attribute vec3 a_pos;
attribute vec3 a_col;

uniform mat4 u_mvp;
uniform float u_point_size;

varying vec3 v_col;

void main() {
    v_col = a_col;
    gl_PointSize = u_point_size;
    gl_Position = u_mvp * vec4(a_pos, 1.0);
}
"""

_LINE_FRAG_120 = """
#version 120
varying vec3 v_col;
uniform float u_alpha;

void main() {
    gl_FragColor = vec4(v_col, u_alpha);
}
"""


def _perspective(fovy_deg: float, aspect: float, z_near: float, z_far: float) -> np.ndarray:
    """Build a 4x4 perspective projection matrix in OpenGL clip space."""
    f = 1.0 / math.tan(math.radians(fovy_deg) * 0.5)
    m = np.zeros((4, 4), dtype=np.float32)
    m[0, 0] = f / max(aspect, 1e-4)
    m[1, 1] = f
    m[2, 2] = (z_far + z_near) / (z_near - z_far)
    m[2, 3] = (2.0 * z_far * z_near) / (z_near - z_far)
    m[3, 2] = -1.0
    return m


def _look_at(eye: np.ndarray, target: np.ndarray, up: np.ndarray) -> np.ndarray:
    """Build a 4x4 right-handed view matrix looking from ``eye`` toward ``target``."""
    fwd = target - eye
    fn = np.linalg.norm(fwd)
    if fn < 1e-8:
        fwd = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        fwd = fwd / fn
    side = np.cross(fwd, up)
    sn = np.linalg.norm(side)
    if sn < 1e-8:
        alt_up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        side = np.cross(fwd, alt_up)
        side = side / max(np.linalg.norm(side), 1e-8)
    else:
        side = side / sn
    true_up = np.cross(side, fwd)

    m = np.eye(4, dtype=np.float32)
    m[0, :3] = side.astype(np.float32)
    m[1, :3] = true_up.astype(np.float32)
    m[2, :3] = (-fwd).astype(np.float32)
    m[0, 3] = -float(np.dot(side, eye))
    m[1, 3] = -float(np.dot(true_up, eye))
    m[2, 3] = float(np.dot(fwd, eye))
    return m


def _build_grid_vertices(extent: float = 2.0, step: float = 0.25, z: float = -0.648) -> np.ndarray:
    """Generate colored line-segment vertices for the ground reference grid."""
    lines: List[List[float]] = []
    n = int(round(extent / step))
    for i in range(-n, n + 1):
        coord = i * step
        major = (i % 4 == 0)
        col = [0.80, 0.83, 0.87] if major else [0.90, 0.92, 0.94]
        lines.append([-extent, coord, z, *col])
        lines.append([extent, coord, z, *col])
        lines.append([coord, -extent, z, *col])
        lines.append([coord, extent, z, *col])
    return np.asarray(lines, dtype=np.float32)


def _build_axes_vertices(length: float = 0.5) -> np.ndarray:
    """Generate RGB line-segment vertices for the XYZ coordinate axes."""
    return np.asarray(
        [
            [0.0, 0.0, 0.0, 0.92, 0.18, 0.18],
            [length, 0.0, 0.0, 0.92, 0.18, 0.18],
            [0.0, 0.0, 0.0, 0.16, 0.72, 0.24],
            [0.0, length, 0.0, 0.16, 0.72, 0.24],
            [0.0, 0.0, 0.0, 0.18, 0.42, 0.92],
            [0.0, 0.0, length, 0.18, 0.42, 0.92],
        ],
        dtype=np.float32,
    )


@dataclass
class _SceneMeshItem:
    """Renderable mesh entry holding geometry, transform, color, and GL buffer handles."""

    key: str
    mesh_data: LoadedMeshData
    color: Tuple[float, float, float, float]
    model: np.ndarray
    is_robot: bool
    vao: int = 0
    vbo: int = 0


@dataclass
class _SkinCloudItem:
    """Renderable tactile skin point-cloud entry for a single robot body part."""

    part_name: str
    base_points: np.ndarray
    active_indices: List[int]
    model: np.ndarray


def configure_default_gl_surface_format() -> None:
    """Configure Qt's default OpenGL surface format with depth/stencil buffers."""
    if os.environ.get("QT_QPA_PLATFORM") in ("offscreen", "minimal"):
        return
    fmt = QSurfaceFormat()
    fmt.setDepthBufferSize(24)
    fmt.setStencilBufferSize(8)
    fmt.setSwapBehavior(QSurfaceFormat.SwapBehavior.DoubleBuffer)
    if sys.platform == "darwin":
        # macOS native Core Profile is OpenGL 4.1 (supports GLSL 330 core shaders and QRhiGles2).
        # Avoid global MSAA so QQuickWidget/QWebEngineView and QOpenGLWidget share identical formats.
        fmt.setRenderableType(QSurfaceFormat.RenderableType.OpenGL)
        fmt.setVersion(4, 1)
        fmt.setProfile(QSurfaceFormat.OpenGLContextProfile.CoreProfile)
    elif sys.platform != "win32":
        # Avoid setting global MSAA on Windows default format, which can cause QWebEngineView to render black;
        # _HardwareGLCanvas sets 4x MSAA directly on its own QOpenGLWidget surface format.
        fmt.setSamples(4)
    QSurfaceFormat.setDefaultFormat(fmt)


class _SceneState:
    """Shared camera & scene geometry state used by both hardware OpenGL and headless software canvases."""

    def __init__(self):
        """Initialize default orbit camera parameters and empty scene collections."""
        self.default_center = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.target = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.azimuth_deg = 180.0
        self.elevation_deg = 8.0
        self.distance = 1.15
        self.fovy = 52.0
        self.z_near = 0.03
        self.z_far = 25.0

        self.meshes: Dict[str, _SceneMeshItem] = {}
        self.mesh_order: List[str] = []
        self.skin_clouds: Dict[str, _SkinCloudItem] = {}
        self.custom_lines: Dict[str, Tuple[np.ndarray, float]] = {}
        self.show_axes = False
        self.show_grid = True
        self.overlay_status = ""
        self.grid_verts = _build_grid_vertices()
        self.axes_verts = _build_axes_vertices()

    def camera_eye(self) -> np.ndarray:
        """Compute the world-space camera position from spherical orbit coordinates."""
        az = math.radians(self.azimuth_deg)
        el = math.radians(self.elevation_deg)
        cos_el = math.cos(el)
        offset = np.array(
            [
                self.distance * cos_el * math.cos(az),
                self.distance * cos_el * math.sin(az),
                self.distance * math.sin(el),
            ],
            dtype=np.float64,
        )
        return self.target + offset

    def compute_view_proj(self, width: int, height: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return the camera eye position, view matrix, and projection matrix for a viewport size."""
        eye = self.camera_eye()
        up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        view = _look_at(eye, self.target, up)
        aspect = max(width, 1) / max(height, 1)
        proj = _perspective(self.fovy, aspect, self.z_near, self.z_far)
        return eye.astype(np.float32), view, proj


def _paint_software_scene(widget: QWidget, st: _SceneState) -> None:
    """Render a depth-sorted triangle fallback of the 3D scene using ``QPainter``."""
    painter = QPainter(widget)
    painter.setRenderHint(QPainter.RenderHint.Antialiasing)
    painter.fillRect(widget.rect(), QColor("#f8fafc"))

    _, view, proj = st.compute_view_proj(widget.width(), widget.height())
    vp = proj @ view
    w = max(1, widget.width())
    h = max(1, widget.height())

    polys: List[Tuple[float, QPolygonF, QColor]] = []
    for key in st.mesh_order:
        it = st.meshes[key]
        vn = it.mesh_data.interleaved_vn
        if vn.shape[0] < 3:
            continue
        tris = vn[:, :3].reshape((-1, 3, 3))
        step = max(1, tris.shape[0] // 250)
        sampled = tris[::step]
        mvp = vp @ it.model
        ones = np.ones((sampled.shape[0], 3, 1), dtype=np.float32)
        hom = np.concatenate([sampled, ones], axis=2) @ mvp.T
        w_clip = hom[:, :, 3:4]
        valid = np.all(w_clip[:, :, 0] > 0.05, axis=1)
        if not np.any(valid):
            continue
        ndc = hom[valid, :, :3] / w_clip[valid]
        sx = (ndc[:, :, 0] * 0.5 + 0.5) * w
        sy = (1.0 - (ndc[:, :, 1] * 0.5 + 0.5)) * h
        depths = np.mean(ndc[:, :, 2], axis=1)
        col = QColor.fromRgbF(
            min(max(it.color[0], 0.0), 1.0),
            min(max(it.color[1], 0.0), 1.0),
            min(max(it.color[2], 0.0), 1.0),
            min(max(it.color[3], 0.0), 1.0),
        )
        for idx in range(sx.shape[0]):
            poly = QPolygonF(
                [
                    QPoint(int(sx[idx, 0]), int(sy[idx, 0])),
                    QPoint(int(sx[idx, 1]), int(sy[idx, 1])),
                    QPoint(int(sx[idx, 2]), int(sy[idx, 2])),
                ]
            )
            polys.append((float(depths[idx]), poly, col))

    polys.sort(key=lambda t: t[0], reverse=True)
    painter.setPen(Qt.PenStyle.NoPen)
    for _, poly, col in polys:
        painter.setBrush(col)
        painter.drawPolygon(poly)

    if st.overlay_status:
        _draw_status_badge(painter, widget.width(), st.overlay_status)
    painter.end()


def _draw_status_badge(painter: QPainter, width: int, text: str) -> None:
    """Draw a centered translucent status banner near the top of the viewport."""
    font = QFont()
    font.setPointSize(10)
    font.setBold(True)
    painter.setFont(font)
    fm = painter.fontMetrics()
    pad_x, pad_y = 14, 8
    tw = fm.horizontalAdvance(text) + pad_x * 2
    th = fm.height() + pad_y * 2
    x = max(12, (width - tw) // 2)
    y = 14
    painter.setPen(QPen(QColor("#cbd5e1"), 1))
    painter.setBrush(QColor(15, 23, 42, 215))
    painter.drawRoundedRect(x, y, tw, th, 10, 10)
    painter.setPen(QColor("#f8fafc"))
    painter.drawText(x + pad_x, y + pad_y + fm.ascent(), text)


class _SoftwareCanvas(QWidget):
    """Transparent-for-mouse fallback ``QWidget`` canvas for headless or non-GL environments."""

    def __init__(self, state: _SceneState, parent: Optional[QWidget] = None):
        """Initialize the software-rendered canvas backed by ``state``."""
        super().__init__(parent)
        self._st = state
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)

    def release_buffers(self) -> None:
        """No-op buffer release matching the ``_HardwareGLCanvas`` interface."""
        pass

    def upload_all(self) -> None:
        """No-op mesh upload matching the ``_HardwareGLCanvas`` interface."""
        pass

    def save_depth(self, target_path: str) -> bool:
        """Save a placeholder grayscale depth image to ``target_path``."""
        img = QImage(max(self.width(), 1), max(self.height(), 1), QImage.Format.Format_Grayscale8)
        img.fill(240)
        return img.save(target_path)

    def paintEvent(self, _event) -> None:
        """Repaint the scene using the software ``QPainter`` renderer."""
        _paint_software_scene(self, self._st)


class _HardwareGLCanvas(QOpenGLWidget):
    """Hardware-accelerated ``QOpenGLWidget`` canvas supporting OpenGL 3.3 Core and 2.1 fallback."""

    def __init__(self, state: _SceneState, parent: Optional[QWidget] = None):
        """Configure OpenGL surface format and initialize GL state handles."""
        super().__init__(parent)
        fmt = QSurfaceFormat(QSurfaceFormat.defaultFormat())
        if sys.platform != "darwin":
            # On macOS, keep pixel format attributes identical to defaultFormat() so NSOpenGLContext
            # sharing with QQuickWidget/QWebEngineView and _GLSurfaceAnchor never mismatches on MSAA.
            fmt.setSamples(4)
        self.setFormat(fmt)
        self._st = state
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self._gl_ready = False
        self._use_vao = False
        self._mesh_prog = 0
        self._line_prog = 0
        self._dyn_vao = 0
        self._dyn_vbo = 0
        self._bound_ctx = None

    def _reset_gl_handles(self) -> None:
        """Reset stored shader program, VAO, and VBO handles to zero."""
        self._gl_ready = False
        self._mesh_prog = 0
        self._line_prog = 0
        self._dyn_vao = 0
        self._dyn_vbo = 0
        for item in self._st.meshes.values():
            item.vao = 0
            item.vbo = 0

    def _on_context_about_to_be_destroyed(self) -> None:
        """Delete GPU buffers and shader programs before the OpenGL context is destroyed."""
        ctx = self._bound_ctx
        self._bound_ctx = None
        if ctx is not None and ctx.isValid():
            try:
                self.makeCurrent()
                for item in self._st.meshes.values():
                    if item.vbo:
                        try:
                            gl.glDeleteBuffers(1, [item.vbo])
                        except Exception:
                            pass
                    if item.vao and self._use_vao:
                        try:
                            gl.glDeleteVertexArrays(1, [item.vao])
                        except Exception:
                            pass
                if self._dyn_vbo:
                    try:
                        gl.glDeleteBuffers(1, [self._dyn_vbo])
                    except Exception:
                        pass
                if self._dyn_vao and self._use_vao:
                    try:
                        gl.glDeleteVertexArrays(1, [self._dyn_vao])
                    except Exception:
                        pass
                for prog in (self._mesh_prog, self._line_prog):
                    if prog:
                        try:
                            gl.glDeleteProgram(prog)
                        except Exception:
                            pass
                self.doneCurrent()
            except Exception:
                pass
        self._reset_gl_handles()

    def initializeGL(self) -> None:
        """Compile shaders, allocate dynamic buffers, and upload initial scene meshes."""
        self._reset_gl_handles()
        if not HAS_PYOPENGL:
            return
        ctx = self.context()
        if ctx is None or not ctx.isValid():
            return
        if ctx is not self._bound_ctx:
            self._bound_ctx = ctx
            ctx.aboutToBeDestroyed.connect(
                self._on_context_about_to_be_destroyed,
                Qt.ConnectionType.DirectConnection,
            )
        try:
            try:
                vao_probe = gl.glGenVertexArrays(1)
                gl.glBindVertexArray(vao_probe)
                self._mesh_prog = self._compile_program(_MESH_VERT_330, _MESH_FRAG_330)
                self._line_prog = self._compile_program(_LINE_VERT_330, _LINE_FRAG_330)
                gl.glBindVertexArray(0)
                gl.glDeleteVertexArrays(1, [vao_probe])
                self._use_vao = True
            except Exception:
                self._mesh_prog = self._compile_program(
                    _MESH_VERT_120, _MESH_FRAG_120, [(0, "a_pos"), (1, "a_norm")]
                )
                self._line_prog = self._compile_program(
                    _LINE_VERT_120, _LINE_FRAG_120, [(0, "a_pos"), (1, "a_col")]
                )
                self._use_vao = False

            if self._use_vao:
                self._dyn_vao = int(gl.glGenVertexArrays(1))
            self._dyn_vbo = int(gl.glGenBuffers(1))
            for item in self._st.meshes.values():
                self._upload_mesh(item)

            gl.glEnable(gl.GL_DEPTH_TEST)
            for cap in (getattr(gl, "GL_PROGRAM_POINT_SIZE", None), getattr(gl, "GL_MULTISAMPLE", None)):
                if cap is not None:
                    try:
                        gl.glEnable(cap)
                    except Exception:
                        pass
            self._gl_ready = True
        except Exception:
            self._gl_ready = False

    def _compile_program(
        self, vert_src: str, frag_src: str, attribs: Optional[List[Tuple[int, str]]] = None
    ) -> int:
        """Compile and link a vertex/fragment shader pair into an OpenGL program ID."""
        vs = gl_shaders.compileShader(vert_src, gl.GL_VERTEX_SHADER)
        fs = gl_shaders.compileShader(frag_src, gl.GL_FRAGMENT_SHADER)
        prog = gl.glCreateProgram()
        gl.glAttachShader(prog, vs)
        gl.glAttachShader(prog, fs)
        if attribs:
            for idx, name in attribs:
                gl.glBindAttribLocation(prog, idx, name.encode("ascii"))
        gl.glLinkProgram(prog)
        if not gl.glGetProgramiv(prog, gl.GL_LINK_STATUS):
            raise RuntimeError(f"GL program link error: {gl.glGetProgramInfoLog(prog)}")
        gl.glDeleteShader(vs)
        gl.glDeleteShader(fs)
        return int(prog)

    def _upload_mesh(self, item: _SceneMeshItem) -> None:
        """Upload an item's interleaved vertex/normal data into static GPU buffers."""
        if item.mesh_data.vertex_count == 0 or item.vbo != 0:
            return
        if self._use_vao:
            item.vao = int(gl.glGenVertexArrays(1))
            gl.glBindVertexArray(item.vao)
        item.vbo = int(gl.glGenBuffers(1))
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, item.vbo)
        data = item.mesh_data.interleaved_vn
        gl.glBufferData(gl.GL_ARRAY_BUFFER, data.nbytes, data, gl.GL_STATIC_DRAW)
        stride = 6 * 4
        if self._use_vao:
            gl.glEnableVertexAttribArray(0)
            gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, stride, ctypes.c_void_p(0))
            gl.glEnableVertexAttribArray(1)
            gl.glVertexAttribPointer(1, 3, gl.GL_FLOAT, gl.GL_FALSE, stride, ctypes.c_void_p(12))
            gl.glBindVertexArray(0)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def _has_valid_context(self) -> bool:
        """Return whether the widget is initialized and holds a valid OpenGL context."""
        ctx = self.context()
        return bool(self._gl_ready and self.isValid() and ctx is not None and ctx.isValid())

    def upload_all(self) -> None:
        """Upload any uninitialized scene meshes to GPU memory."""
        if self._has_valid_context():
            self.makeCurrent()
            for item in self._st.meshes.values():
                self._upload_mesh(item)
            self.doneCurrent()

    def release_buffers(self) -> None:
        """Delete GPU vertex buffers and VAOs for all loaded scene meshes."""
        if not self._has_valid_context():
            for item in self._st.meshes.values():
                item.vbo = 0
                item.vao = 0
            return
        self.makeCurrent()
        for item in self._st.meshes.values():
            if item.vbo:
                try:
                    gl.glDeleteBuffers(1, [item.vbo])
                except Exception:
                    pass
                item.vbo = 0
            if item.vao and self._use_vao:
                try:
                    gl.glDeleteVertexArrays(1, [item.vao])
                except Exception:
                    pass
                item.vao = 0
        self.doneCurrent()

    def save_depth(self, target_path: str) -> bool:
        """Read back the OpenGL depth buffer and save it as a grayscale image."""
        if not self._has_valid_context():
            img = QImage(max(self.width(), 1), max(self.height(), 1), QImage.Format.Format_Grayscale8)
            img.fill(240)
            return img.save(target_path)
        self.makeCurrent()
        dpr = self.devicePixelRatioF()
        w = max(1, int(round(self.width() * dpr)))
        h = max(1, int(round(self.height() * dpr)))
        gl.glPixelStorei(gl.GL_PACK_ALIGNMENT, 1)
        raw = gl.glReadPixels(0, 0, w, h, gl.GL_DEPTH_COMPONENT, gl.GL_FLOAT)
        self.doneCurrent()
        depth = np.flipud(np.frombuffer(raw, dtype=np.float32).reshape((h, w)))
        vis = np.ascontiguousarray(np.clip(depth * 255.0, 0, 255).astype(np.uint8))
        qimg = QImage(vis.data, w, h, w, QImage.Format.Format_Grayscale8).copy()
        return qimg.save(target_path)

    def resizeGL(self, w: int, h: int) -> None:
        """Update the OpenGL viewport dimensions accounting for device pixel ratio."""
        if self._gl_ready:
            dpr = self.devicePixelRatioF()
            gl.glViewport(0, 0, max(1, int(round(w * dpr))), max(1, int(round(h * dpr))))

    def paintGL(self) -> None:
        """Render grid, axes, custom lines, opaque/transparent meshes, skin clouds, and status badge."""
        if not self._has_valid_context():
            _paint_software_scene(self, self._st)
            return

        dpr = self.devicePixelRatioF()
        gl.glViewport(0, 0, max(1, int(round(self.width() * dpr))), max(1, int(round(self.height() * dpr))))
        gl.glClearColor(0.975, 0.980, 0.990, 1.0)
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthMask(gl.GL_TRUE)
        gl.glDisable(gl.GL_BLEND)

        eye, view, proj = self._st.compute_view_proj(self.width(), self.height())
        vp = proj @ view

        if self._st.show_grid and self._st.grid_verts.shape[0] > 0:
            self._draw_colored(self._st.grid_verts, vp, gl.GL_LINES, line_width=1.0)
        if self._st.show_axes:
            self._draw_colored(self._st.axes_verts, vp, gl.GL_LINES, line_width=2.5)
        for verts, width in self._st.custom_lines.values():
            self._draw_colored(verts, vp, gl.GL_LINES, line_width=min(width, 4.0))

        opaque: List[_SceneMeshItem] = []
        transp: List[_SceneMeshItem] = []
        for key in self._st.mesh_order:
            it = self._st.meshes[key]
            if it.mesh_data.vertex_count == 0:
                continue
            if it.vbo == 0:
                self._upload_mesh(it)
            if it.color[3] < 0.995:
                transp.append(it)
            else:
                opaque.append(it)

        gl.glUseProgram(self._mesh_prog)
        loc_mvp = gl.glGetUniformLocation(self._mesh_prog, "u_mvp")
        loc_model = gl.glGetUniformLocation(self._mesh_prog, "u_model")
        loc_nmat = gl.glGetUniformLocation(self._mesh_prog, "u_normal_mat")
        loc_color = gl.glGetUniformLocation(self._mesh_prog, "u_color")
        loc_cam = gl.glGetUniformLocation(self._mesh_prog, "u_cam_pos")
        gl.glUniform3f(loc_cam, float(eye[0]), float(eye[1]), float(eye[2]))

        for it in opaque:
            self._draw_mesh(it, vp, loc_mvp, loc_model, loc_nmat, loc_color)

        if self._st.skin_clouds:
            self._draw_skin(vp)

        if transp:
            transp.sort(key=lambda item: float(np.linalg.norm(item.model[:3, 3] - eye)), reverse=True)
            gl.glUseProgram(self._mesh_prog)
            gl.glUniform3f(loc_cam, float(eye[0]), float(eye[1]), float(eye[2]))
            gl.glEnable(gl.GL_BLEND)
            gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
            gl.glDepthMask(gl.GL_FALSE)
            for it in transp:
                self._draw_mesh(it, vp, loc_mvp, loc_model, loc_nmat, loc_color)
            gl.glDepthMask(gl.GL_TRUE)
            gl.glDisable(gl.GL_BLEND)

        gl.glUseProgram(0)

        if self._st.overlay_status:
            painter = QPainter(self)
            painter.setRenderHint(QPainter.RenderHint.Antialiasing)
            _draw_status_badge(painter, self.width(), self._st.overlay_status)
            painter.end()

    def _draw_mesh(
        self, it: _SceneMeshItem, vp: np.ndarray, loc_mvp: int, loc_model: int, loc_nmat: int, loc_color: int
    ) -> None:
        """Bind uniforms and vertex attributes and draw a single triangle mesh."""
        mvp = vp @ it.model
        m3 = it.model[:3, :3]
        try:
            nmat = np.linalg.inv(m3).T.astype(np.float32)
        except np.linalg.LinAlgError:
            nmat = m3.astype(np.float32)
        gl.glUniformMatrix4fv(loc_mvp, 1, gl.GL_TRUE, mvp)
        gl.glUniformMatrix4fv(loc_model, 1, gl.GL_TRUE, it.model)
        gl.glUniformMatrix3fv(loc_nmat, 1, gl.GL_TRUE, nmat)
        gl.glUniform4f(loc_color, *it.color)
        if self._use_vao:
            gl.glBindVertexArray(it.vao)
            gl.glDrawArrays(gl.GL_TRIANGLES, 0, it.mesh_data.vertex_count)
            gl.glBindVertexArray(0)
        else:
            stride = 6 * 4
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, it.vbo)
            gl.glEnableVertexAttribArray(0)
            gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, stride, ctypes.c_void_p(0))
            gl.glEnableVertexAttribArray(1)
            gl.glVertexAttribPointer(1, 3, gl.GL_FLOAT, gl.GL_FALSE, stride, ctypes.c_void_p(12))
            gl.glDrawArrays(gl.GL_TRIANGLES, 0, it.mesh_data.vertex_count)
            gl.glDisableVertexAttribArray(0)
            gl.glDisableVertexAttribArray(1)
            gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def _draw_colored(
        self, verts: np.ndarray, mvp: np.ndarray, mode: int, line_width: float = 1.0, point_size: float = 5.0
    ) -> None:
        """Stream and draw colored ``GL_LINES`` or ``GL_POINTS`` primitives."""
        if verts.shape[0] == 0:
            return
        gl.glUseProgram(self._line_prog)
        gl.glUniformMatrix4fv(gl.glGetUniformLocation(self._line_prog, "u_mvp"), 1, gl.GL_TRUE, mvp)
        gl.glUniform1f(
            gl.glGetUniformLocation(self._line_prog, "u_point_size"), float(point_size * self.devicePixelRatioF())
        )
        gl.glUniform1f(gl.glGetUniformLocation(self._line_prog, "u_alpha"), 1.0)
        if mode == gl.GL_LINES:
            try:
                gl.glLineWidth(max(1.0, float(line_width)))
            except Exception:
                pass
        stride = 6 * 4
        if self._use_vao:
            gl.glBindVertexArray(self._dyn_vao)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, self._dyn_vbo)
        gl.glBufferData(gl.GL_ARRAY_BUFFER, verts.nbytes, verts, gl.GL_STREAM_DRAW)
        gl.glEnableVertexAttribArray(0)
        gl.glVertexAttribPointer(0, 3, gl.GL_FLOAT, gl.GL_FALSE, stride, ctypes.c_void_p(0))
        gl.glEnableVertexAttribArray(1)
        gl.glVertexAttribPointer(1, 3, gl.GL_FLOAT, gl.GL_FALSE, stride, ctypes.c_void_p(12))
        gl.glDrawArrays(mode, 0, int(verts.shape[0]))
        if self._use_vao:
            gl.glBindVertexArray(0)
        else:
            gl.glDisableVertexAttribArray(0)
            gl.glDisableVertexAttribArray(1)
        gl.glBindBuffer(gl.GL_ARRAY_BUFFER, 0)

    def _draw_skin(self, vp: np.ndarray) -> None:
        """Transform and render active and inactive tactile skin taxels as point clouds."""
        inactive_chunks: List[np.ndarray] = []
        active_chunks: List[np.ndarray] = []
        for sc in self._st.skin_clouds.values():
            pts = sc.base_points
            if pts.shape[0] == 0:
                continue
            world_pts = (pts @ sc.model[:3, :3].T) + sc.model[:3, 3]
            n = world_pts.shape[0]
            if sc.active_indices:
                act_mask = np.zeros(n, dtype=bool)
                valid_idx = [i for i in sc.active_indices if 0 <= i < n]
                if valid_idx:
                    act_mask[valid_idx] = True
                inact_pts = world_pts[~act_mask]
                act_pts = world_pts[act_mask]
            else:
                inact_pts = world_pts
                act_pts = np.empty((0, 3), dtype=np.float32)

            if inact_pts.shape[0] > 0:
                buf = np.empty((inact_pts.shape[0], 6), dtype=np.float32)
                buf[:, :3] = inact_pts
                buf[:, 3:] = (0.12, 0.38, 0.96)
                inactive_chunks.append(buf)
            if act_pts.shape[0] > 0:
                buf_a = np.empty((act_pts.shape[0], 6), dtype=np.float32)
                buf_a[:, :3] = act_pts
                buf_a[:, 3:] = (1.0, 0.12, 0.12)
                active_chunks.append(buf_a)

        if inactive_chunks:
            self._draw_colored(np.vstack(inactive_chunks), vp, gl.GL_POINTS, point_size=4.0)
        if active_chunks:
            self._draw_colored(np.vstack(active_chunks), vp, gl.GL_POINTS, point_size=8.0)


class EmbeddedGLViewer(QWidget):
    """
    Hardware-accelerated embedded 3D scene widget for Modes 1, 2, 3, and 4.
    Uses `_HardwareGLCanvas(QOpenGLWidget)` on desktop displays and automatically selects
    `_SoftwareCanvas(QWidget)` when running under headless `offscreen`/`minimal` Qt platforms.
    """

    viewReset = Signal()

    def __init__(self, parent: Optional[QWidget] = None):
        """Initialize the embedded 3D scene widget and select hardware or software rendering."""
        super().__init__(parent)
        self.setMinimumSize(160, 120)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self._st = _SceneState()
        self._last_mouse_pos = QPoint()
        self._drag_mode: Optional[str] = None

        plat = (QGuiApplication.platformName() or "").lower()
        use_sw = (
            not HAS_PYOPENGL
            or plat in ("offscreen", "minimal")
            or os.environ.get("PYCUB_FORCE_SW_VIEWPORT") == "1"
        )

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self._canvas = _SoftwareCanvas(self._st, self) if use_sw else _HardwareGLCanvas(self._st, self)
        layout.addWidget(self._canvas)

    def showEvent(self, event) -> None:
        """Trigger an initial canvas redraw when the viewport is shown."""
        super().showEvent(event)
        self._trigger_redraw()

    def _trigger_redraw(self) -> None:
        """Schedule a repaint on the underlying GL or software canvas."""
        self._canvas.update()

    def set_overlay_status(self, text: str) -> None:
        """
        Set or clear the status badge text overlaid on the viewport.

        :param text: Status text to display, or an empty string to hide the badge.
        """
        if self._st.overlay_status != text:
            self._st.overlay_status = text
            self._trigger_redraw()

    def set_show_axes(self, show: bool) -> None:
        """
        Toggle rendering of the world XYZ coordinate axes.

        :param show: Whether coordinate axes should be visible.
        """
        self._st.show_axes = bool(show)
        self._trigger_redraw()

    def set_show_grid(self, show: bool) -> None:
        """
        Toggle rendering of the ground reference grid.

        :param show: Whether the floor grid should be visible.
        """
        self._st.show_grid = bool(show)
        self._trigger_redraw()

    def reset_view(self) -> None:
        """Reset the orbit camera target, angles, and distance to defaults and emit ``viewReset``."""
        self._st.target = self._st.default_center.copy()
        self._st.azimuth_deg = 180.0
        self._st.elevation_deg = 8.0
        self._st.distance = 1.15
        self.viewReset.emit()
        self._trigger_redraw()

    def clear_scene(self) -> None:
        """Release GPU buffers and clear all meshes, skin clouds, and custom lines."""
        self._canvas.release_buffers()
        self._st.meshes.clear()
        self._st.mesh_order.clear()
        self._st.skin_clouds.clear()
        self._st.custom_lines.clear()
        self._trigger_redraw()

    def load_scene_manifest(self, manifest: Dict[str, Any]) -> None:
        """
        Populate the 3D scene from a manifest describing meshes, skin clouds, and floor height.

        :param manifest: Scene dictionary containing ``meshes``, ``skin_parts``, ``default_center``, and ``floor_z``.
        """
        self._canvas.release_buffers()
        self._st.meshes.clear()
        self._st.mesh_order.clear()
        self._st.skin_clouds.clear()
        self._st.custom_lines.clear()

        self._st.default_center = np.asarray(manifest.get("default_center", [0.0, 0.0, 0.0]), dtype=np.float64)

        for item in manifest.get("meshes", []):
            key = item["key"]
            path = item["path"]
            scale = float(item.get("scale", 1.0))
            rgba = tuple(float(c) for c in item.get("color", [0.85, 0.85, 0.85, 1.0]))
            if len(rgba) == 3:
                rgba = (rgba[0], rgba[1], rgba[2], 1.0)
            model = np.asarray(item.get("transform", np.eye(4)), dtype=np.float32).reshape((4, 4))
            is_robot = bool(item.get("is_robot", False))
            try:
                mesh_data = load_obj_mesh(path, scale=scale)
            except OSError:
                continue
            self._st.meshes[key] = _SceneMeshItem(
                key=key,
                mesh_data=mesh_data,
                color=rgba,
                model=model,
                is_robot=is_robot,
            )
            self._st.mesh_order.append(key)

        for skin_info in manifest.get("skin_parts", []):
            try:
                pts = load_skin_pcd_points(skin_info["pcd_path"])
            except Exception:
                continue
            self._st.skin_clouds[skin_info["part"]] = _SkinCloudItem(
                part_name=skin_info["part"],
                base_points=pts,
                active_indices=[],
                model=np.eye(4, dtype=np.float32),
            )

        plane_z = manifest.get("floor_z")
        self._st.grid_verts = (
            _build_grid_vertices(z=float(plane_z) + 0.002) if plane_z is not None else _build_grid_vertices()
        )

        self._canvas.upload_all()
        self.reset_view()

    def update_frame(self, frame: Dict[str, Any]) -> None:
        """
        Apply per-frame mesh transforms, skin activations, and debug line updates.

        :param frame: Frame payload with optional ``transforms``, ``skin_transforms``, ``skin_active``, and ``lines``.
        """
        transforms = frame.get("transforms")
        if transforms:
            for key, mat in transforms.items():
                item = self._st.meshes.get(key)
                if item is not None:
                    item.model = np.asarray(mat, dtype=np.float32).reshape((4, 4))

        skin_transforms = frame.get("skin_transforms")
        if skin_transforms:
            for part, mat in skin_transforms.items():
                sc = self._st.skin_clouds.get(part)
                if sc is not None:
                    sc.model = np.asarray(mat, dtype=np.float32).reshape((4, 4))

        skin_active = frame.get("skin_active")
        if skin_active is not None:
            for part, sc in self._st.skin_clouds.items():
                sc.active_indices = skin_active.get(part, [])

        if "lines" in frame and frame["lines"] is not None:
            self._st.custom_lines.clear()
            for name, ldata in frame["lines"].items():
                arr = np.asarray(ldata["vertices"], dtype=np.float32).reshape((-1, 6))
                width = float(ldata.get("width", 3.0))
                if arr.shape[0] >= 2:
                    self._st.custom_lines[name] = (arr, width)

        self._trigger_redraw()

    def save_rgb_image(self, target_path: str) -> bool:
        """
        Capture the current viewport framebuffer and save it as an image file.

        :param target_path: Destination image file path.
        """
        os.makedirs(os.path.dirname(os.path.abspath(target_path)), exist_ok=True)
        if isinstance(self._canvas, QOpenGLWidget):
            img = self._canvas.grabFramebuffer()
        else:
            img = self._canvas.grab().toImage()
        return img.save(target_path)

    def save_depth_image(self, target_path: str) -> bool:
        """
        Capture the current viewport depth buffer and save it as a grayscale image.

        :param target_path: Destination image file path.
        """
        os.makedirs(os.path.dirname(os.path.abspath(target_path)), exist_ok=True)
        return self._canvas.save_depth(target_path)

    def mousePressEvent(self, event: QMouseEvent) -> None:
        """Begin camera orbit or pan drag based on mouse button and modifier keys."""
        self._last_mouse_pos = event.position().toPoint()
        btn = event.button()
        mods = event.modifiers()
        if btn == Qt.MouseButton.LeftButton and not (mods & Qt.KeyboardModifier.ShiftModifier):
            self._drag_mode = "orbit"
        elif btn in (Qt.MouseButton.RightButton, Qt.MouseButton.MiddleButton) or (
            btn == Qt.MouseButton.LeftButton and (mods & Qt.KeyboardModifier.ShiftModifier)
        ):
            self._drag_mode = "pan"
        else:
            self._drag_mode = None
        event.accept()

    def mouseMoveEvent(self, event: QMouseEvent) -> None:
        """Update camera orbit angles or pan target while dragging."""
        cur = event.position().toPoint()
        dx = cur.x() - self._last_mouse_pos.x()
        dy = cur.y() - self._last_mouse_pos.y()
        self._last_mouse_pos = cur

        if self._drag_mode == "orbit":
            self._st.azimuth_deg = (self._st.azimuth_deg - dx * 0.45) % 360.0
            self._st.elevation_deg = max(-88.0, min(88.0, self._st.elevation_deg + dy * 0.40))
            self._trigger_redraw()
        elif self._drag_mode == "pan":
            eye = self._st.camera_eye()
            fwd = self._st.target - eye
            fn = np.linalg.norm(fwd)
            if fn > 1e-6:
                fwd /= fn
            right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
            rn = np.linalg.norm(right)
            if rn > 1e-6:
                right /= rn
            up = np.cross(right, fwd)
            scale = self._st.distance * 0.0016
            self._st.target = self._st.target - right * (dx * scale) + up * (dy * scale)
            self._trigger_redraw()
        event.accept()

    def mouseReleaseEvent(self, event: QMouseEvent) -> None:
        """End the active camera drag interaction."""
        self._drag_mode = None
        event.accept()

    def mouseDoubleClickEvent(self, event: QMouseEvent) -> None:
        """Reset the camera view on double-click."""
        self.reset_view()
        event.accept()

    def wheelEvent(self, event: QWheelEvent) -> None:
        """Zoom the orbit camera in or out in response to wheel scrolling."""
        delta = event.angleDelta().y()
        if delta != 0:
            factor = math.pow(0.88, delta / 120.0)
            self._st.distance = max(0.18, min(12.0, self._st.distance * factor))
            self._trigger_redraw()
        event.accept()


class EyeCameraPanel(QWidget):
    """
    Live preview panel for iCub's `l_eye` or `r_eye` camera feed (RGB or Depth) with PNG save button.
    """

    def __init__(self, eye_name: str, title: Optional[str] = None, parent: Optional[QWidget] = None):
        """Initialize an eye camera preview panel with RGB/Depth selector and PNG export."""
        super().__init__(parent)
        self.eye_name = eye_name
        self._last_rgb: Optional[np.ndarray] = None
        self._last_depth: Optional[np.ndarray] = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        header = QHBoxLayout()
        self.title_label = QLabel(title or eye_name)
        header.addWidget(self.title_label)
        header.addStretch(1)

        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["RGB", "Depth"])
        self.mode_combo.currentIndexChanged.connect(self._refresh_pixmap)
        header.addWidget(self.mode_combo)

        self.save_btn = QPushButton("Save PNG")
        self.save_btn.clicked.connect(self._save_current)
        header.addWidget(self.save_btn)
        layout.addLayout(header)

        self.image_label = QLabel("No camera frame yet")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setMinimumSize(96, 72)
        layout.addWidget(self.image_label, 1)

        from studio.theme import get_active_theme

        self.apply_theme(get_active_theme())

    def apply_theme(self, theme: str) -> None:
        """Apply ``theme`` colors to the eye camera header label and frame border."""
        from studio.theme import get_theme_tokens

        t = get_theme_tokens(theme)
        self.title_label.setStyleSheet(f"font-weight: 700; color: {t.text_primary};")
        self.image_label.setStyleSheet(
            f"background: {t.console_bg}; color: #94a3b8; border: 1px solid {t.border}; border-radius: 8px;"
        )

    def update_eye_frames(self, rgb: Optional[np.ndarray], depth: Optional[np.ndarray]) -> None:
        """
        Store new RGB and/or depth camera arrays and refresh the preview image.

        :param rgb: Optional ``(H, W, 3)`` uint8 RGB image array.
        :param depth: Optional ``(H, W)`` float32 normalized depth array.
        """
        if rgb is not None:
            self._last_rgb = np.asarray(rgb, dtype=np.uint8)
        if depth is not None:
            self._last_depth = np.asarray(depth, dtype=np.float32)
        self._refresh_pixmap()

    def _current_qimage(self) -> Optional[QImage]:
        """Convert the currently selected RGB or depth buffer into a ``QImage``."""
        if self.mode_combo.currentIndex() == 0:
            if self._last_rgb is None or self._last_rgb.ndim != 3:
                return None
            arr = np.ascontiguousarray(self._last_rgb[:, :, :3])
            h, w, _ = arr.shape
            return QImage(arr.data, w, h, w * 3, QImage.Format.Format_RGB888).copy()
        else:
            if self._last_depth is None or self._last_depth.ndim != 2:
                return None
            vis = np.clip(self._last_depth * 255.0, 0, 255).astype(np.uint8)
            arr = np.ascontiguousarray(vis)
            h, w = arr.shape
            return QImage(arr.data, w, h, w, QImage.Format.Format_Grayscale8).copy()

    def _refresh_pixmap(self) -> None:
        """Scale and display the active ``QImage`` in the preview label."""
        qimg = self._current_qimage()
        if qimg is None:
            return
        pix = QPixmap.fromImage(qimg).scaled(
            max(80, self.image_label.width() - 4),
            max(60, self.image_label.height() - 4),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.image_label.setPixmap(pix)

    def _save_current(self) -> None:
        """Prompt for a destination path and save the current eye camera view as PNG."""
        qimg = self._current_qimage()
        if qimg is None:
            return
        default_dir = str(ensure_user_workspace() / "images")
        suffix = "rgb" if self.mode_combo.currentIndex() == 0 else "depth"
        default_path = os.path.join(default_dir, f"{self.eye_name}_{suffix}.png")
        path, _ = QFileDialog.getSaveFileName(self, "Save Eye Image", default_path, "PNG Image (*.png)")
        if path:
            qimg.save(path)
