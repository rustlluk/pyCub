"""
Visualization utils for pyCub simulator

:Author: Lukas Rustler
"""
from __future__ import annotations
import webbrowser
import open3d as o3d
import open3d.core as o3c
import os
import numpy as np
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
from typing import Optional, Tuple

try:
    from icub_pybullet.runtime_paths import ensure_user_workspace, is_writable_dir
except Exception:
    from runtime_paths import ensure_user_workspace, is_writable_dir


class Visualizer:
    """Class to help with custom rendering"""

    def __init__(self, client: pyCub):
        """Initialize the Open3D GUI window, scene meshes, menus, and optional eye cameras.

        :param client: Active ``pyCub`` simulator client to visualize.
        """
        self.client = client

        if self.client.config.gui.web:
            o3d.visualization.webrtc_server.enable_webrtc()

        self.meshes = {}
        self.R_urdf = {}
        self.skin_activated_last_turn = {}

        self.blue = o3c.Tensor([0, 0, 1], dtype=o3c.Dtype.Float32)
        self.red = o3c.Tensor([1, 0, 0], dtype=o3c.Dtype.Float32)

        self._R = np.eye(4)
        self._R_ori = self._R[:3, :3]

        if self.client.config.skin.use:
            self.fpath_to_skin = {}

        self.gui = gui.Application.instance
        self.gui.initialize()
        self.window = self.gui.create_window("pyCub", 640, 480)

        self.gui.menubar = gui.Menu()
        self.menu = self.gui.menubar
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)

        self.vis = self.scene.scene
        self.scene.background_color = o3d.visualization.gui.Color(1, 1, 1, 1)
        self.vis.set_background([1, 1, 1, 1])
        self.vis.set_lighting(self.vis.NO_SHADOWS, [0, 0, 0])

        self.mattr = rendering.MaterialRecord()
        self.mattr.shader = 'defaultLitTransparency'
        if hasattr(self.client.config.gui, "pycub_alpha"):
            PYCUB_ALPHA = self.client.config.gui.pycub_alpha
        else:
            PYCUB_ALPHA = 1
        self.mattr.base_color = [1.0, 1.0, 1.0, PYCUB_ALPHA]

        self.mat = rendering.MaterialRecord()
        self.mat.shader = 'defaultLit'

        self.file_menu = gui.Menu()
        self.file_menu.add_item("RGB image", 0)
        self.file_menu.add_item("depth image", 1)
        menu_counter = 2
        for eye in ["l_eye", "r_eye"]:
            for image_type in ["RGB", "RGB_D"]:
                self.file_menu.add_item(f"{eye} {image_type} image", menu_counter)
                self.file_menu.set_enabled(menu_counter, False)
                menu_counter += 1
        self.menu.add_menu("Save", self.file_menu)

        self.show_menu = gui.Menu()
        self.show_menu.add_item("l_eye", menu_counter)
        self.show_menu.add_item("r_eye", menu_counter + 1)

        self.menu.add_menu("Show", self.show_menu)

        self.file_menu.add_item("Stream l_eye to file", menu_counter + 2)
        self.file_menu.set_enabled(menu_counter + 2, False)
        self.file_menu.add_item("Stream r_eye to file", menu_counter + 3)
        self.file_menu.set_enabled(menu_counter + 3, False)

        show_callbacks = []
        for menu_id in range(menu_counter + 4):
            c = self.MenuCallback(menu_id, self)
            if menu_id in [6, 7]:
                show_callbacks.append(c)
            self.window.set_on_menu_item_activated(menu_id, c)

        self.show_menu.add_item("Coordination Frame", menu_counter + 4)
        show_callbacks.append(self.MenuCallback(menu_counter + 4, self))
        self.window.set_on_menu_item_activated(menu_counter + 4, show_callbacks[-1])

        self.window.add_child(self.scene)

        self.file_dir = os.path.dirname(os.path.abspath(__file__))
        self.orientations = []
        self.positions = []
        self.colors = []
        self.paths = []
        self.eye_windows = {}

        self.read_info(self.client.robot)
        self.show_first()
        self.show_mesh()

        scene_mesh = o3d.geometry.TriangleMesh()
        for f_path, m in self.meshes.items():
            R = self.vis.get_geometry_transform(f_path)
            m.transform(R)
            scene_mesh += m
            m.transform(np.linalg.inv(R))
        bbox = scene_mesh.get_axis_aligned_bounding_box()
        center = bbox.get_center()

        self.default_center = center
        self.reset_view()

        for obj_id, obj_name, _, _, _ in self.client.other_objects:
            self.read_info(obj_id)
            self.show_first(urdf_name=obj_name)
            self.show_mesh()

        self.is_alive = True
        if self.client.config.eyes.l_eye:
            show_callbacks[0]()
        if self.client.config.eyes.r_eye:
            show_callbacks[1]()

        if self.client.config.gui.web:
            webbrowser.open("http://localhost:8888", new=0)

    def reset_view(self) -> None:
        """Reset the camera view to the initial look-at pose."""
        self.scene.look_at(self.default_center, self.default_center + [-1, 0, 0], [0, 0, 1])
        self.scene.center_of_rotation = self.default_center
        self.window.post_redraw()

    def show_first(self, urdf_name: Optional[str] = "robot") -> None:
        """Load and register initial visual meshes and skin clouds for ``urdf_name``."""
        for mesh_id in range(len(self.positions) // 3):
            col = self.colors[mesh_id * 3:(mesh_id + 1) * 3]
            f_path = self.paths[mesh_id]

            self.meshes[f_path] = o3d.io.read_triangle_mesh(f_path)

            if not self.meshes[f_path].has_triangle_normals():
                self.meshes[f_path].compute_triangle_normals()
            if not self.meshes[f_path].has_vertex_normals():
                self.meshes[f_path].compute_vertex_normals()

            self.meshes[f_path].paint_uniform_color(col[:3])

            init_xyz, init_rpy, scale, link_name = self.find_xyz_rpy(os.path.basename(f_path), urdf_name=urdf_name)

            R_urdf = np.eye(4)
            R_urdf[:3, :3] = np.reshape(self.client.getMatrixFromQuaternion(self.client.getQuaternionFromEuler(init_rpy)), (3, 3))
            R_urdf[:3, 3] = init_xyz

            self.R_urdf[f_path] = R_urdf

            self.meshes[f_path].scale(scale, self.meshes[f_path].get_center())
            self.meshes[f_path].translate(self.meshes[f_path].get_center() * scale, relative=False)

            if urdf_name == "robot":
                self.vis.add_geometry(f_path, geometry=self.meshes[f_path], material=self.mattr)
            else:
                self.vis.add_geometry(f_path, geometry=self.meshes[f_path], material=self.mat)

            if self.client.config.skin.use:
                if link_name in self.client.skin_point_clouds:
                    self.fpath_to_skin[f_path] = link_name
                    self.vis.add_geometry(f_path + "_skin", geometry=self.client.skin_point_clouds[link_name], material=self.mat)
                    self.skin_activated_last_turn[f_path] = 1
                else:
                    self.fpath_to_skin[f_path] = None

    def show_mesh(self) -> None:
        """Update mesh transforms, eye camera poses, and skin activation colors."""
        for mesh_id in range(len(self.positions) // 3):
            pos = self.positions[mesh_id * 3:(mesh_id + 1) * 3]
            ori = self.orientations[mesh_id * 4:(mesh_id + 1) * 4]
            f_path = self.paths[mesh_id]

            R_urdf = self.R_urdf[f_path]

            self._R_ori.flat[:] = self.client.getMatrixFromQuaternion(ori)
            self._R[:3, 3] = pos

            self.vis.set_geometry_transform(f_path, self._R @ R_urdf)
            for ew in self.eye_windows.values():
                ew.vis.set_geometry_transform(f_path, self._R @ R_urdf)
                if ew.link_name in f_path:
                    R_ = self._R @ R_urdf
                    ew.center = (R_ @ np.hstack((self.meshes[f_path].get_center(), 1)))[:3]
                    R_[:3, 3] = [0, 0, 0]
                    ew.dir = (R_ @ [0, 0, 1, 1])[:3]
                    ew.dir = 0.1 * ew.dir / np.linalg.norm(ew.dir)

            if self.client.config.skin.use and self.fpath_to_skin[f_path] is not None:
                colored_idxs = self.client.skin_activations[self.fpath_to_skin[f_path]] > 0
                if self.skin_activated_last_turn[f_path] == 1 or np.sum(colored_idxs) > 0:
                    point_cloud = self.client.skin_point_clouds[self.fpath_to_skin[f_path]]
                    if self.skin_activated_last_turn[f_path] == 1:
                        point_cloud.point["colors"][:] = self.blue

                    if np.sum(colored_idxs) > 0:
                        point_cloud.point["colors"][colored_idxs] = self.red
                        self.skin_activated_last_turn[f_path] = 1
                    else:
                        self.skin_activated_last_turn[f_path] = 0

                    geometry_name = f_path + "_skin"
                    self.vis.scene.update_geometry(
                        geometry_name,
                        point_cloud,
                        rendering.Scene.UPDATE_COLORS_FLAG,
                    )

                self.vis.set_geometry_transform(f_path + "_skin", self._R)

    def read_info(self, obj_id: int) -> int:
        """Fetch current link poses, colors, and mesh file paths for body ``obj_id``."""
        visualData = self.client.getVisualShapeData(obj_id)
        linkStates = self.client.getLinkStates(
            obj_id,
            range(self.client.getNumJoints(obj_id)),
            computeLinkVelocity=0,
            computeForwardKinematics=0,
        )

        self.positions = []
        self.orientations = []
        self.colors = []
        self.paths = []

        for m in visualData:
            f_path = m[self.client.visualShapeData["FILE"]].decode("utf-8")
            if f_path == "":
                continue

            col = m[self.client.visualShapeData["COLOR"]]
            link = m[self.client.visualShapeData["LINK"]]

            if link != -1:
                linkState = linkStates[link]
                ori = linkState[self.client.linkInfo["URDFORI"]]
                pos = linkState[self.client.linkInfo["URDFPOS"]]
            else:
                pos, ori = self.client.getBasePositionAndOrientation(obj_id)

            self.positions += pos
            self.orientations += ori
            self.colors += col[:-1]
            self.paths.append(f_path)

        return 0

    def render(self) -> None:
        """Synchronize the 3D scene and eye windows with PyBullet and advance one GUI tick."""
        self.read_info(self.client.robot)
        self.show_mesh()

        for obj_id, _, fixed, _, _ in self.client.other_objects:
            self.read_info(obj_id)
            self.show_mesh()

        self.window.post_redraw()
        for ew in self.eye_windows.values():
            ew.scene.look_at(ew.center + ew.dir, ew.center, [0, 0, 1])
            ew.window.post_redraw()
            ew.get_image()
            ew.get_depth_image()
            if ew.save_images_bool:
                ew.save_images()
        if not self.gui.run_one_tick():
            self.is_alive = False
            self.gui.quit()

    class MenuCallback:
        """Callable handler bound to Open3D Save/Show menu items."""
        def __init__(self, menu_id: int, parent: Visualizer):
            """Bind a menu item ID to the owning ``Visualizer`` instance."""
            self.id = menu_id
            self.parent = parent
            self.path = None
            self.dialog_opened = False

        def __call__(self):
            """Dispatch the clicked menu action (image save, eye window toggle, or frame overlay)."""
            if self.id > 5:
                self.parent.menu.set_checked(self.id, not self.parent.menu.is_checked(self.id))
            if self.id == 0:
                self.parent.vis.scene.render_to_image(lambda im: self.save_image(im, 0))
            elif self.id == 1:
                self.parent.vis.scene.render_to_depth_image(lambda im: self.save_image(im, 1))
            elif self.id == 2:
                self.parent.eye_windows["l_eye"].vis.scene.render_to_image(lambda im: self.save_image(im, 0))
            elif self.id == 3:
                self.parent.eye_windows["l_eye"].vis.scene.render_to_depth_image(lambda im: self.save_image(im, 1))
            elif self.id == 4:
                self.parent.eye_windows["r_eye"].vis.scene.render_to_image(lambda im: self.save_image(im, 0))
            elif self.id == 5:
                self.parent.eye_windows["r_eye"].vis.scene.render_to_depth_image(lambda im: self.save_image(im, 1))
            elif self.id in [6, 7]:
                eye = "l_eye" if self.id == 6 else "r_eye"
                if eye in self.parent.eye_windows:
                    self.parent.eye_windows[eye].window.close()
                else:
                    self.parent.EyeWindow(eye, self.parent)
            elif self.id in [8, 9]:
                eye = "l_eye" if self.id == 8 else "r_eye"
                self.parent.eye_windows[eye].save_images_bool = not self.parent.eye_windows[eye].save_images_bool
            elif self.id == 10:
                if self.parent.menu.is_checked(self.id):
                    self.parent.vis.add_geometry("coordination_frame", o3d.geometry.TriangleMesh.create_coordinate_frame(size=1), self.parent.mat)
                else:
                    self.parent.vis.remove_geometry("coordination_frame")

        def save_image(self, im: o3d.geometry.Image, mode: int) -> None:
            """Prompt for an output path and write an RGB (mode 0) or depth (mode 1) image."""
            dia = gui.Dialog("path_save_dialog")
            fd = gui.FileDialog(gui.FileDialog.Mode.SAVE, "Where to save?", self.parent.window.theme)
            fd.set_on_done(self.input_completed)
            fd.set_on_cancel(self.input_completed)
            default_dir = os.path.normpath(os.path.join(self.parent.client.file_dir, ".."))
            if not is_writable_dir(default_dir):
                default_dir = str(ensure_user_workspace() / "images")
            fd.set_path(default_dir)
            dia.add_child(fd)
            self.dialog_opened = True
            self.parent.window.show_dialog(dia)

            self.wait_for_dialog_completion()
            self.parent.window.close_dialog()

            if self.path is None:
                return 0
            if mode == 1:
                im = o3d.geometry.Image((255 * np.asarray(im)).astype(np.uint8))
            o3d.io.write_image(self.path, im)

            self.parent.client.logger.info(f"File saved to {self.path}")
            self.path = None

        def wait_for_dialog_completion(self) -> None:
            """Pump Open3D GUI events until the save-file dialog closes."""
            while self.dialog_opened:
                self.parent.window.post_redraw()
                self.parent.gui.run_one_tick()

        def input_completed(self, text: Optional[str] = None):
            """Store the selected file path and mark the file dialog complete."""
            self.path = text
            self.dialog_opened = False

    def find_xyz_rpy(self, mesh_name: str, urdf_name: Optional[str] = "robot") -> Tuple[list, list, float, str]:
        """Look up visual origin offset, rotation, scale, and link name for ``mesh_name``."""
        for link in self.client.urdfs[urdf_name].links:
            if hasattr(link, "visual"):
                for idx in range(len(link.visual.geometry.mesh.filename)):
                    if os.path.basename(link.visual.geometry.mesh.filename) == mesh_name:
                        xyz = link.visual.origin.xyz
                        rpy = link.visual.origin.rpy
                        if hasattr(link.visual.geometry.mesh, "scale"):
                            scale = link.visual.geometry.mesh.scale[0]
                        else:
                            scale = 1
                        link_name = link.name
                        return xyz, rpy, scale, link_name

    class EyeWindow:
        """Secondary Open3D viewport rendering the view from ``l_eye`` or ``r_eye``."""
        MENU_IDS = {"l_eye": [2, 3, 8], "r_eye": [4, 5, 9]}
        POSITIONS = {"l_eye": [320, 560], "r_eye": [0, 560]}

        def __init__(self, eye: str, parent: Visualizer) -> None:
            """Create the eye camera window and mirror existing scene geometries into it."""
            self.eye = eye
            self.link_name = self.eye + "_pupil"
            self.parent = parent

            self.window = self.parent.gui.create_window(self.eye, 320, 240)
            self.window.set_on_close(self.on_close)

            self.scene = gui.SceneWidget()
            self.scene.set_on_mouse(self.on_mouse)
            self.scene.scene = rendering.Open3DScene(self.window.renderer)

            self.vis = self.scene.scene
            self.scene.background_color = o3d.visualization.gui.Color(1, 1, 1, 1)
            self.vis.set_background([1, 1, 1, 1])
            self.vis.set_lighting(self.vis.NO_SHADOWS, [0, 0, 0])
            self.window.show_menu(False)

            self.window.add_child(self.scene)

            self.center = None
            self.dir = None
            self.img_counter = 0
            self.save_images_bool = False
            self.last_image = None
            self.last_depth_image = None

            for menu_id in self.MENU_IDS[self.eye]:
                self.parent.file_menu.set_enabled(menu_id, True)

            for mesh in self.parent.meshes:
                self.vis.add_geometry(mesh, geometry=self.parent.meshes[mesh], material=self.parent.mat)
                self.vis.set_geometry_transform(mesh, self.parent.vis.get_geometry_transform(mesh))

            self.parent.eye_windows[self.eye] = self

        def on_close(self) -> bool:
            """Unregister this eye window and disable its stream menu entries."""
            for menu_id in self.MENU_IDS[self.eye]:
                self.parent.file_menu.set_enabled(menu_id, False)
            del self.parent.eye_windows[self.eye]
            return True

        def on_mouse(self, event: gui.MouseEvent) -> int:
            """Consume mouse events so users cannot orbit the fixed eye camera view."""
            return self.scene.EventCallbackResult.CONSUMED

        def get_image(self) -> None:
            """Capture an asynchronous RGB snapshot from the eye scene."""
            self.vis.scene.render_to_image(self.save_image)

        def get_depth_image(self) -> None:
            """Capture an asynchronous depth snapshot from the eye scene."""
            self.vis.scene.render_to_depth_image(self.save_depth_image)

        def save_images(self) -> None:
            """Save the latest captured RGB frame to disk and increment the frame counter."""
            parent_root = os.path.normpath(os.path.join(self.parent.file_dir, ".."))
            if is_writable_dir(parent_root):
                p = os.path.join(parent_root, "images", self.eye, "RGB")
            else:
                p = str(ensure_user_workspace() / "images" / self.eye / "RGB")
            if not os.path.exists(p):
                os.makedirs(p, exist_ok=True)
            p = os.path.join(p, str(self.img_counter) + ".png")
            self.img_counter += 1
            o3d.io.write_image(p, self.last_image)

        def save_image(self, im: o3d.geometry.Image) -> None:
            """Callback storing the latest rendered RGB image."""
            self.last_image = im

        def save_depth_image(self, im: o3d.geometry.Image) -> None:
            """Callback storing the latest rendered depth image."""
            self.last_depth_image = im

        def unproject(self, u, v, d):
            """Unproject pixel ``(u, v)`` at depth ``d`` into 3D world coordinates."""
            return self.vis.camera.unproject(u, v, d, self.window.size.width, self.window.size.height)
