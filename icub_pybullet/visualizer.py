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


class Visualizer:
    """Class to help with custom rendering"""

    def __init__(self, client: pyCub):
        """

        :param client: pyCub instance
        :type client: pyCub
        """
        self.client = client

        # if the web gui is enabled, enable webrtc server
        if self.client.config.gui.web:
            o3d.visualization.webrtc_server.enable_webrtc()

        # prepare empty dictionaries
        self.meshes = {}
        self.meshes = {}
        self.R_urdf = {}
        self.skin_activated_last_turn = {}

        self.blue = o3c.Tensor([0, 0, 1], dtype=o3c.Dtype.Float32)
        self.red = o3c.Tensor([1, 0, 0], dtype=o3c.Dtype.Float32)

        self._R = np.eye(4)
        self._R_ori = self._R[:3, :3]

        if self.client.config.skin.use:
            self.fpath_to_skin = {}

        # Init GUI, windows, etc.
        self.gui = gui.Application.instance
        self.gui.initialize()
        self.window = self.gui.create_window("pyCub", 640, 480, 0, 0)

        self.gui.menubar = gui.Menu()
        self.menu = self.gui.menubar
        self.scene = gui.SceneWidget()
        self.scene.scene = rendering.Open3DScene(self.window.renderer)

        self.vis = self.scene.scene
        self.scene.background_color = o3d.visualization.gui.Color(1, 1, 1, 1)
        self.vis.set_background([1, 1, 1, 1])
        self.vis.set_lighting(self.vis.NO_SHADOWS, [0, 0, 0])

        # prepare default material
        self.mattr = rendering.MaterialRecord()
        self.mattr.shader = 'defaultLitTransparency'
        if hasattr(self.client.config.gui, "pycub_alpha"):
            PYCUB_ALPHA = self.client.config.gui.pycub_alpha
        else:
            PYCUB_ALPHA = 1
        self.mattr.base_color = [1.0, 1.0, 1.0, PYCUB_ALPHA]

        self.mat = rendering.MaterialRecord()
        self.mat.shader = 'defaultLit'

        # Prepare the menu
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

        self.show_menu.add_item("Coordination Frame", menu_counter+4)
        show_callbacks.append(self.MenuCallback(menu_counter+4, self))
        self.window.set_on_menu_item_activated(menu_counter+4, show_callbacks[-1])

        self.window.add_child(self.scene)

        self.file_dir = os.path.dirname(os.path.abspath(__file__))
        self.orientations = []
        self.positions = []
        self.colors = []
        self.paths = []
        self.eye_windows = {}

        # Run this, to load the initial meshes, compute bounding boxes and init camera and center of rotation
        self.read_info(self.client.robot)
        self.show_first()
        self.show_mesh()  # this is here only for bounding box computation

        # compute center of all objects
        scene_mesh = o3d.geometry.TriangleMesh()
        for f_path, m in self.meshes.items():
            R = self.vis.get_geometry_transform(f_path)
            m.transform(R)
            scene_mesh += m
            m.transform(np.linalg.inv(R))
        bbox = scene_mesh.get_axis_aligned_bounding_box()
        center = bbox.get_center()

        # look at the center
        self.scene.look_at(center, center+[-1, 0, 0], [0, 0, 1])
        self.scene.center_of_rotation = center

        # show all "other objects" -> non-robot
        for obj_id, obj_name, _, _, _ in self.client.other_objects:
            self.read_info(obj_id)
            self.show_first(urdf_name=obj_name)
            self.show_mesh()

        self.is_alive = True
        if self.client.config.eyes.l_eye:
            show_callbacks[0]()
        if self.client.config.eyes.r_eye:
            show_callbacks[1]()

        # Open new tab (or new window) of default browser with the web gui
        if self.client.config.gui.web:
            webbrowser.open("http://localhost:8888", new=0)

    def show_first(self, urdf_name: Optional[str] = "robot") -> None:
        """
        Show the first batch of meshes in the visualizer. It loads the meshes and saves the to dict for quicker use later

        :param urdf_name: The name of the urdf to be used.
        :type urdf_name: str, optional, default="robot"
        """
        for mesh_id in range(len(self.positions) // 3):
            # get correct values for given mesh
            col = self.colors[mesh_id * 3:(mesh_id + 1) * 3]
            f_path = self.paths[mesh_id]

            self.meshes[f_path] = o3d.io.read_triangle_mesh(f_path)

            # Just for visualization
            if not self.meshes[f_path].has_triangle_normals():
                self.meshes[f_path].compute_triangle_normals()
            if not self.meshes[f_path].has_vertex_normals():
                self.meshes[f_path].compute_vertex_normals()

            self.meshes[f_path].paint_uniform_color(col[:3])

            # URDF rotations and translations
            init_xyz, init_rpy, scale, link_name = self.find_xyz_rpy(os.path.basename(f_path), urdf_name=urdf_name)

            R_urdf = np.eye(4)
            R_urdf[:3, :3] = np.reshape(self.client.getMatrixFromQuaternion(self.client.getQuaternionFromEuler(init_rpy)), (3, 3))
            R_urdf[:3, 3] = init_xyz

            self.R_urdf[f_path] = R_urdf

            self.meshes[f_path].scale(scale, self.meshes[f_path].get_center())
            self.meshes[f_path].translate(self.meshes[f_path].get_center()*scale, relative=False)

            # Add the mesh
            if urdf_name == "robot":
                self.vis.add_geometry(f_path, geometry=self.meshes[f_path], material=self.mattr)
            else:
                self.vis.add_geometry(f_path, geometry=self.meshes[f_path], material=self.mat)

            if self.client.config.skin.use:
                if link_name in self.client.skin_point_clouds:
                    self.fpath_to_skin[f_path] = link_name
                    self.vis.add_geometry(f_path+"_skin", geometry=self.client.skin_point_clouds[link_name], material=self.mat)
                    self.skin_activated_last_turn[f_path] = 1
                else:
                    self.fpath_to_skin[f_path] = None

    def show_mesh(self) -> None:
        """
        Function to parse info about meshes from PyBullet

        """
        for mesh_id in range(len(self.positions) // 3):
            # get correct values for given mesh
            pos = self.positions[mesh_id * 3:(mesh_id + 1) * 3]
            ori = self.orientations[mesh_id * 4:(mesh_id + 1) * 4]
            f_path = self.paths[mesh_id]

            R_urdf = self.R_urdf[f_path]

            # get ori and position as 4x4 transformation matrix

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
                    ew.dir = 0.1*ew.dir/np.linalg.norm(ew.dir)

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
                        rendering.Scene.UPDATE_COLORS_FLAG
                    )

                self.vis.set_geometry_transform(f_path+"_skin", self._R)

    def read_info(self, obj_id: int) -> int:
        """
        Read info from PyBullet

        :param obj_id: id of the object; given by pybullet
        :type obj_id: int
        :return: 0 for success
        :rtype: int
        """
        # All meshes from the current object
        visualData = self.client.getVisualShapeData(obj_id)
        linkStates = self.client.getLinkStates(obj_id, range(self.client.getNumJoints(obj_id)), computeLinkVelocity=0,
                                               computeForwardKinematics=0)

        self.positions = []
        self.orientations = []
        self.colors = []
        self.paths = []

        for m in visualData:
            # Get information about individual parts of the object
            f_path = m[self.client.visualShapeData["FILE"]].decode("utf-8")
            if f_path == "":
                continue

            col = m[self.client.visualShapeData["COLOR"]]
            link = m[self.client.visualShapeData["LINK"]]

            # non-base links
            if link != -1:
                # get link info
                linkState = linkStates[link]
                # get orientation and position wrt URDF - better than in world
                ori = linkState[self.client.linkInfo["URDFORI"]]
                pos = linkState[self.client.linkInfo["URDFPOS"]]

            # link == -1 is base. For that, getBasePosition... needs to be used. This joint must not contain URDF visual xyz and rpy
            else:
                pos, ori = self.client.getBasePositionAndOrientation(obj_id)

            self.positions += pos
            self.orientations += ori
            self.colors += col[:-1]
            self.paths.append(f_path)

        return 0

    def render(self) -> None:
        """
        Render all the things

        """
        # read info
        self.read_info(self.client.robot)
        # add new
        self.show_mesh()

        for obj_id, _, fixed, _, _ in self.client.other_objects:
            # if not fixed:
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
        """
        Class to handle menu callbacks.
        """
        def __init__(self, menu_id: int, parent: Visualizer):
            """
            Initialize the MenuCallback class.

            :param menu_id: The id of the menu.
            :type menu_id: int
            :param parent: The parent class (Visualizer).
            :type parent: pointer to the class of visualizer.Visualizer type
            """
            self.id = menu_id
            self.parent = parent
            self.path = None
            self.dialog_opened = False

        def __call__(self):
            """
            What happens when the menu is clicked

            """

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

                # When already opened -> close
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
            """
            Save the image. It shows FileDialog to find path for image save. It saves it with the current resolution
            of the window.

            :param im: The image to be saved.
            :type im: o3d.geometry.Image
            :param mode: The mode of the image. 0 for RGB, 1 for depth.
            :type mode: int
            """
            dia = gui.Dialog("path_save_dialog")
            fd = gui.FileDialog(gui.FileDialog.Mode.SAVE, "Where to save?", self.parent.window.theme)
            fd.set_on_done(self.input_completed)
            fd.set_on_cancel(self.input_completed)
            fd.set_path(os.path.normpath(os.path.join(self.parent.client.file_dir, "..")))
            dia.add_child(fd)
            self.dialog_opened = True
            self.parent.window.show_dialog(dia)

            self.wait_for_dialog_completion()

            self.parent.window.close_dialog()

            if self.path is None:
                return 0
            if mode == 1:
                im = o3d.geometry.Image((255*np.asarray(im)).astype(np.uint8))
            o3d.io.write_image(self.path, im)

            self.parent.client.logger.info(f"File saved to {self.path}")
            self.path = None

        def wait_for_dialog_completion(self) -> None:
            """
            Help function to keep the gui loop running

            """
            while self.dialog_opened:
                self.parent.window.post_redraw()
                self.parent.gui.run_one_tick()

        def input_completed(self, text: Optional[str] = None):
            """
            Callback for the dialog

            :param text: input text
            :type text: str
            :return:
            :rtype:
            """
            self.path = text
            self.dialog_opened = False

    def find_xyz_rpy(self, mesh_name: str, urdf_name: Optional[str] = "robot") -> Tuple[list, list, float, str]:
        """
        Find the xyz, rpy and scales values.

        :param mesh_name: The name of the mesh.
        :type mesh_name: str
        :param urdf_name: The name of the urdf.
        :type urdf_name: str, optional, default="robot"
        :return: The xyz, rpy, and scales, link_name
        :rtype: list, list, float, str
        """
        """
        

        :param mesh_name: The name of the mesh.
        :type mesh_name: str
        :param urdf_name: The name of the urdf.
        :type urdf_name: str, optional, default="robot"
        :return: The xyz, rpy, and scales, link_name
        """

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

        MENU_IDS = {"l_eye": [2, 3, 8], "r_eye": [4, 5, 9]}
        POSITIONS = {"l_eye": [320, 560], "r_eye": [0, 560]}

        def __init__(self, eye: str, parent: Visualizer) -> None:
            """
            Class to handle windows for eye rendering

            :param eye: name of the eye
            :type eye: str
            :param parent: The parent class (Visualizer).
            :type parent: Visualizer
            """
            self.eye = eye
            self.link_name = self.eye + "_pupil"
            self.parent = parent

            self.window = self.parent.gui.create_window(self.eye, 320, 240,
                                                        self.POSITIONS[self.eye][0], self.POSITIONS[self.eye][1])
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

            # Add all existing meshes to the visualizer and transform them to current position
            for mesh in self.parent.meshes:
                self.vis.add_geometry(mesh, geometry=self.parent.meshes[mesh], material=self.parent.mat)
                # This is needed basically only for stationary objects, but it does not really matter if done for all
                self.vis.set_geometry_transform(mesh, self.parent.vis.get_geometry_transform(mesh))

            self.parent.eye_windows[self.eye] = self

        def on_close(self) -> bool:
            """
            Small function to delete the window from the parent class

            """
            for menu_id in self.MENU_IDS[self.eye]:
                self.parent.file_menu.set_enabled(menu_id, False)
            del self.parent.eye_windows[self.eye]
            return True

        def on_mouse(self, event: gui.MouseEvent) -> int:
            """
            Small function to ignore mouse events

            :param event: Mouse event
            :type event: gui.MouseEvent
            """
            return self.scene.EventCallbackResult.CONSUMED

        def get_image(self) -> None:
            """
            Small function to get image from open3d

            :return:
            :rtype:
            """
            self.vis.scene.render_to_image(self.save_image)

        def get_depth_image(self) -> None:
            """
            Small function to get image from open3d

            :return:
            :rtype:
            """
            self.vis.scene.render_to_depth_image(self.save_depth_image)

        def save_images(self) -> None:
            """
            Function to save stream of images to file

            """
            p = os.path.join(self.parent.file_dir, "..", "images", self.eye, "RGB")
            if not os.path.exists(p):
                os.makedirs(p)
            p = os.path.join(p, str(self.img_counter)+".png")
            self.img_counter += 1
            o3d.io.write_image(p, self.last_image)

        def save_image(self, im: o3d.geometry.Image) -> None:
            """
            Callback to get images from open3d
            
            :param im: the image to be saves
            :type im: o3d.geometry.Image
            """

            self.last_image = im

        def save_depth_image(self, im: o3d.geometry.Image) -> None:
            """
            Callback to get images from open3d

            :param im: the image to be saves
            :type im: o3d.geometry.Image
            """

            self.last_depth_image = im

        def unproject(self, u, v, d):
            return self.vis.camera.unproject(u, v, d, self.window.size.width, self.window.size.height)
