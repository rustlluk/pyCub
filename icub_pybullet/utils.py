"""
Utils for pyCub simulator

:Author: Lukas Rustler
"""
import xml.etree.ElementTree as ET
from collections import namedtuple, _tuplegetter
from pathlib import Path
from typing import Any, Optional, Literal
import yaml
import os
import re
import numpy as np
import logging

try:
    from icub_pybullet.runtime_paths import (
        file_fingerprint,
        get_cached_urdf_path,
        get_user_cache_dir,
        is_writable_dir,
        resolve_config_path,
        to_posix,
        write_text_atomic,
    )
except Exception:
    from runtime_paths import (
        file_fingerprint,
        get_cached_urdf_path,
        get_user_cache_dir,
        is_writable_dir,
        resolve_config_path,
        to_posix,
        write_text_atomic,
    )


class URDF:
    """
    Class to parse URDF file
    """
    ROOT_TAGS = []

    def __init__(self, path: str, package_dir: Optional[str] = None):
        """

        :param path: path to the URDF file
        :type path: str
        :param package_dir: root directory for resolving package:// URIs
        :type package_dir: str, optional
        """
        tree = ET.parse(path)
        root = tree.getroot()
        self.path = os.path.abspath(path)
        self.package_dir = package_dir or os.path.dirname(os.path.abspath(__file__))
        self.links = []
        self.joints = []
        self.robot_name = root.attrib["name"]

        for child in root:
            if child.tag == "link":
                self.links.append(namedtuple(child.attrib["name"], []))
                self.read(child, self.links[-1])

            elif child.tag == "joint":
                self.joints.append(namedtuple(child.attrib["name"], []))
                self.read(child, self.joints[-1])

        self.ROOT_TAGS = None
        self.new_urdf = ''
        self.fix_urdf()
        self.make_references()

    def find_root_tags(self) -> None:
        """
        Finds tags that are 'root', i.e., they have child 'inside'
        """
        with open(self.path, "r", encoding="utf-8") as f:
            data = f.read()
        x = re.findall("^(?!.* name).*<(.*?) ", data, flags=re.MULTILINE)
        x = np.unique(x).tolist()
        self.ROOT_TAGS = x

    def read(self, el: ET, parent: ET) -> None:
        """
        Recursive function to read the URDF file. When there are no children, it reads the attributes and saves them.

        :param el: The current element in the XML tree.
        :type el: xml.etree.ElementTree.Element
        :param parent: The parent element in the XML tree.
        :type parent: xml.etree.ElementTree.Element
        """
        for child in el:
            childer_attrs = [_.tag for _ in child]
            # Check for existing fields, so we can append when something is twice for one link/joint (usually visual)
            if not hasattr(parent, child.tag) or isinstance(getattr(parent, child.tag), _tuplegetter):
                # create named tuple for all children
                setattr(parent, child.tag, namedtuple(child.tag, childer_attrs))
            # recursive call
            self.read(child, getattr(parent, child.tag))
        # read all attributes of leaf node and save it in LIST
        for attr, raw_val in el.attrib.items():
            parts = [p for p in raw_val.split(" ") if p != ""]
            if len(parts) > 1:
                try:
                    attr_val = list(map(float, parts))
                except ValueError:
                    attr_val = raw_val
            elif len(parts) == 1:
                try:
                    attr_val = float(parts[0])
                except ValueError:
                    attr_val = raw_val
            else:
                attr_val = raw_val
            setattr(parent, attr, attr_val)

    def dereference(self) -> None:
        """
        Make parent/child again as names to allow urdf write
        """
        for j in self.joints:
            if not hasattr(j.parent, "link"):
                for relative in ["parent", "child"]:
                    name = getattr(j, relative).name
                    delattr(j, relative)
                    setattr(j, relative, namedtuple(relative, ["link"]))
                    getattr(j, relative).link = name
        for l in self.links:
            if hasattr(l, "joint"):
                delattr(l, "joint")

    def make_references(self) -> None:
        """
        Make parent/child in joint list as references to the given link
        """
        for j in self.joints:
            for l in self.links:
                if hasattr(j.parent, "link"):
                    if j.parent.link == l.name:
                        j.parent = l
                        if not hasattr(l, "joint"):
                            l.joint = [j]
                        else:
                            l.joint.append(j)
                        break

            for l in self.links:
                if hasattr(j.child, "link"):
                    if j.child.link == l.name:
                        j.child = l
                        break

    def expand_package_uris(self) -> None:
        """Expand package:// mesh paths to absolute POSIX paths so cached URDFs load from any directory."""
        urdf_dir = os.path.dirname(self.path)
        for link in self.links:
            for visual_type in ("visual", "collision"):
                if hasattr(link, visual_type):
                    geom = getattr(link, visual_type).geometry
                    if hasattr(geom, "mesh") and hasattr(geom.mesh, "filename"):
                        fn = geom.mesh.filename
                        if isinstance(fn, str) and fn.startswith("package://"):
                            rel = fn.replace("package://", "")
                            cand = os.path.join(self.package_dir, rel)
                            if not os.path.exists(cand):
                                cand = os.path.join(urdf_dir, rel)
                            geom.mesh.filename = to_posix(cand)
                        elif isinstance(fn, str) and not os.path.isabs(fn):
                            cand = os.path.join(urdf_dir, fn)
                            if os.path.exists(cand):
                                geom.mesh.filename = to_posix(cand)

    def fix_urdf(self) -> None:
        """
        Fix the URDF file by converting non-mesh geometries to mesh and saving them as .obj files.
        If changes were made, write the new URDF to a writable directory or user cache.
        """
        something_changed = False
        parent_dir = os.path.dirname(self.path)
        writable_in_place = is_writable_dir(parent_dir)
        fp = file_fingerprint(self.path)

        for link in self.links:
            for visual_type in ["visual", "collision"]:
                if hasattr(link, visual_type):
                    geom = getattr(link, visual_type).geometry
                    if not hasattr(geom, "mesh"):
                        import open3d as o3d

                        mesh = None
                        if hasattr(geom, "box"):
                            mesh = o3d.geometry.TriangleMesh().create_box(*geom.box.size)
                            mesh.translate([0, 0, 0], relative=False)
                            delattr(geom, "box")
                        elif hasattr(geom, "cylinder"):
                            mesh = o3d.geometry.TriangleMesh().create_cylinder(
                                radius=geom.cylinder.radius,
                                height=geom.cylinder.length,
                            )
                            delattr(geom, "cylinder")
                        elif hasattr(geom, "sphere"):
                            mesh = o3d.geometry.TriangleMesh().create_sphere(radius=geom.sphere.radius)
                            delattr(geom, "sphere")

                        if mesh is not None:
                            setattr(geom, "mesh", namedtuple("mesh", ["filename"]))
                            if writable_in_place:
                                mesh_dir = os.path.normpath(os.path.join(parent_dir, "meshes", "fixed_urdf", visual_type))
                            else:
                                mesh_dir = str(
                                    get_user_cache_dir()
                                    / "fixed_urdf"
                                    / f"{Path(self.path).stem}_{fp}"
                                    / visual_type
                                )
                            os.makedirs(mesh_dir, exist_ok=True)
                            mesh_file = os.path.join(mesh_dir, link.name + ".obj")
                            if not os.path.exists(mesh_file):
                                o3d.io.write_triangle_mesh(mesh_file, mesh)
                            geom.mesh.filename = to_posix(mesh_file)
                            something_changed = True

        if something_changed:
            if self.ROOT_TAGS is None:
                self.find_root_tags()
            if "mesh" not in self.ROOT_TAGS:
                self.ROOT_TAGS.append("mesh")
            if not writable_in_place:
                self.expand_package_uris()
                out_urdf = get_cached_urdf_path(Path(self.path).stem + "_fixed", fp, subdir="fixed_urdf")
            else:
                out_urdf = self.path.replace(".urdf", "_fixed.urdf")
            self.write_urdf()
            write_text_atomic(out_urdf, self.new_urdf)
            self.path = out_urdf

    def write_attr(self, attr_name: str, attr: Any, level: Optional[int] = 1, skip_header: Optional[bool] = False) -> None:
        """
        Write an attribute to the new URDF string.
        """
        if not skip_header and attr_name in self.ROOT_TAGS:
            self.new_urdf += ' ' * level * 4 + '<' + attr_name
        elif not skip_header:
            if hasattr(attr, "name"):
                self.new_urdf += ' ' * level * 4 + '<' + attr_name + ' name="' + attr.name + '">\n'
            else:
                self.new_urdf += ' ' * level * 4 + '<' + attr_name + '>\n'

        if hasattr(attr, "__dict__"):
            for inner_attr_name, inner_attr in attr.__dict__.items():
                if inner_attr_name[0] != "_" and inner_attr_name != "name":
                    self.write_attr(inner_attr_name, inner_attr, level + 1, attr_name in self.ROOT_TAGS)
        else:
            self.new_urdf += ' ' + attr_name + '="'
            if attr_name != "name":
                if isinstance(attr, str):
                    self.new_urdf += attr + '"'
                else:
                    try:
                        self.new_urdf += ' '.join(map(str, attr)) + '"'
                    except Exception:
                        self.new_urdf += ' '.join(map(str, [attr])) + '"'

        if not skip_header and attr_name in self.ROOT_TAGS:
            self.new_urdf += '/>\n'
        elif not skip_header:
            self.new_urdf += ' ' * level * 4 + '</' + attr_name + '>\n'

    def write_urdf(self) -> None:
        """
        Write the URDF object to a string.
        """
        if self.ROOT_TAGS is None:
            self.find_root_tags()
        self.dereference()

        self.new_urdf = '<robot name="' + self.robot_name + '">\n'
        for link in self.links:
            self.new_urdf += '<link name="' + link.name + '">\n'
            for attr_name, attr in link.__dict__.items():
                if attr_name[0] != "_" and attr_name != "name":
                    self.write_attr(attr_name, attr)

            self.new_urdf += '</link>\n'

        for joint in self.joints:
            self.new_urdf += '<joint name="' + joint.name + '" type="' + joint.type + '">\n'
            for attr_name, attr in joint.__dict__.items():
                if attr_name[0] != "_" and attr_name != "name" and attr_name != "type":
                    self.write_attr(attr_name, attr)

            self.new_urdf += '</joint>\n'
        self.new_urdf += '</robot>'

        self.make_references()


def _deep_merge_dict(base: dict, override: dict) -> dict:
    """Recursively merge ``override`` into ``base``, ignoring the ``parent`` key."""
    merged = dict(base)
    for k, v in override.items():
        if k == "parent":
            continue
        if isinstance(v, dict) and isinstance(merged.get(k), dict):
            merged[k] = _deep_merge_dict(merged[k], v)
        else:
            merged[k] = v
    return merged


def _load_yaml_with_inheritance(config_path: str, package_dir: str, _visited: Optional[set] = None) -> dict:
    """Load a YAML config file and recursively resolve optional ``parent`` inheritance."""
    visited = _visited or set()
    abs_path = os.path.abspath(config_path)
    if abs_path in visited:
        raise ValueError(f"Circular config parent inheritance detected at {abs_path}")
    visited.add(abs_path)

    with open(abs_path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}

    parent_ref = raw.get("parent")
    if parent_ref:
        parent_path = resolve_config_path(str(parent_ref), package_dir, caller_file=abs_path)
        parent_dict = _load_yaml_with_inheritance(parent_path, package_dir, visited)
        return _deep_merge_dict(parent_dict, raw)
    return raw


class Config:
    """
    Class to parse and keep the config loaded from yaml file.
    Supports optional `parent: <base.yaml>` inheritance so user-defined custom worlds can override only what they need.
    """
    def __init__(self, config_path: str):
        """
        :param config_path: path to the config file
        :type config_path: str
        """
        package_dir = os.path.dirname(os.path.abspath(__file__))
        config_dict = _load_yaml_with_inheritance(config_path, package_dir)

        for attr, value in config_dict.items():
            self.set_attribute(attr, value, self)

        required_attributes = {
            "vhacd": ["use_vhacd", "force_vhacd", "force_vhacd_urdf"],
            "robot_urdf_path": [],
            "gui": ["standard", "web"],
            "tolerance": ["joint"],
            "skin": ["use", "radius", "num_cores", "skin_parts"],
            "collision_tolerance": [],
            "end_effector": [],
            "debug": [],
            "log": ["log", "period"],
            "simulation_step": [],
            "self_collisions": [],
            "eyes": ["l_eye", "r_eye"],
        }
        for attr in required_attributes:
            if not hasattr(self, attr):
                raise AttributeError(f"Missing attribute {attr} in config file {config_path}")
            for sub_attr in required_attributes[attr]:
                if not hasattr(getattr(self, attr), sub_attr):
                    raise AttributeError(f"Missing attribute {attr}.{sub_attr} in config file {config_path}")

    def set_attribute(self, attr: str, value: Any, reference: int) -> int:
        """Recursively assign dictionary entries as named attributes on ``reference``.

        :param attr: Attribute name to assign.
        :param value: Scalar or nested dictionary value.
        :param reference: Target object or namedtuple receiving the attribute.
        """
        if not isinstance(value, dict):
            setattr(reference, attr, value)
            return 0
        else:
            setattr(reference, attr, namedtuple(attr, list(value.keys())))
            for inner_attr, inner_value in value.items():
                self.set_attribute(inner_attr, inner_value, getattr(reference, attr))
        return 0


class Pose:
    """
    Mini help class for Pose representation
    """
    def __init__(self, pos: list, ori: list):
        """Initialize a pose from 3D position and quaternion orientation.

        :param pos: Cartesian position ``[x, y, z]``.
        :param ori: Orientation quaternion ``[x, y, z, w]``.
        """
        self.pos = pos
        self.ori = ori

    def __str__(self) -> str:
        """Return a human-readable position and orientation summary."""
        return f"position: {self.pos}, orientation: {self.ori}"

    def to_string(self) -> str:
        """Serialize position and orientation components as a semicolon-delimited string."""
        return ";".join(map(str, self.pos)) + ";" + ";".join(map(str, self.ori))


class CustomFormatter(logging.Formatter):
    """
    Custom formatter that assigns colors to logs
    From https://stackoverflow.com/a/56944256
    """

    grey = "\x1b[97;10m"
    yellow = "\x1b[33;20m"
    red = "\x1b[31;20m"
    bold_red = "\x1b[31;1m"
    reset = "\x1b[0m"
    format = '%(module)s %(levelname)s: %(message)s'

    FORMATS = {
        logging.DEBUG: grey + format + reset,
        logging.INFO: grey + format + reset,
        logging.WARNING: yellow + format + reset,
        logging.ERROR: red + format + reset,
        logging.CRITICAL: bold_red + format + reset
    }

    def format(self, record: logging.LogRecord) -> str:
        """Format ``record`` using the ANSI color mapped to its log level."""
        log_fmt = self.FORMATS.get(record.levelno)
        formatter = logging.Formatter(log_fmt)
        return formatter.format(record)


JOINTS = Literal[
    'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll', 'l_hip_pitch',
    'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll', 'torso_pitch', 'torso_roll', 'torso_yaw',
    'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow', 'r_wrist_prosup', 'r_wrist_pitch', 'r_wrist_yaw',
    'r_hand_thumb_0_joint', 'r_hand_thumb_1_joint', 'r_hand_thumb_2_joint', 'r_hand_thumb_3_joint', 'r_hand_index_0_joint',
    'r_hand_index_1_joint', 'r_hand_index_2_joint', 'r_hand_index_3_joint', 'r_hand_middle_0_joint',
    'r_hand_middle_1_joint', 'r_hand_middle_2_joint', 'r_hand_middle_3_joint', 'r_hand_ring_0_joint',
    'r_hand_ring_1_joint', 'r_hand_ring_2_joint', 'r_hand_ring_3_joint', 'r_hand_little_0_joint', 'r_hand_little_1_joint',
    'r_hand_little_2_joint', 'r_hand_little_3_joint', 'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow',
    'l_wrist_prosup', 'l_wrist_pitch', 'l_wrist_yaw', 'l_hand_thumb_0_joint', 'l_hand_thumb_1_joint',
    'l_hand_thumb_2_joint', 'l_hand_thumb_3_joint', 'l_hand_index_0_joint', 'l_hand_index_1_joint', 'l_hand_index_2_joint',
    'l_hand_index_3_joint', 'l_hand_middle_0_joint', 'l_hand_middle_1_joint', 'l_hand_middle_2_joint',
    'l_hand_middle_3_joint', 'l_hand_ring_0_joint', 'l_hand_ring_1_joint', 'l_hand_ring_2_joint', 'l_hand_ring_3_joint',
    'l_hand_little_0_joint', 'l_hand_little_1_joint', 'l_hand_little_2_joint', 'l_hand_little_3_joint', 'neck_pitch',
    'neck_roll', 'neck_yaw', 'eyes_tilt', 'l_eye_pan_joint', 'r_eye_pan_joint'
]

JOINTS_IDS = Literal[
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
    27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56,
    57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74
]

CHAINS = Literal["left_arm", "right_arm", "left_leg", "right_leg", "torso", "head"]
