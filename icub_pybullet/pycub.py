"""
The main class and utils for pyCub simulator

:Author: Lukas Rustler
"""
from __future__ import annotations
import glob
import shutil
import time
from pathlib import Path
import numpy as np
import pybullet as p
from pybullet_utils.bullet_client import BulletClient
import os
import sys
try:
    from icub_pybullet.utils import Config, URDF, Pose, CustomFormatter, JOINTS, JOINTS_IDS, CHAINS
    from icub_pybullet.runtime_paths import (
        file_fingerprint,
        get_cached_urdf_path,
        get_user_cache_dir,
        get_user_log_dir,
        get_vhacd_cache_path,
        resolve_config_path,
        resolve_mesh_or_urdf_path,
        resolve_robot_urdf_path,
        to_posix,
        write_text_atomic,
    )
except Exception:
    from utils import Config, URDF, Pose, CustomFormatter, JOINTS, JOINTS_IDS, CHAINS
    from runtime_paths import (
        file_fingerprint,
        get_cached_urdf_path,
        get_user_cache_dir,
        get_user_log_dir,
        get_vhacd_cache_path,
        resolve_config_path,
        resolve_mesh_or_urdf_path,
        resolve_robot_urdf_path,
        to_posix,
        write_text_atomic,
    )
import logging
import datetime
import psutil
import atexit
from typing import Any, Callable, Tuple, Optional, List


def __getattr__(name: str) -> Any:
    """Lazy module attributes for heavy optional dependencies (`open3d`, `roboticstoolbox`, `Visualizer`)."""
    if name == "Visualizer":
        try:
            from icub_pybullet.visualizer import Visualizer
        except Exception:
            from visualizer import Visualizer
        return Visualizer
    if name == "o3d":
        import open3d as o3d
        return o3d
    if name == "o3c":
        import open3d.core as o3c
        return o3c
    if name == "rtb":
        import roboticstoolbox as rtb
        return rtb
    if name == "URDFROBOT":
        try:
            from roboticstoolbox.models.URDF.URDFRobot import URDFRobot  # noqa: F401
            return True
        except Exception:
            return False
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


# Pluggable hooks used by studio for embedded native OpenGL rendering and UI status reporting
_VISUALIZER_FACTORY: Optional[Callable[["pyCub"], Any]] = None
_STATUS_CALLBACK: Optional[Callable[[str, str], None]] = None


def set_visualizer_factory(factory: Optional[Callable[["pyCub"], Any]]) -> None:
    """Register a custom visualizer factory (e.g. EmbeddedVisualizerProxy in the standalone desktop app)."""
    global _VISUALIZER_FACTORY
    _VISUALIZER_FACTORY = factory


def get_visualizer_factory() -> Optional[Callable[["pyCub"], Any]]:
    """Return the currently registered custom visualizer factory, or ``None``."""
    return _VISUALIZER_FACTORY


def set_status_callback(callback: Optional[Callable[[str, str], None]]) -> None:
    """Register an optional status callback receiving (stage, message) updates (e.g. during dynamic VHACD)."""
    global _STATUS_CALLBACK
    _STATUS_CALLBACK = callback


class Joint:
    """Container describing a movable PyBullet robot joint and its active control target."""
    def __init__(self, name: str, robot_joint_id: int, joints_id: int, lower_limit: float, upper_limit: float,
                 max_force: float, max_velocity: float):
        """Initialize joint metadata and limits from PyBullet joint info."""
        self.name = name
        self.robot_joint_id = robot_joint_id
        self.joints_id = joints_id
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit
        self.max_force = max_force
        self.max_velocity = max_velocity
        self.set_point = None
        self.start_time = None
        self.timeout = None

    def __repr__(self) -> str:
        """Return a readable identifier for the joint."""
        return f"Joint {self.name} with id {self.robot_joint_id}"


class Link:
    """Container mapping a robot link name to its PyBullet and URDF indices."""
    def __init__(self, name: str, robot_link_id: int, urdf_link: int):
        """Initialize link name and index references."""
        self.name = name
        self.robot_link_id = robot_link_id
        self.urdf_link = urdf_link


class pyCub(BulletClient):
    """
    Client class which inherits from BulletClient and contains the whole simulation functionality
    """
    jointInfo = {name: i for i, name in enumerate(["INDEX", "NAME", "TYPE", "QINDEX", "UINDEX", "FLAGS", "DAMPING",
                                                   "FRICTION", "LOWERLIMIT", "UPPERLIMIT", "MAXFORCE", "MAXVELOCITY",
                                                   "LINKNAME", "AXIS", "PARENTPOS", "PARENTORN", "PARENTINDEX"])}
    jointStates = {name: i for i, name in enumerate(["POSITION", "VELOCITY", "FORCES", "TORQUE"])}

    linkInfo = {name: i for i, name in enumerate(["WORLDPOS", "WORLDORI", "INERTIAPOS", "INERTIAORI", "URDFPOS",
                                                  "URDFORI", "LINVEL", "ANGVEL"])}

    contactPoints = {name: i for i, name in enumerate(["FLAG", "IDA", "IDB", "INDEXA", "INDEXB", "POSITIONA",
                                                       "POSITIONB", "NORMAL", "DISTANCE", "FORCE",
                                                       "FRICTION1", "FRICTIONDIR1", "FRICTION2", "FRICTIONDIR2"])}
    dynamicsInfo = {name: i for i, name in enumerate(["MASS", "FRICTION", "INTERTIADIAGONAL", "INERTIAPOS", "INERTIAOR",
                                                      "RESTITUTION", "ROLLINGFRICTION", "SPINNINGFRICTION", "DAMPING",
                                                      "STIFFNESS", "BODYTYPE", "MARGIN"])}
    visualShapeData = {name: i for i, name in enumerate(["ID", "LINK", "GEOMTYPE", "DIMS", "FILE", "POS", "ORI",
                                                         "COLOR", "TEXTURE"])}

    def __init__(self, config: Optional[str] = "default.yaml"):
        """
        :param config: path or name of the config file
        :type config: str, optional, default="default.yaml"
        """
        super().__init__(p.DIRECT)

        caller_filename = None
        raw_caller = None
        try:
            raw_caller = sys._getframe(1).f_code.co_filename
        except (AttributeError, ValueError):
            raw_caller = None
        if raw_caller and not raw_caller.startswith("<") and os.path.exists(raw_caller):
            caller_filename = os.path.abspath(raw_caller)
        self.parent_name = os.path.basename(raw_caller) if raw_caller else "pycub"

        self.file_dir = os.path.dirname(os.path.abspath(__file__))

        # Ensure ROS_PACKAGE_PATH contains self.file_dir so roboticstoolbox / xacrodoc resolves package://iCub/...
        existing_ros_path = os.environ.get("ROS_PACKAGE_PATH", "")
        ros_parts = [pt for pt in existing_ros_path.split(os.pathsep) if pt]
        if self.file_dir not in ros_parts:
            os.environ["ROS_PACKAGE_PATH"] = os.pathsep.join([self.file_dir] + ros_parts)

        self.config_path = resolve_config_path(config, self.file_dir, caller_filename)
        self.config_dir = os.path.dirname(self.config_path)
        self.config = Config(self.config_path)

        self.config.simulation_step = 1 / self.config.simulation_step
        self.setTimeStep(self.config.simulation_step)

        force_embedded = os.environ.get("PYCUB_EMBEDDED_VIEWER") == "1" or _VISUALIZER_FACTORY is not None
        if self.config.gui.standard or self.config.gui.web or (force_embedded and os.environ.get("PYCUB_FORCE_HEADLESS") != "1"):
            self.gui = True
            if self.config.gui.standard and self.config.gui.web:
                self.config.gui.web = False
        else:
            self.gui = False

        if self.gui and _VISUALIZER_FACTORY is None and os.environ.get("PYCUB_DISABLE_KILL_OPEN3D") != "1":
            atexit.register(self.kill_open3d)

        if self.gui:
            self.default_step = 0.01
        else:
            self.default_step = None

        self.logger = logging.getLogger("pycub_logger")
        self.logger.setLevel(logging.DEBUG if self.config.debug else logging.INFO)
        if not self.logger.handlers:
            stream_handler = logging.StreamHandler()
            stream_handler.setFormatter(CustomFormatter())
            self.logger.addHandler(stream_handler)

        if hasattr(self.config, "log_pose"):
            self.log_pose = self.config.log_pose
            self.pose_logger = []
        else:
            self.log_pose = False

        self.gravity = False

        self.urdf_path = resolve_robot_urdf_path(self.config.robot_urdf_path, self.file_dir, self.config_dir)
        self.urdfs = {"robot": URDF(self.urdf_path, package_dir=self.file_dir)}

        if self.config.vhacd.use_vhacd:
            self.run_vhacd()
        else:
            # Ensure package:// URIs are resolved to POSIX file paths if needed on Windows/frozen builds
            self.urdfs["robot"].expand_package_uris()
            cached_robot_urdf = get_cached_urdf_path(
                Path(self.urdfs["robot"].path).stem + "_resolved",
                f"{self.urdfs['robot'].path}|{file_fingerprint(self.urdfs['robot'].path)}",
                subdir="generated_urdf",
            )
            if not os.path.isfile(cached_robot_urdf):
                self.urdfs["robot"].write_urdf()
                write_text_atomic(cached_robot_urdf, self.urdfs["robot"].new_urdf)
            self.urdfs["robot"].path = cached_robot_urdf

        self.urdf_path = self.urdfs["robot"].path
        self.other_objects = []
        self.free_objects = []

        self.robot, self.joints, self.links = self.init_robot()

        # prepare IK config so we can utilize null space
        self.IK_config = {
            "movable_joints": [_.joints_id for _ in self.joints if "_hand_" not in _.name],
            "lower_limits": [_.lower_limit + 0.025 * _.lower_limit for _ in self.joints],
            "upper_limits": [_.upper_limit - 0.025 * _.upper_limit for _ in self.joints],
            "joint_ranges": [np.abs(_.upper_limit - _.lower_limit) for _ in self.joints],
            "rest_poses": [
                0 if not hasattr(self.config.initial_joint_angles, _.name)
                else np.deg2rad(getattr(self.config.initial_joint_angles, _.name))
                for _ in self.joints
            ],
        }

        self.end_effector = EndEffector(self.config.end_effector, self)

        self.last_step = time.time()
        self.last_log = time.time()
        self.joint_tolerance = float(self.config.tolerance.joint)

        self.neighbour_links = {}
        if self.config.skin.use:
            import open3d as o3d
            import open3d.core as o3c
            self.skin_point_clouds = {}
            self.skin = {}
            self.skin_activations = {}
            self.activated_skin_points = {}
            self.activated_skin_normals = {}
            with open(os.path.join(self.file_dir, "iCub", "skin", "point_clouds", "config.txt"), "r", encoding="utf-8") as f:
                skin_config = {_.split(";")[0]: _.split(";")[1] for _ in f.read().splitlines() if ";" in _}
            if len(self.config.skin.skin_parts) == 0:
                skin_pcds = glob.glob(os.path.join(self.file_dir, "iCub", "skin", "point_clouds", "*.pcd"))
            else:
                skin_pcds = [
                    os.path.join(self.file_dir, "iCub", "skin", "point_clouds", f"{_}.pcd")
                    for _ in self.config.skin.skin_parts
                ]
            for pc_path in skin_pcds:
                pc = o3d.io.read_point_cloud(pc_path)
                pc.normalize_normals()
                if "foot" not in pc_path:
                    pc.scale(1.05, pc.get_center())

                skin_part = skin_config[os.path.basename(pc_path).split(".")[0]]
                self.skin_point_clouds[skin_part] = o3d.t.geometry.PointCloud.from_legacy(
                    pc, o3c.Dtype.Float32, o3c.Device("CPU:0")
                )
                self.skin[skin_part] = [np.asarray(pc.points), np.asarray(pc.normals)]
                self.skin_activations[skin_part] = np.zeros((len(pc.points),))

        if self.config.log.log:
            self.file_logger = logging.getLogger("pycub_file_logger")
            self.file_logger.setLevel(logging.INFO)
            for h in list(self.file_logger.handlers):
                self.file_logger.removeHandler(h)
                try:
                    h.close()
                except Exception:
                    pass
            log_dir = get_user_log_dir(self.file_dir)
            ts = str(datetime.datetime.now()).replace(".", "-").replace(" ", "-").replace(":", "-")
            file_handler = logging.FileHandler(str(log_dir / f"{ts}.csv"), mode="a", encoding="utf-8")
            file_handler.setFormatter(logging.Formatter('%(message)s'))
            self.file_logger.addHandler(file_handler)
            initial_string = "timestamp;steps_done;" + ";".join([_.name for _ in self.joints])

            if self.config.skin.use:
                for skin_part in self.skin_activations.keys():
                    initial_string += ";" + skin_part
            self.file_logger.info(initial_string)

        if self.gui:
            if _VISUALIZER_FACTORY is not None:
                self.visualizer = _VISUALIZER_FACTORY(self)
            else:
                try:
                    from icub_pybullet.visualizer import Visualizer
                except Exception:
                    from visualizer import Visualizer
                self.visualizer = Visualizer(self)
            self.last_render = time.time()

        self._rtb_robot = None

        self.chains, self.chains_joints = self.get_chains()

        self.collision_during_motion = False
        self.steps_done = 0
        self.toggle_gravity()

        self.precomputed_link_ids = {}
        for link in self.links:
            self.precomputed_link_ids[link.name] = link.robot_link_id

    @property
    def rtb_robot(self):
        """Lazily constructed Robotics Toolbox ``Robot`` model used for Jacobian queries."""
        if self._rtb_robot is None:
            self._rtb_robot = self._build_rtb_robot()
        return self._rtb_robot

    @rtb_robot.setter
    def rtb_robot(self, value: Any) -> None:
        """Override the cached Robotics Toolbox robot instance."""
        self._rtb_robot = value

    def _build_rtb_robot(self) -> Any:
        """Parse the active robot URDF into a Robotics Toolbox ``Robot`` instance."""
        import roboticstoolbox as rtb
        try:
            from roboticstoolbox.models.URDF.URDFRobot import URDFRobot, _links_from_urdf
            from roboticstoolbox.tools.urdf import URDF as RTB_URDF
            has_urdf_robot = True
        except Exception:
            try:
                from roboticstoolbox.models.URDF.URDFRobot import URDFRobot
                has_urdf_robot = True
            except Exception:
                has_urdf_robot = False
            _links_from_urdf = None
            RTB_URDF = None

        if _links_from_urdf is not None and RTB_URDF is not None:
            try:
                with open(self.urdf_path, "r", encoding="utf-8") as f:
                    urdf_str = f.read()
                urdf_obj = RTB_URDF.loadstr(urdf_str, None)
                rtb_links, rtb_name = _links_from_urdf(urdf_obj)
                robot = rtb.robot.Robot(
                    rtb_links,
                    name=rtb_name.upper(),
                    manufacturer="IIT",
                )
                robot._urdf_filepath = self.urdf_path
                return robot
            except Exception:
                pass
        if has_urdf_robot:
            try:
                robot = URDFRobot(self.urdf_path, manufacturer="IIT")
                robot.name = robot.name.upper()
                return robot
            except Exception:
                pass
        rtb_links, rtb_name, rtb_urdf_string, rtb_urdf_file_path = rtb.robot.Robot.URDF_read(self.urdf_path)
        return rtb.robot.Robot(
            rtb_links,
            name=rtb_name.upper(),
            manufacturer="IIT",
            urdf_string=rtb_urdf_string,
            urdf_filepath=rtb_urdf_file_path,
        )

    @staticmethod
    def get_chains() -> Tuple[dict, dict]:
        """Return link and joint definitions for each kinematic chain on iCub."""
        chains = {
            "left_arm": ['chest', 'l_shoulder_1', 'l_shoulder_2', 'l_shoulder_3',
                         'l_upper_arm', 'l_elbow_1', 'l_forearm', 'l_wrist_1', 'l_hand'],
            "right_arm": ['chest', 'r_shoulder_1', 'r_shoulder_2', 'r_shoulder_3',
                          'r_upper_arm', 'r_elbow_1', 'r_forearm', 'r_wrist_1', 'r_hand'],
            "left_leg": ['root_link', 'l_hip_1', 'l_hip_2', 'l_hip_3', 'l_upper_leg', 'l_lower_leg',
                         'l_ankle_1', 'l_ankle_2', 'l_foot', 'l_foot_dh_frame'],
            "right_leg": ['root_link', 'r_hip_1', 'r_hip_2', 'r_hip_3', 'r_upper_leg', 'r_lower_leg',
                          'r_ankle_1', 'r_ankle_2', 'r_foot', 'r_foot_dh_frame'],
            "head": ['chest', 'neck_1', 'neck_2', 'head'],
            "torso": ['root_link', 'torso_1', 'torso_2', 'chest'],
        }
        chains_joints = {
            "left_arm": np.array(['l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_arm_ft_sensor',
                                  'l_elbow', 'l_wrist_prosup', 'l_wrist_pitch', 'l_wrist_yaw']),
            "right_arm": np.array(['r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_arm_ft_sensor',
                                   'r_elbow', 'r_wrist_prosup', 'r_wrist_pitch', 'r_wrist_yaw']),
            "right_leg": np.array(['r_hip_pitch', 'r_hip_roll', 'r_leg_ft_sensor', 'r_hip_yaw', 'r_knee',
                                   'r_ankle_pitch', 'r_ankle_roll', 'r_foot_ft_sensor', 'r_foot_dh_frame_fixed_joint']),
            "left_leg": np.array(['l_hip_pitch', 'l_hip_roll', 'l_leg_ft_sensor', 'l_hip_yaw', 'l_knee',
                                  'l_ankle_pitch', 'l_ankle_roll', 'l_foot_ft_sensor', 'l_foot_dh_frame_fixed_joint']),
            "head": np.array(['neck_pitch', 'neck_roll', 'neck_yaw']),
            "torso": np.array(['torso_pitch', 'torso_roll', 'torso_yaw']),
        }
        return chains, chains_joints

    def compute_jacobian(self, chain: CHAINS, start: Optional[str] = None, end: Optional[str] = None) -> Tuple[np.array, np.array]:
        """Compute the base-frame geometric Jacobian and active joint names for a kinematic chain.

        :param chain: Kinematic chain identifier (e.g. ``"right_arm"``).
        :param start: Optional base link name; defaults to the first link in ``chain``.
        :param end: Optional tip link name; defaults to the last link in ``chain``.
        """
        if start is None:
            start = self.chains[chain][0]
        if end is None:
            end = self.chains[chain][-1]
            end_id = 0
        else:
            end_id = self.chains[chain].index(end)

        q = self.get_joint_state(self.chains_joints[chain], allow_error=True)[:end_id]
        return self.rtb_robot.jacob0(q, end, start), self.chains_joints[chain][:end_id][np.array(q) != 0]

    def get_camera_images(self, eyes: Optional[str | List[str]] = None) -> dict:
        """Return the latest RGB arrays keyed by eye name. Omit eyes for all open eye views."""
        if eyes is None:
            eyes = self.visualizer.eye_windows.keys()
        if isinstance(eyes, str):
            eyes = [eyes]

        images = {}
        for eye in eyes:
            ew = self.visualizer.eye_windows[eye]
            images[ew.eye] = np.asarray(ew.last_image)
        return images

    def get_camera_depth_images(self, eyes: Optional[str | List[str]] = None) -> dict:
        """Return the latest depth arrays keyed by eye name. Omit eyes for all open eye views."""
        if eyes is None:
            eyes = self.visualizer.eye_windows.keys()
        if isinstance(eyes, str):
            eyes = [eyes]

        images = {}
        for eye in eyes:
            ew = self.visualizer.eye_windows[eye]
            images[ew.eye] = np.asarray(ew.last_depth_image)
        return images

    def find_processes_by_name(self) -> List[int]:
        """Return the current process ID (legacy helper preserved for compatibility)."""
        return [os.getpid()]

    def kill_open3d(self) -> None:
        """
        Cleanly terminate only the current process if Open3D hangs on exit, never parent/sibling processes.
        """
        try:
            psutil.Process(os.getpid()).kill()
        except Exception:
            pass

    def init_robot(self) -> Tuple[int, List[Joint], List[Link]]:
        """Load the iCub URDF into PyBullet and build ``Joint`` and ``Link`` lists."""
        self.config.self_collisions = False
        if self.config.self_collisions:
            robot = self.loadURDF(self.urdf_path, useFixedBase=True, flags=self.URDF_USE_SELF_COLLISION)
        else:
            robot = self.loadURDF(self.urdf_path, useFixedBase=True)

        joints = []
        for joint in range(self.getNumJoints(robot)):
            info = self.getJointInfo(robot, joint)
            if info[self.jointInfo["TYPE"]] != p.JOINT_FIXED:
                joint = Joint(
                    str(info[self.jointInfo["NAME"]], "utf-8"),
                    info[self.jointInfo["INDEX"]],
                    len(joints),
                    info[self.jointInfo["LOWERLIMIT"]],
                    info[self.jointInfo["UPPERLIMIT"]],
                    info[self.jointInfo["MAXFORCE"]],
                    info[self.jointInfo["MAXVELOCITY"]],
                )
                joints.append(joint)
                if hasattr(self.config.initial_joint_angles, joint.name):
                    self.resetJointState(
                        robot,
                        joint.robot_joint_id,
                        np.deg2rad(getattr(self.config.initial_joint_angles, joint.name)),
                    )

        links = []
        for link in self.urdfs["robot"].links:
            if hasattr(link, "collision"):
                link_id = self.find_link_id(os.path.basename(link.collision.geometry.mesh.filename), robot=robot)
                link = Link(link.name, link_id, link)
                links.append(link)

        self.init_urdfs()

        self.stepSimulation()
        self_collisions = self.getContactPoints(robot, robot)
        for c in self_collisions:
            self.setCollisionFilterPair(robot, robot, c[3], c[4], False)

        return robot, joints, links

    def init_urdfs(self) -> None:
        """
        Load URDFs / .obj files of scene objects (supports both built-in meshes and user custom meshes).
        """
        resolved_urdf_paths: List[str] = []
        if hasattr(self.config, "urdfs"):
            for obj_id, urdf, fixed, color in zip(
                np.arange(len(self.config.urdfs.paths)),
                self.config.urdfs.paths,
                self.config.urdfs.fixed,
                self.config.urdfs.color,
            ):
                suffix = ""
                resolved_source = resolve_mesh_or_urdf_path(urdf, self.file_dir, self.config_dir)
                ext = os.path.splitext(resolved_source)[1].lower()
                if ext == ".obj":
                    base_stem = Path(resolved_source).stem
                    obj_name = base_stem
                    while obj_name in self.urdfs:
                        suffix += "_"
                        obj_name = base_stem + suffix
                    urdf_file = self.create_urdf(resolved_source, fixed, color, suffix, obj_name=obj_name)
                elif ext == ".urdf":
                    obj_name = Path(resolved_source).stem
                    while obj_name in self.urdfs:
                        suffix += "_"
                        obj_name = Path(resolved_source).stem + suffix
                    urdf_file = resolved_source
                else:
                    raise ValueError(f"Objects must be .obj or .urdf! Got: {urdf}")

                self.urdfs[obj_name] = URDF(urdf_file, package_dir=self.file_dir)
                resolved_urdf_paths.append((obj_name, self.urdfs[obj_name].path))

        if self.config.vhacd.use_vhacd:
            self.run_vhacd(robot=False)

        if hasattr(self.config, "urdfs"):
            for urdf_id, (obj_name, _), pos, color, fixed in zip(
                range(len(resolved_urdf_paths)),
                resolved_urdf_paths,
                self.config.urdfs.positions,
                self.config.urdfs.color,
                self.config.urdfs.fixed,
            ):
                urdf_file = self.urdfs[obj_name].path
                self.other_objects.append((
                    self.loadURDF(urdf_file, pos),
                    obj_name,
                    fixed,
                    color,
                    f"{obj_name}.obj",
                ))
                if not fixed:
                    self.free_objects.append(self.other_objects[-1][0])
                f = 0.25 if not hasattr(self.config.urdfs, "force") else self.config.urdfs.force[urdf_id]
                for joint in range(self.getNumJoints(self.other_objects[-1][0])):
                    self.setJointMotorControl2(
                        self.other_objects[-1][0],
                        joint,
                        self.POSITION_CONTROL,
                        targetPosition=0,
                        targetVelocity=0,
                        force=f,
                    )

    def is_alive(self) -> bool:
        """Return True while the PyBullet client (and GUI, if enabled) remain active."""
        if self.gui and not self.visualizer.is_alive:
            return False
        return True if self._client >= 0 else False

    def update_simulation(self, sleep_duration: Optional[float | None] = -1) -> None:
        """Advance physics when the interval in seconds has elapsed and refresh the visualizer.

        Use -1 for the configured step interval or None to step on every call.
        """
        if sleep_duration == -1:
            sleep_duration = self.default_step

        cur_time = time.time()
        if sleep_duration is None or cur_time - self.last_step > sleep_duration:
            self.stepSimulation()
            if self.config.skin.use:
                self.compute_skin()
            self.last_step = cur_time
            self.steps_done += 1

            if self.config.log.log and cur_time - self.last_log > self.config.log.period:
                self.file_logger.info(self.prepare_log())
                self.last_log = cur_time

            if self.log_pose:
                self.pose_logger.append(self.end_effector.get_position())

        if self.gui and cur_time - self.last_render > 0.033 and self.visualizer.is_alive:
            self.visualizer.render()
            self.last_render = cur_time

    def toggle_gravity(self) -> None:
        """Toggle simulated gravity between Earth standard (-9.81 m/s^2) and zero-G."""
        if not self.gravity:
            self.gravity = True
            self.setGravity(0, 0, -9.81)
        else:
            self.gravity = False
            self.setGravity(0, 0, 0)

    def __del__(self) -> None:
        """Ensure the PyBullet client disconnects cleanly on garbage collection."""
        if getattr(self, "_client", -1) >= 0:
            try:
                self.disconnect()
            except Exception:
                pass

    @staticmethod
    def scale_bbox(bbox: list, scale: float) -> Tuple[np.array, np.array]:
        """Scale an axis-aligned bounding box ``(min, max)`` about its center."""
        if scale <= 0:
            raise ValueError("scale must be positive")
        if not (isinstance(bbox, (list, tuple)) and len(bbox) == 2):
            raise ValueError("bbox must be a sequence of two elements: (min, max)")

        bmin = np.asarray(bbox[0], dtype=float)
        bmax = np.asarray(bbox[1], dtype=float)

        center = (bmin + bmax) * 0.5
        half = (bmax - bmin) * 0.5
        new_half = half * scale

        return center - new_half, center + new_half

    @staticmethod
    def bbox_overlap(b1_min: list, b1_max: list, b2_min: list, b2_max: list) -> bool:
        """Return True when two axis-aligned 3D bounding boxes overlap."""
        for min1, max1, min2, max2 in zip(b1_min, b1_max, b2_min, b2_max):
            if min1 >= max2 or min2 >= max1:
                return False
        return True

    def compute_skin(self) -> None:
        """Update simulated tactile skin taxel activations from current PyBullet contacts."""
        temp = []
        points = None
        normals = None
        links_to_test = [
            "l_hand", "r_hand", "l_forearm", "r_forearm", "l_upper_arm", "r_upper_arm", "chest",
            "l_upper_leg", "r_upper_leg", "l_lower_leg", "r_lower_leg", "l_foot", "r_foot", "head",
        ]

        bboxes = []
        for l in links_to_test:
            bboxes.append(self.scale_bbox(self.getAABB(self.robot, self.precomputed_link_ids[l]), 0.8))

        for fo_id, fo in enumerate(self.other_objects):
            bboxes.append(self.scale_bbox(self.getAABB(fo[0]), 0.8))
            links_to_test.append("other_object_" + str(fo_id))

        allowed_collisions = {
            "r_hand": ["r_hand", "r_forearm"],
            "r_upper_leg": ["r_lower_leg", "r_upper_leg", "r_foot"],
            "l_forearm": ["l_hand", "l_forearm", "l_upper_arm"],
            "l_upper_leg": ["l_lower_leg", "l_upper_leg", "l_foot"],
            "chest": ["l_upper_arm", "r_upper_arm", "chest", "head"],
            "r_upper_arm": ["r_upper_arm", "r_forearm", "chest", "head"],
            "r_foot": ["r_foot", "r_upper_leg", "r_lower_leg"],
            "l_foot": ["l_foot", "l_upper_leg", "l_lower_leg"],
            "l_upper_arm": ["l_upper_arm", "l_forearm", "chest", "head"],
            "r_lower_leg": ["r_lower_leg", "r_upper_leg", "r_foot"],
            "l_lower_leg": ["l_lower_leg", "l_upper_leg", "l_foot"],
            "r_forearm": ["r_hand", "r_forearm", "r_upper_arm"],
            "l_hand": ["l_hand", "l_forearm"],
            "head": ["head", "chest", "l_upper_arm", "r_upper_arm"],
        }

        R = np.eye(4)
        R_ori = R[:3, :3]

        idxs = self.precomputed_link_ids.values()
        names = list(self.precomputed_link_ids.keys())

        link_states = self.getLinkStates(self.robot, idxs, computeLinkVelocity=0, computeForwardKinematics=0)

        for skin_part, pc in self.skin.items():
            use_skin = False
            self.skin_activations[skin_part].fill(0)
            self.activated_skin_points[skin_part] = []
            self.activated_skin_normals[skin_part] = []

            robot_link_id = self.precomputed_link_ids[skin_part]
            linkState = link_states[names.index(skin_part)]
            ori = linkState[self.linkInfo["URDFORI"]]
            pos = linkState[self.linkInfo["URDFPOS"]]

            R_ori.flat[:] = self.getMatrixFromQuaternion(ori)
            R[:3, 3] = pos

            points_ = (R[:3, :3] @ pc[0].T).T + R[:3, 3]
            normals_ = (R[:3, :3] @ pc[1].T).T

            bbox = (np.min(points_, axis=0), np.max(points_, axis=0))
            bbox_min, bbox_max = self.scale_bbox(bbox, 1)

            for bb_i, bb in enumerate(bboxes):
                if self.bbox_overlap(bb[0], bb[1], bbox_min, bbox_max) and links_to_test[bb_i] not in allowed_collisions[skin_part]:
                    use_skin = True
                    break

            if not use_skin:
                continue

            if points is None:
                points = points_
                normals = normals_
            else:
                points = np.vstack((points, points_))
                normals = np.vstack((normals, normals_))

            temp.append((robot_link_id, skin_part, points_.shape[0]))

        if points is not None:
            contacts = self.rayTestBatch(
                points,
                points + self.config.skin.radius * normals,
                numThreads=self.config.skin.num_cores,
            )
            start_id = 0
            for link_id, skin_part, num_points in temp:
                for c_id, c in enumerate(contacts[start_id:start_id + num_points]):
                    if c[0] == -1:
                        continue
                    if c[1] == link_id:
                        continue
                    self.skin_activations[skin_part][c_id] = 1 - c[2]
                    self.activated_skin_points[skin_part].append(points[start_id + c_id])
                    self.activated_skin_normals[skin_part].append(normals[start_id + c_id])
                start_id += num_points

    def prepare_log(self) -> str:
        """Serialize current timestamp, step count, joint angles, and skin state into a CSV row."""
        states = self.getJointStates(self.robot, [_.robot_joint_id for _ in self.joints])
        joint_states = ";".join([str(_[0]) for _ in states])

        s = f"{self.last_step};{self.steps_done};{joint_states}"

        if self.config.skin.use:
            for skin_part, activations in self.skin_activations.items():
                s += ";" + ",".join([str(_) for _ in activations])
        return s

    def move_position(self, joints: JOINTS | List[JOINTS] | JOINTS_IDS | List[JOINTS_IDS],
                      positions: float | List[float], wait: Optional[bool] = True, velocity: Optional[float] = 1,
                      set_col_state: Optional[bool] = True, check_collision: Optional[bool] = True,
                      timeout: Optional[float] = None) -> None:
        """Move one joint or a list of joints to target angles in radians.

        :param joints: Joint name or ID, or a list of joints to move together.
        :param positions: Target angle in radians for each joint, in the same order.
        :param wait: Step the simulation until the movement finishes.
        :param velocity: Maximum joint speed in radians per second.
        :param set_col_state: Clear the collision flag at the start of this move.
        :param check_collision: Check for collisions while waiting for the move.
        :param timeout: Maximum motion time in seconds, or None for no time limit.
        """
        if isinstance(joints, int) or isinstance(joints, str):
            positions = [positions]
            joints = [joints]

        s_time = time.time()
        for joint, position in zip(joints, positions):
            robot_joint_id, joint_id = self.find_joint_id(joint)
            if not (self.joints[joint_id].lower_limit <= position <= self.joints[joint_id].upper_limit):
                self.logger.warning(
                    f"Joint {joint} cannot be moved to {position} as it is out of bounds "
                    f"({self.joints[joint_id].lower_limit}, {self.joints[joint_id].upper_limit})."
                )
                position = np.clip(position, self.joints[joint_id].lower_limit, self.joints[joint_id].upper_limit)
            self.joints[joint_id].set_point = position
            self.joints[joint_id].start_time = s_time
            self.joints[joint_id].timeout = timeout
            self.setJointMotorControl2(
                self.robot,
                robot_joint_id,
                controlMode=self.POSITION_CONTROL,
                targetPosition=position,
                force=self.joints[joint_id].max_force,
                maxVelocity=velocity,
            )
        if set_col_state:
            self.collision_during_motion = False
        if wait:
            self.wait_motion_done(check_collision=check_collision)

    def move_velocity(self, joints: JOINTS | List[JOINTS] | JOINTS_IDS | List[JOINTS_IDS],
                      velocities: float | List[float]) -> None:
        """Command one or more joint velocities in radians per second, in joint order."""
        if isinstance(joints, int) or isinstance(joints, str):
            velocities = [velocities]
            joints = [joints]

        for joint, velocity in zip(joints, velocities):
            robot_joint_id, joint_id = self.find_joint_id(joint)
            if np.abs(velocity) > self.joints[joint_id].max_velocity:
                self.logger.warning(
                    f"Joint {joint} cannot be moved with velocity {velocity} as it is over the max velocity "
                    f"{self.joints[joint_id].max_velocity}"
                )
            self.setJointMotorControl2(
                self.robot,
                robot_joint_id,
                controlMode=self.VELOCITY_CONTROL,
                targetVelocity=velocity,
                force=1 if velocity != 0 else 100,
                maxVelocity=self.joints[joint_id].max_velocity,
            )
            self.joints[joint_id].set_point = "vel"

    def get_joint_state(self, joints: Optional[JOINTS | List[JOINTS] | JOINTS_IDS | List[JOINTS_IDS]] = None,
                        allow_error: Optional[bool] = False) -> List[list]:
        """Read positions in radians in the requested order, defaulting to all joints."""
        if joints is None:
            joints = [joint.name for joint in self.joints]
        elif isinstance(joints, int) or isinstance(joints, str):
            joints = [joints]

        states = []
        for joint in joints:
            try:
                robot_joint_id, joint_id = self.find_joint_id(joint)
            except Exception as e:
                if allow_error:
                    states.append(0)
                    continue
                else:
                    raise e

            states.append(self.getJointState(self.robot, robot_joint_id)[self.jointStates["POSITION"]])

        return states

    def motion_done(self, joints: Optional[JOINTS | List[JOINTS] | JOINTS_IDS | List[JOINTS_IDS]] = None,
                    check_collision=True) -> bool:
        """Return True when all specified joints reached their targets, timed out, or hit a collision.

        :param joints: Joint name/ID or list of joints to check; defaults to all robot joints.
        :param check_collision: Abort and return True if an unexpected contact occurs.
        """
        if joints is None:
            joints = [joint.name for joint in self.joints]
        elif not isinstance(joints, list):
            joints = [joints]
        if check_collision:
            contacts = self.getContactPoints(self.robot)
            for c in contacts:
                if c[self.contactPoints["IDB"]] not in self.free_objects and c[self.contactPoints["DISTANCE"]] < self.config.collision_tolerance:
                    self.collision_during_motion = True
                    self.stop_robot()
                    self.logger.warning("Collision detected during motion!")
                    self.print_collision_info()
                    return True
        cur_time = time.time()
        for joint in joints:
            robot_joint_id, joint_id = self.find_joint_id(joint)
            state = self.getJointState(self.robot, robot_joint_id)
            if self.joints[joint_id].set_point is not None and self.joints[joint_id].set_point != "vel":
                if self.joints[joint_id].timeout is not None and cur_time - self.joints[joint_id].start_time > self.joints[joint_id].timeout:
                    self.stop_robot()
                    return True
                if np.abs(state[self.jointStates["POSITION"]] - self.joints[joint_id].set_point) > self.joint_tolerance:
                    return False

        self.stop_robot()
        return True

    def wait_motion_done(self, sleep_duration: Optional[float] = 0.01,
                         check_collision: Optional[bool] = True) -> None:
        """Block while stepping the simulation until ``motion_done`` returns True."""
        while not self.motion_done(check_collision=check_collision):
            self.update_simulation(sleep_duration)

    def stop_robot(self, joints: Optional[JOINTS | List[JOINTS] | JOINTS_IDS | List[JOINTS_IDS]] = None) -> None:
        """Halt active position or velocity commands for ``joints`` (or all joints)."""
        if joints is None:
            joints = [joint.name for joint in self.joints]
        else:
            if isinstance(joints, str) or isinstance(joints, int):
                joints = [joints]
        for joint in joints:
            robot_joint_id, joint_id = self.find_joint_id(joint)
            state = self.getJointState(self.robot, robot_joint_id)
            if self.joints[joint_id].set_point is not None:
                if self.joints[joint_id].set_point == "vel":
                    self.move_velocity(joint, 0)
                else:
                    self.move_position(joint, state[self.jointStates["POSITION"]], wait=False, set_col_state=False)
                self.joints[joint_id].set_point = None
                self.joints[joint_id].start_time = None
                self.joints[joint_id].timeout = None

    def move_cartesian(self, pose: Pose, wait: Optional[bool] = True, velocity: Optional[float] = 1,
                       check_collision: Optional[bool] = True, timeout: Optional[float] = None) -> None:
        """Move the end effector to a pose using inverse kinematics.

        Velocity limits joint speed in radians per second. With wait=False,
        call update_simulation to advance the motion.
        """
        ik_solution = np.array(self.calculateInverseKinematics(
            self.robot,
            self.end_effector.link_id,
            pose.pos,
            pose.ori,
            lowerLimits=self.IK_config["lower_limits"],
            upperLimits=self.IK_config["upper_limits"],
            jointRanges=self.IK_config["joint_ranges"],
            restPoses=self.IK_config["rest_poses"],
        ))
        self.move_position(
            self.IK_config["movable_joints"],
            ik_solution[self.IK_config["movable_joints"]],
            wait=False,
            velocity=velocity,
            timeout=timeout,
        )
        if wait:
            self.wait_motion_done(check_collision=check_collision)

    def find_joint_id(self, joint_name: JOINTS | JOINTS_IDS) -> Tuple[int, int]:
        """Resolve a joint name or index to ``(pybullet_joint_id, joints_list_index)``."""
        for joint in self.joints:
            if joint_name in [joint.name, joint.joints_id]:
                return joint.robot_joint_id, joint.joints_id
        raise ValueError(f"Unknown joint: {joint_name}")

    def find_link_id(self, mesh_name: str, robot: Optional[int] = None, urdf_name: Optional[str] = "robot") -> int:
        """Find the PyBullet link index whose collision shape uses ``mesh_name``."""
        if robot is None:
            robot = self.robot

        for link_id in range(0, len(self.urdfs[urdf_name].links) - 1):
            cs = self.getCollisionShapeData(robot, link_id)
            if len(cs) > 0:
                if mesh_name == os.path.basename(cs[0][4].decode("utf-8")):
                    return link_id

    def _resolve_urdf_mesh_path(self, mesh_uri: str, urdf_path: str) -> str:
        """Resolve a mesh filename inside a URDF (supports package://, relative, and absolute paths)."""
        if mesh_uri.startswith("package://"):
            rel = mesh_uri.replace("package://", "")
            cand = os.path.normpath(os.path.join(self.file_dir, rel))
            if os.path.exists(cand):
                return cand
            return os.path.normpath(os.path.join(os.path.dirname(urdf_path), rel))
        if os.path.isabs(mesh_uri):
            return os.path.normpath(mesh_uri)
        return os.path.normpath(os.path.join(os.path.dirname(urdf_path), mesh_uri))

    def run_vhacd(self, robot: Optional[bool] = True) -> None:
        """
        Run VHACD on loaded URDF objects using pre-baked meshes when present, or writing newly generated
        collision meshes and patched URDFs into the writable user cache directory.
        """
        for obj_name, obj in self.urdfs.items():
            if not robot and "robot" in obj_name:
                continue
            something_changed = False
            vhacd_key_parts = [obj.path, file_fingerprint(obj.path)]
            for link in obj.links:
                if hasattr(link, "collision") and hasattr(link.collision.geometry, "mesh"):
                    col_path_ori = link.collision.geometry.mesh.filename
                    col_path = self._resolve_urdf_mesh_path(col_path_ori, obj.path)
                    vhacd_path, needs_compute = get_vhacd_cache_path(
                        col_path, force_vhacd=bool(self.config.vhacd.force_vhacd)
                    )
                    if needs_compute:
                        msg = f"Generating collision mesh (VHACD) for {os.path.basename(col_path)}..."
                        if _STATUS_CALLBACK is not None:
                            _STATUS_CALLBACK("vhacd", msg)
                        self.vhacd(
                            col_path,
                            vhacd_path,
                            "",
                            resolution=1000000,
                            maxNumVerticesPerCH=1,
                            gamma=0.0005,
                            concavity=0,
                        )
                        something_changed = True
                    link.collision.geometry.mesh.filename = to_posix(vhacd_path)
                    vhacd_key_parts.append(vhacd_path)

            obj.expand_package_uris()
            base_stem = Path(obj.path).stem.replace("_fixed", "")
            target_urdf = get_cached_urdf_path(
                f"{base_stem}_vhacd",
                "|".join(vhacd_key_parts),
                subdir="generated_urdf",
            )
            if something_changed or self.config.vhacd.force_vhacd_urdf or not os.path.isfile(target_urdf):
                obj.write_urdf()
                write_text_atomic(target_urdf, obj.new_urdf)
            obj.path = target_urdf

    def create_urdf(self, object_path: str, fixed: bool, color: List[float], suffix: Optional[str] = "",
                    obj_name: Optional[str] = None) -> str:
        """
        Create a URDF for a given .obj file inside the writable user cache directory (safe on read-only installs
        and safe under concurrent multi-instance execution).
        """
        with open(os.path.join(self.file_dir, "other_meshes", "object_default.urdf"), "r", encoding="utf-8") as f:
            urdf = f.read()

        src_abs = os.path.abspath(object_path)
        fp = file_fingerprint(src_abs)
        stem = Path(src_abs).stem
        model_name = obj_name or (stem + suffix)

        if suffix != "":
            dup_obj = get_user_cache_dir() / "generated_urdf" / f"{stem}{suffix}_{fp}.obj"
            if not dup_obj.is_file():
                shutil.copyfile(src_abs, str(dup_obj))
            mesh_for_visual = str(dup_obj)
        else:
            mesh_for_visual = src_abs

        posix_visual = to_posix(mesh_for_visual)
        posix_collision = to_posix(src_abs)
        color_str = " ".join(map(str, color))
        urdf = (
            urdf.replace("OBJECTNAME", model_name)
            .replace("LATERALFRICTION", "1")
            .replace("ROLLINGFRICTION", "0")
            .replace("MASS", "0.2")
            .replace("FILENAMECOLLISION", posix_collision)
            .replace("FILENAME", posix_visual)
            .replace("VISUALCOLOR", color_str)
        )

        if fixed:
            with open(os.path.join(self.file_dir, "other_meshes", "fixed_link.txt"), "r", encoding="utf-8") as f:
                fixed_link_text = f.read()
            urdf = urdf.replace("</robot>", fixed_link_text)

        out_urdf = get_cached_urdf_path(
            model_name,
            f"{posix_visual}|{posix_collision}|{fixed}|{color_str}|{fp}",
            subdir="generated_urdf",
        )
        write_text_atomic(out_urdf, urdf)
        return out_urdf

    def print_collision_info(self, c: Optional[list] = None):
        """Log details for a single PyBullet contact record or all active robot contacts."""
        if c is None:
            contacts = self.getContactPoints(self.robot)
            for c in contacts:
                self.print_collision_info(c)
        else:
            body_b = "unknown"
            if c[self.contactPoints['IDB']] == self.robot:
                body_b = "robot"
            else:
                for obj, name, _, _, _ in self.other_objects:
                    if c[self.contactPoints['IDB']] == obj:
                        body_b = name
                        break
            link_a = "base"
            for link in self.links:
                if link.robot_link_id == c[self.contactPoints['INDEXA']]:
                    link_a = link.name
                    break
            if c[self.contactPoints['IDB']] == self.robot:
                link_b = "base"
                for link in self.links:
                    if link.robot_link_id == c[self.contactPoints['INDEXB']]:
                        link_b = link.name
                        break
            else:
                link_b = f"{body_b} link {c[self.contactPoints['INDEXB']]}"
            self.logger.info(
                f"\nCollision of robot with {body_b}\n"
                f"Collision of {link_a} with {link_b}\n"
                f"Position A: {c[self.contactPoints['POSITIONA']]}\n"
                f"Position B: {c[self.contactPoints['POSITIONB']]}\n"
                f"Normal: {c[self.contactPoints['NORMAL']]}\n"
                f"Distance: {c[self.contactPoints['DISTANCE']]}\n"
                f"Force: {c[self.contactPoints['FORCE']]}\n"
                f"Friction 1: {c[self.contactPoints['FRICTION1']]}\n"
                f"Friction dir 1: {c[self.contactPoints['FRICTIONDIR1']]}\n"
                f"Friction 2: {c[self.contactPoints['FRICTION2']]}\n"
                f"Friction dir 2: {c[self.contactPoints['FRICTIONDIR2']]}\n"
            )


class EndEffector:
    """Helper representing the configured end-effector link of the robot."""
    def __init__(self, name: str, client: pyCub):
        """Bind ``name`` to its PyBullet link ID on ``client``."""
        self.name = name
        self.client = client
        for link in self.client.urdfs["robot"].links:
            if link.name == self.name:
                self.link_id = self.client.find_link_id(os.path.basename(link.collision.geometry.mesh.filename))
                break

    def get_position(self) -> Pose:
        """Return the current world-frame ``Pose`` of the end-effector link."""
        state = self.client.getLinkState(self.client.robot, self.link_id)
        pos = list(state[self.client.linkInfo["URDFPOS"]])
        ori = list(state[self.client.linkInfo["URDFORI"]])
        return Pose(pos, ori)
