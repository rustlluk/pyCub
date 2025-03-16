import copy
import glob
import time
import numpy as np
import psutil
import pybullet as p
from pybullet_utils.bullet_client import BulletClient
import os
from icub_pybullet.visualizer import Visualizer
from icub_pybullet.utils import Config, URDF, Pose, CustomFormatter
import open3d as o3d
import logging
import datetime
import inspect
import psutil
import atexit
import roboticstoolbox as rtb


class pyCub(BulletClient):
    """
    Client class which inherits from BulletClient and contains the whole simulation functionality

    """
    # As dict, because IntEnum is about 1.5-2x slower
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

    def __init__(self, config="default.yaml"):
        """
        :param config: path to the config file
        :type config: str, optional, default="default.yaml"
        """
        super().__init__(p.DIRECT)

        self.parent_name = os.path.basename(inspect.stack()[1].filename)

        self.file_dir = os.path.dirname(os.path.abspath(__file__))

        for c_path in [os.path.join(self.file_dir, "configs", config), config,
                       os.path.join(os.getcwd(), os.path.dirname(inspect.stack()[1].filename), config)]:
            if os.path.exists(c_path):
                self.config = Config(c_path)
        self.config.simulation_step = 1/self.config.simulation_step
        self.setTimeStep(self.config.simulation_step)
        if self.config.gui.standard or self.config.gui.web:
            self.gui = True
            if self.config.gui.standard and self.config.gui.web:
                self.config.gui.web = False
        else:
            self.gui = False

        if self.gui:
            atexit.register(self.kill_open3d)

        if self.gui:
            self.default_step = 0.01
        else:
            self.default_step = None

        self.logger = logging.getLogger("pycub_logger")
        self.logger.setLevel(logging.DEBUG if self.config.debug else logging.INFO)
        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(CustomFormatter())
        self.logger.addHandler(stream_handler)

        if hasattr(self.config, "log_pose"):
            self.log_pose = self.config.log_pose
            self.pose_logger = []
        else:
            self.log_pose = False

        self.gravity = False

        self.urdf_path = os.path.join(self.file_dir, self.config.robot_urdf_path)

        self.urdfs = {"robot": URDF(self.urdf_path)}

        self.other_objects = []

        if hasattr(self.config, "urdfs"):
            for obj_id, urdf, fixed, color in zip(np.arange(len(self.config.urdfs.paths)), self.config.urdfs.paths, self.config.urdfs.fixed, self.config.urdfs.color):
                suffix = ""
                if os.path.basename(urdf).split(".")[-1] == "obj":
                    obj_name = os.path.basename(urdf).split(".")[0]
                    while obj_name in self.urdfs:
                        suffix += "_"
                        obj_name = obj_name+suffix
                    self.create_urdf(urdf, fixed, color, suffix)
                    urdf = os.path.normpath(os.path.join(self.file_dir, "..", "other_meshes", urdf.replace(".obj", suffix+".urdf")))
                elif os.path.basename(urdf).split(".")[-1] == "urdf":
                    urdf = os.path.normpath(os.path.join(self.file_dir, "..", "other_meshes", urdf))
                else:
                    raise ValueError("Objects must be .obj or .urdf!")
                self.urdfs[os.path.basename(urdf).split(".")[0]] = URDF(urdf)
                self.config.urdfs.paths[obj_id] = urdf

        if self.config.vhacd.use_vhacd:
            self.run_vhacd()
        self.free_objects = []
        if hasattr(self.config, "urdfs"):
            for urdf, pos, fixed in zip(self.config.urdfs.paths, self.config.urdfs.positions, self.config.urdfs.fixed):
                obj_name = os.path.basename(urdf).split(".")[0]
                urdf = self.urdfs[obj_name].path
                self.other_objects.append((self.loadURDF(urdf, pos), obj_name, fixed))
                if not fixed:
                    self.free_objects.append(self.other_objects[-1][0])

        self.urdf_path = self.urdfs["robot"].path
        self.robot, self.joints, self.links = self.init_robot()

        # prepare IK config so we can utilize null space
        self.IK_config = {"movable_joints": [_.joints_id for _ in self.joints if "_hand_" not in _.name],
                          "lower_limits": [_.lower_limit for _ in self.joints],
                          "upper_limits": [_.upper_limit for _ in self.joints],
                          "joint_ranges": [np.abs(_.upper_limit - _.lower_limit) for _ in self.joints],
                          "rest_poses": [0 if not hasattr(self.config.initial_joint_angles, _.name)
                                         else np.deg2rad(getattr(self.config.initial_joint_angles, _.name))
                                         for _ in self.joints]}

        self.end_effector = EndEffector(self.config.end_effector, self)

        self.last_step = time.time()
        self.last_log = time.time()
        self.joint_tolerance = float(self.config.tolerance.joint)

        self.neighbour_links = {}
        if self.config.skin.use:
            self.skin_point_clouds = {}
            self.skin = {}
            self.skin_activations = {}
            self.activated_skin_points = {}
            self.activated_skin_normals = {}
            with open(os.path.join(self.file_dir, "..", "iCub/skin/point_clouds/config.txt"), "r") as f:
                skin_config = {_.split(";")[0]: _.split(";")[1] for _ in f.read().splitlines()}
            if len(self.config.skin.skin_parts) == 0:
                skin_pcds = glob.glob(os.path.join(self.file_dir, "..", "iCub/skin/point_clouds", "*.pcd"))
            else:
                skin_pcds = [os.path.join(self.file_dir, "..", "iCub/skin/point_clouds", f"{_}.pcd")
                             for _ in self.config.skin.skin_parts]
            for pc_path in skin_pcds:
                # if "leg" not in pc_path:
                #     continue
                pc = o3d.io.read_point_cloud(pc_path)
                pc.normalize_normals()
                if "foot" not in pc_path:
                    pc.scale(1.05, pc.get_center())
                skin_part = skin_config[os.path.basename(pc_path).split(".")[0]]
                self.skin_point_clouds[skin_part] = pc
                self.skin[skin_part] = [np.asarray(pc.points), np.asarray(pc.normals)]
                self.skin_activations[skin_part] = np.zeros((len(pc.points), ))

        if self.config.log.log:
            self.file_logger = logging.getLogger("pycub_file_logger")
            self.file_logger.setLevel(logging.INFO)
            file_handler = logging.FileHandler(os.path.join(self.file_dir, "logs",
                                                            str(datetime.datetime.now()).replace(".", "-").replace(" ", "-")
                                                            .replace(":", "-")+".csv"),
                                               mode="a")
            file_handler.setFormatter(logging.Formatter('%(message)s'))
            self.file_logger.addHandler(file_handler)
            initial_string = "timestamp;steps_done;"+";".join([_.name for _ in self.joints])

            if self.config.skin.use:
                for skin_part in self.skin_activations.keys():
                    initial_string += ";"+skin_part
            self.file_logger.info(initial_string)

        if self.gui:
            self.visualizer = Visualizer(self)
            self.last_render = time.time()

        rtb_links, rtb_name, rtb_urdf_string, rtb_urdf_file_path = rtb.robot.Robot.URDF_read(self.urdf_path)
        self.rtb_robot = rtb.robot.Robot(rtb_links, name=rtb_name.upper(), manufacturer="IIT",
                                         urdf_string=rtb_urdf_string, urdf_filepath=rtb_urdf_file_path,)
        #
        chains, chains_joints = self.get_all_chains(self.urdfs["robot"].joints[0], [], [], [], [])

        self.chains, self.chains_joints = self.get_chains()

        self.collision_during_motion = False
        self.steps_done = 0
        self.toggle_gravity()

    def get_chains(self):
        chains = {"left_arm": ['chest', 'l_shoulder_1', 'l_shoulder_2', 'l_shoulder_3',
                                    'l_upper_arm', 'l_elbow_1', 'l_forearm', 'l_wrist_1', 'l_hand'],
                       "right_arm": ['chest', 'r_shoulder_1', 'r_shoulder_2', 'r_shoulder_3',
                                    'r_upper_arm', 'r_elbow_1', 'r_forearm', 'r_wrist_1', 'r_hand'],
                       "left_leg": ['root_link', 'l_hip_1', 'l_hip_2', 'l_hip_3', 'l_upper_leg', 'l_lower_leg',
                                    'l_ankle_1', 'l_ankle_2', 'l_foot', 'l_foot_dh_frame'],
                       "right_leg": ['root_link', 'r_hip_1', 'r_hip_2', 'r_hip_3', 'r_upper_leg', 'r_lower_leg',
                                     'r_ankle_1', 'r_ankle_2', 'r_foot', 'r_foot_dh_frame'],
                       "head": ['chest', 'neck_1', 'neck_2', 'head'],
                       "torso": ['root_link', 'torso_1', 'torso_2', 'chest']}
        chains_joints = {"left_arm": np.array(['l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_arm_ft_sensor',
                                      'l_elbow', 'l_wrist_prosup', 'l_wrist_pitch', 'l_wrist_yaw']),
                         "right_arm": np.array(['r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_arm_ft_sensor',
                                       'r_elbow', 'r_wrist_prosup', 'r_wrist_pitch', 'r_wrist_yaw']),
                         "right_leg": np.array(['r_hip_pitch', 'r_hip_roll', 'r_leg_ft_sensor', 'r_hip_yaw', 'r_knee',
                                      'r_ankle_pitch', 'r_ankle_roll', 'r_foot_ft_sensor', 'r_foot_dh_frame_fixed_joint']),
                         "left_leg": np.array(['l_hip_pitch', 'l_hip_roll', 'l_leg_ft_sensor', 'l_hip_yaw', 'l_knee',
                                       'l_ankle_pitch', 'l_ankle_roll', 'l_foot_ft_sensor', 'l_foot_dh_frame_fixed_joint']),
                         "head": np.array(['neck_pitch', 'neck_roll', 'neck_yaw']),
                         "torso": np.array(['torso_pitch', 'torso_roll', 'torso_yaw'])}
        return chains, chains_joints

    def get_all_chains(self, joint, chain, chains, chain_joint, chains_joints):
        while hasattr(joint, "child"):
            chain.append(joint.child.name)
            chain_joint.append(joint.name)
            if hasattr(joint.child, "joint"):
                joints = joint.child.joint
                if len(joints) == 1:
                    joint = joints[0]
                else:
                    for joint in joints:
                        self.get_all_chains(joint, copy.deepcopy(chain), chains, copy.deepcopy(chain_joint), chains_joints)
                    break
            else:
                chains.append(chain)
                chains_joints.append(chain_joint)
                break
        return chains, chains_joints

    def compute_jacobian(self, chain, start=None, end=None):
        if start is None:
            start = self.chains[chain][0]
        if end is None:
            end = self.chains[chain][-1]
            end_id = 0
        else:
            end_id = self.chains[chain].index(end)

        q = self.get_joint_state(self.chains_joints[chain], allow_error=True)[:end_id]
        return self.rtb_robot.jacob0(q, end, start), self.chains_joints[chain][:end_id][np.array(q) != 0]

    def get_camera_images(self):
        """
        Gets the images from enabled eye cameras

        :return: list of numpy arrays
        :rtype: list
        """
        images = []
        for ew in self.visualizer.eye_windows.values():
            images.append(np.asarray(ew.last_image))
        return images

    def find_processes_by_name(self):
        matching_pids = []
        for process in psutil.process_iter(attrs=['pid', 'name', 'cmdline']):
            try:
                if self.parent_name in ' '.join(process.info['cmdline']):
                    matching_pids.append(process.info['pid'])
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return matching_pids

    def kill_open3d(self):
        # a bit of a workaround to kill open3d, that seems to hang for some
        for _ in self.find_processes_by_name():
            psutil.Process(_).kill()


    def init_robot(self):
        """
        Load the robot URDF and get its joints' information

        :return: robot and its joints
        :rtype: int or list
        """
        if self.config.self_collisions:
            robot = self.loadURDF(self.urdf_path, useFixedBase=True, flags=self.URDF_USE_SELF_COLLISION)
        else:
            robot = self.loadURDF(self.urdf_path, useFixedBase=True)

        joints = []
        for joint in range(self.getNumJoints(robot)):
            info = self.getJointInfo(robot, joint)
            if info[self.jointInfo["TYPE"]] != p.JOINT_FIXED:
                joint = Joint(str(info[self.jointInfo["NAME"]], "utf-8"), info[self.jointInfo["INDEX"]], len(joints),
                              info[self.jointInfo["LOWERLIMIT"]], info[self.jointInfo["UPPERLIMIT"]],
                              info[self.jointInfo["MAXFORCE"]], info[self.jointInfo["MAXVELOCITY"]])
                joints.append(joint)
                if hasattr(self.config.initial_joint_angles, joint.name):
                    self.resetJointState(robot, joint.robot_joint_id, np.deg2rad(getattr(self.config.initial_joint_angles, joint.name)))

        links = []
        for link in self.urdfs["robot"].links:
            if hasattr(link, "collision"):
                link_id = self.find_link_id(os.path.basename(link.collision.geometry.mesh.filename), robot=robot)
                link = Link(link.name, link_id, link)
                links.append(link)

        # perform one step of collision detection
        self.stepSimulation()
        # get all collisions
        self_collisions = self.getContactPoints(robot, robot)
        # disable collision for all links in collision -> these links should be in collision by default, so we need
        # to disable checks for them
        for c in self_collisions:
            self.setCollisionFilterPair(robot, robot, c[3], c[4], False)

        return robot, joints, links

    def is_alive(self):
        """
        Checks whether the engine is still running

        :return: True when running
        :rtype: bool
        """
        if self.gui and not self.visualizer.is_alive:
            return False
        return True if self._client >= 0 else False

    def update_simulation(self, sleep_duration=-1):
        """
        Updates the simulation

        :param sleep_duration: duration to sleep before the next simulation step
        :type sleep_duration: float, optional, default=-1
        """

        if sleep_duration == -1:
            sleep_duration = self.default_step

        # This is here to keep events and everything in open3D work even if we want slower simulation
        cur_time = time.time()
        if sleep_duration is None or cur_time-self.last_step > sleep_duration:
            self.stepSimulation()
            if self.config.skin.use:
                self.compute_skin()
            self.last_step = cur_time
            self.steps_done += 1

            if self.config.log.log and cur_time-self.last_log > self.config.log.period:
                self.file_logger.info(self.prepare_log())
                self.last_log = cur_time

            if self.log_pose:
                self.pose_logger.append(self.end_effector.get_position())

        if self.gui and cur_time-self.last_render > 0.01 and self.visualizer.is_alive:
            self.visualizer.render()
            self.last_render = cur_time
    
    def toggle_gravity(self):
        """
        Toggles the gravity

        """
        if not self.gravity:
            self.gravity = True
            self.setGravity(0, 0, -9.81)
        else:
            self.gravity = False
            self.setGravity(0, 0, 0)

    def __del__(self):
        """
        Destructor to make sure the engine is closed

        """
        self.disconnect()

    @staticmethod
    def scale_bbox(bbox, scale):
        com = (np.array(bbox[0]) + bbox[1]) / 2
        vec = np.array(bbox[0]) - bbox[1]
        norm = np.linalg.norm(vec)
        vec = vec / norm
        new_norm = scale * norm
        bbox_min = com + new_norm / 2 * vec
        bbox_max = com - new_norm / 2 * vec
        return (bbox_min, bbox_max)

    @staticmethod
    def bbox_overlap(b1_min, b1_max, b2_min, b2_max):
        for min1, max1, min2, max2 in zip(b1_min, b1_max, b2_min, b2_max):
            if min1 >= max2:
                return False
            if min2 >= max1:
                return False
        return True

    def compute_skin(self):
        """
        Function to emulate skin activations using ray casting.

        """
        temp = []
        points = None
        normals = None
        links_to_test = ["l_hand", "r_hand", "l_forearm", "r_forearm", "l_upper_arm", "r_upper_arm", "chest",
                         "l_upper_leg", "r_upper_leg", "l_lower_leg", "r_lower_leg", "l_foot", "r_foot", "head"]
        bboxes = []
        for l in links_to_test:
            for ll in self.links:
                if l == ll.name:
                    bboxes.append(self.scale_bbox(self.getAABB(self.robot, ll.robot_link_id), 0.8))
                    break

        for fo_id, fo in enumerate(self.free_objects):
            bboxes.append(self.scale_bbox(self.getAABB(fo), 0.8))
            links_to_test.append("free_object_"+str(fo_id))

        allowed_collisions = {"r_hand": ["r_hand", "r_forearm"], "r_upper_leg": ["r_lower_leg", "r_upper_leg", "r_foot"],
                              "l_forearm": ["l_hand", "l_forearm", "l_upper_arm"],
                              "l_upper_leg": ["l_lower_leg", "l_upper_leg", "l_foot"],
                              "chest": ["l_upper_arm", "r_upper_arm", "chest", "head"],
                              "r_upper_arm": ["r_upper_arm", "r_forearm", "chest", "head"], "r_foot": ["r_foot", "r_upper_leg", "r_lower_leg"],
                              "l_foot": ["l_foot", "l_upper_leg", "l_lower_leg"], "l_upper_arm": ["l_upper_arm", "l_forearm", "chest", "head"],
                              "r_lower_leg": ["r_lower_leg", "r_upper_leg", "r_foot"], "l_lower_leg": ["l_lower_leg", "l_upper_leg", "l_foot"],
                              "r_forearm": ["r_hand", "r_forearm", "r_upper_arm"], "l_hand": ["l_hand", "l_forearm"],
                              "head": ["head", "chest", "l_upper_arm", "r_upper_arm"]}
        for skin_part, pc in self.skin.items():
            use_skin = False
            self.skin_activations[skin_part].fill(0)
            self.activated_skin_points[skin_part] = []
            self.activated_skin_normals[skin_part] = []

            for link in self.links:
                if link.name == skin_part:
                    break

            linkState = self.getLinkState(self.robot, link.robot_link_id,
                                          computeLinkVelocity=0, computeForwardKinematics=0)
            ori = linkState[self.linkInfo["URDFORI"]]
            pos = linkState[self.linkInfo["URDFPOS"]]

            R = np.eye(4)
            R[:3, :3] = np.reshape(self.getMatrixFromQuaternion(ori), (3, 3))
            R[:3, 3] = pos

            points_ = (R @ np.hstack((pc[0], np.ones((len(pc[0]), 1)))).T)[:3, :].T
            normals_ = (R @ np.hstack((pc[1], np.ones((len(pc[0]), 1)))).T)[:3, :].T

            bbox = (np.min(points_, axis=0),  np.max(points_, axis=0))
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

            temp.append((link.robot_link_id, skin_part, points_.shape[0]))
        if points is not None:
            contacts = self.rayTestBatch(points, points + self.config.skin.radius*normals,
                                         numThreads=self.config.skin.num_cores)
            start_id = 0
            for link_id, skin_part, num_points in temp:
                for c_id, c in enumerate(contacts[start_id:start_id+num_points]):
                    # TODO: Fix this somehow more elegant. Some meshes are not smooth and skin collides with it
                    if c[1] == link_id:
                        continue
                    self.skin_activations[skin_part][c_id] = 1 - c[2]
                    self.activated_skin_points[skin_part].append(points[start_id+c_id])
                    self.activated_skin_normals[skin_part].append(normals[start_id+c_id])
                start_id += num_points

    def prepare_log(self):
        """
        Prepares the log string

        :return: log string
        :rtype: str
        """

        states = self.getJointStates(self.robot, [_.robot_joint_id for _ in self.joints])
        joint_states = ";".join([str(_[0]) for _ in states])

        s = f"{self.last_step};{self.steps_done};{joint_states}"

        if self.config.skin.use:
            for skin_part, activations in self.skin_activations.items():
                s += ";" + ",".join([str(_) for _ in activations])
        return s

    def move_position(self, joints, positions, wait=True, velocity=1, set_col_state=True, check_collision=True):
        """
        Move the specified joints to the given positions

        :param joints: joint or list of joints to move
        :type joints: int, list, str
        :param positions: position or list of positions to move the joints to
        :type positions: float or list
        :param wait: whether to wait until the motion is done
        :type wait: bool, optional, default=True
        :param velocity: velocity to move the joints with
        :type velocity: float, optional, default=1
        :param set_col_state: whether to reset collision state
        :type set_col_state: bool, optional, default=True
        :param check_collision: whether to check for collision during motion
        :type check_collision: bool, optional, default=True
        """
        if isinstance(joints, int) or isinstance(joints, str):
            positions = [positions]
            joints = [joints]

        for joint, position in zip(joints, positions):

            robot_joint_id, joint_id = self.find_joint_id(joint)
            if not (self.joints[joint_id].lower_limit <= position <= self.joints[joint_id].upper_limit):
                self.logger.warning(f"Joint {joint} cannot be moved to {position} as it is out of bounds "
                                    f"({self.joints[joint_id].lower_limit}, {self.joints[joint_id].upper_limit}).")
                continue
            self.joints[joint_id].set_point = position
            self.setJointMotorControl2(self.robot, robot_joint_id,
                                       controlMode=self.POSITION_CONTROL, targetPosition=position,
                                       force=self.joints[joint_id].max_force,
                                       maxVelocity=velocity)
        if set_col_state:
            self.collision_during_motion = False
        if wait:
            self.wait_motion_done(check_collision=check_collision)

    def move_velocity(self, joints, velocities):
        """
        Move the specified joints with the specified velocity

        :param joints: joint or list of joints to move
        :type joints: int or list
        :param velocities: velocity or list of velocities to move the joints to
        :type velocities: float or list
        """
        if isinstance(joints, int) or isinstance(joints, str):
            velocities = [velocities]
            joints = [joints]

        for joint, velocity in zip(joints, velocities):

            robot_joint_id, joint_id = self.find_joint_id(joint)
            if np.abs(velocity) > self.joints[joint_id].max_velocity:
                self.logger.warning(f"Joint {joint} cannot be moved with velocity {velocity} as it is over the max velocity "
                                    f"{self.joints[joint_id].max_velocity}")
            self.setJointMotorControl2(self.robot, robot_joint_id,
                                       controlMode=self.VELOCITY_CONTROL, targetVelocity=velocity,
                                       force=1 if velocity != 0 else 100,
                                       maxVelocity=self.joints[joint_id].max_velocity)
            self.joints[joint_id].set_point = "vel"

    def get_joint_state(self, joints=None, allow_error=False):
        """
        Get the state of the specified joints

        :param joints: joint or list of joints to get the state of
        :type joints: int or list, optional, default=None
        :return: list of states of the joints
        :rtype: list
        """
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

    def motion_done(self, joints=None, check_collision=True):
        """
        Checks whether the motion is done.

        :param joints: joint or list of joints to get the state of
        :type joints: int or list, optional, default=None
        :param check_collision: whether to check for collision during motion
        :type check_collision: bool, optional, default=True
        :return: True when motion is done, false otherwise
        :rtype: bool
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
        for joint in joints:
            robot_joint_id, joint_id = self.find_joint_id(joint)
            state = self.getJointState(self.robot, robot_joint_id)
            if self.joints[joint_id].set_point is not None and self.joints[joint_id].set_point != "vel":
                if np.abs(state[self.jointStates["POSITION"]] - self.joints[joint_id].set_point) > self.joint_tolerance:
                    return False

        self.stop_robot()
        return True

    def wait_motion_done(self, sleep_duration=0.01, check_collision=True):
        """
        Help function to wait for motion to be done. Can sleep for a specific duration

        :param sleep_duration: how long to sleep before running simulation step
        :type sleep_duration: float, optional, default=0.01
        :param check_collision: whether to check for collisions during motion
        :type check_collision: bool, optional, default=True
        """
        while not self.motion_done(check_collision=check_collision):
            self.update_simulation(sleep_duration)

    def stop_robot(self):
        """
        Stops the robot

        """
        joints = [joint.name for joint in self.joints]
        for joint in joints:
            robot_joint_id, joint_id = self.find_joint_id(joint)
            state = self.getJointState(self.robot, robot_joint_id)
            if self.joints[joint_id].set_point is not None:
                if self.joints[joint_id].set_point == "vel":
                    self.move_position(joint, state[self.jointStates["POSITION"]], wait=False, set_col_state=False)
                else:
                    self.move_position(joint, state[self.jointStates["POSITION"]], wait=False, set_col_state=False)
                self.joints[joint_id].set_point = None

    def move_cartesian(self, pose, wait=True, velocity=1, check_collision=True):
        """
        Move the robot in cartesian space by computing inverse kinematics and running position control

        :param pose: desired pose of the end effector
        :type pose: utils.Pose
        :param wait: whether to wait for movement completion
        :type wait: bool, optional, default=True
        :param velocity: joint velocity to move with
        :type velocity: float, optional, default=1
        :param check_collision: whether to check for collisions during motion
        :type check_collision: bool, optional, default=True
        """
        ik_solution = np.array(self.calculateInverseKinematics(self.robot, self.end_effector.link_id, pose.pos, pose.ori,
                                                               lowerLimits=self.IK_config["lower_limits"],
                                                               upperLimits=self.IK_config["upper_limits"],
                                                               jointRanges=self.IK_config["joint_ranges"],
                                                               restPoses=self.IK_config["rest_poses"]))
        self.move_position(self.IK_config["movable_joints"], ik_solution[self.IK_config["movable_joints"]], wait=False,
                           velocity=velocity)
        if wait:
            self.wait_motion_done(check_collision=check_collision)

    def find_joint_id(self, joint_name):
        """
        Help function to get indexes from joint name of joint index in self.joints list

        :param joint_name: name or index of the link
        :type joint_name: str or int
        :return: joint id in pybullet and pycub space
        :rtype: int, int
        """
        for joint in self.joints:
            if joint_name in [joint.name, joint.joints_id]:
                return joint.robot_joint_id, joint.joints_id

    def find_link_id(self, mesh_name, robot=None, urdf_name="robot"):
        """
        Help function to find link id from mesh name

        :param mesh_name: name of the mesh (only basename with extension)
        :type mesh_name: str
        :param robot: robot pybullet id
        :type robot: int, optional, default=None
        :param urdf_name: name of the object in pycub.urdfs
        :type urdf_name: str, optional, default="robot"
        :return: id of the link in pybullet space
        :rtype: int
        """
        if robot is None:
            robot = self.robot

        for link_id in range(0, len(self.urdfs[urdf_name].links)-1):
            cs = self.getCollisionShapeData(robot, link_id)
            if len(cs) > 0:
                if mesh_name == os.path.basename(cs[0][4].decode("utf-8")):
                    return link_id

    def run_vhacd(self):
        """
        Function to run VHACD on all objects in loaded URDFs, and to create new URDFs with changed collision meshes

        """
        something_changed = False
        for obj_name, obj in self.urdfs.items():
            for link in obj.links:
                if hasattr(link, "collision"):
                    if hasattr(link.collision.geometry, "mesh"):
                        col_path_ori = link.collision.geometry.mesh.filename
                        col_path = col_path_ori.replace("package://", "")
                        col_path = os.path.normpath(os.path.join(self.file_dir, "..", col_path))
                        vhacd_path = col_path.replace(".obj", "_vhacd.obj").replace("visual", "vhacd")
                        if self.config.vhacd.force_vhacd or not os.path.exists(vhacd_path):
                            self.vhacd(col_path, vhacd_path, "", resolution=1000000, maxNumVerticesPerCH=1, gamma=0.0005, concavity=0)
                        if self.config.vhacd.force_vhacd_urdf or not os.path.exists(vhacd_path):
                            something_changed = True
                        link.collision.geometry.mesh.filename = col_path_ori.replace("visual", "vhacd").replace(".obj", "_vhacd.obj")
            if something_changed or self.config.vhacd.force_vhacd_urdf or not obj.path.replace(".urdf", "_vhacd.urdf"):
                obj.write_urdf()
                obj.path = obj.path.replace("_fixed", "").replace(".urdf", "_vhacd.urdf")
                with open(obj.path, "w") as f:
                    f.write(obj.new_urdf)

    def create_urdf(self, object_path, fixed, color, suffix=""):
        """
        Creates a URDF for the given .obj file

        :param object_path: path to the .obj
        :type object_path: str
        :param fixed: whether the object is fixed in space
        :type fixed: bool
        :param color: color of the object
        :type color: list of 3 floats

        """
        with open(os.path.join(self.file_dir, "..", "other_meshes", "object_default.urdf"), "r") as f:
            urdf = f.read()
        if suffix != "":
            mesh = o3d.io.read_triangle_mesh(os.path.normpath(os.path.join(self.file_dir, "../other_meshes", object_path)))
        object_path = object_path.replace(".obj", suffix+".obj")
        object_path = os.path.normpath(os.path.join(self.file_dir, "../other_meshes", object_path))
        if suffix != "":
            o3d.io.write_triangle_mesh(object_path, mesh)

        model_name = os.path.basename(object_path).split(".")[0]
        urdf = urdf.replace("OBJECTNAME", model_name).replace("LATERALFRICTION", "1") \
            .replace("ROLLINGFRICTION", "0").replace("MASS", "0.2").replace("FILENAMECOLLISION", object_path) \
            .replace("FILENAME", object_path).replace("VISUALCOLOR", " ".join(map(str, color)))

        if fixed:
            with open(os.path.join(self.file_dir, "..", "other_meshes", "fixed_link.txt"), "r") as f:
                fixed_link_text = f.read()
            urdf = urdf.replace("</robot>", fixed_link_text)

        with open(object_path.replace(".obj", ".urdf"), "w") as f:
            f.write(urdf)

    def print_collision_info(self, c=None):
        """
        Help function to print collision info

        :param c: one collision
        :type c: list, optional, default=None
        """
        if c is None:
            contacts = self.getContactPoints(self.robot)
            for c in contacts:
                self.print_collision_info(c)
        else:
            if c[self.contactPoints['IDB']] == self.robot:
                body_b = "robot"
            else:
                for obj, name, _ in self.other_objects:
                    if c[self.contactPoints['IDB']] == obj:
                        body_b = name
                        break
            for link in self.links:
                if link.robot_link_id == c[self.contactPoints['INDEXA']]:
                    break
            link_a = link.name
            if c[self.contactPoints['IDB']] == self.robot:
                for link in self.links:
                    if link.robot_link_id == c[self.contactPoints['INDEXB']]:
                        break
                link_b = link.name
            else:
                link_b = f"{body_b} link {c[self.contactPoints['INDEXB']]}"
            self.logger.info(f"\nCollision of robot with {body_b}\n"
                             f"Collision of {link_a} with {link_b}\n"
                             f"Position A: {c[self.contactPoints['POSITIONA']]}\n"
                             f"Position B: {c[self.contactPoints['POSITIONB']]}\n"
                             f"Normal: {c[self.contactPoints['NORMAL']]}\n"
                             f"Distance: {c[self.contactPoints['DISTANCE']]}\n"
                             f"Force: {c[self.contactPoints['FORCE']]}\n"
                             f"Friction 1: {c[self.contactPoints['FRICTION1']]}\n"
                             f"Friction dir 1: {c[self.contactPoints['FRICTIONDIR1']]}\n"
                             f"Friction 2: {c[self.contactPoints['FRICTION2']]}\n"
                             f"Friction dir 2: {c[self.contactPoints['FRICTIONDIR2']]}\n")


class Joint:
    def __init__(self, name, robot_joint_id, joints_id, lower_limit, upper_limit, max_force, max_velocity):
        """
        Help class to encapsulate joint information

        :param name: name of the joint
        :type name: str
        :param robot_joint_id: id of the joint in pybullet
        :type robot_joint_id: int
        :param joints_id: id of the joint in pycub.joints
        :type joints_id: int
        :param lower_limit: lower limit of the joint
        :type lower_limit: float
        :param upper_limit: upper limit of the joint
        :type upper_limit: float
        :param max_force: max force of the joint
        :type max_force: float
        :param max_velocity: max velocity of the joint
        :type max_velocity: float
        """
        self.name = name
        self.robot_joint_id = robot_joint_id
        self.joints_id = joints_id
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit
        self.max_force = max_force
        self.max_velocity = max_velocity
        self.set_point = None

    def __repr__(self):
        return f"Joint {self.name} with id {self.robot_joint_id}"


class Link:
    def __init__(self, name, robot_link_id, urdf_link):
        """
        Help function to encapsulate link information

        :param name: name of the link
        :type name: str
        :param robot_link_id: id of the link in pybullet
        :type robot_link_id: int
        :param urdf_link: id of the link in pycub.urdfs["robot"].links
        :type urdf_link: int
        """
        self.name = name
        self.robot_link_id = robot_link_id
        self.urdf_link = urdf_link


class EndEffector:
    def __init__(self, name, client):
        """
        Help function for end-effector encapsulaation

        :param name: name of the end-effector
        :type name: str
        :param client: parent client
        :type client: pointer to pyCub instance
        """
        self.name = name
        self.client = client
        for link in self.client.urdfs["robot"].links:
            if link.name == self.name:
                self.link_id = self.client.find_link_id(os.path.basename(link.collision.geometry.mesh.filename))
                break

    def get_position(self):
        """
        Function to get current position of the end-effector
        """
        state = self.client.getLinkState(self.client.robot, self.link_id)
        pos = list(state[self.client.linkInfo["URDFPOS"]])
        ori = list(state[self.client.linkInfo["URDFORI"]])
        return Pose(pos, ori)
