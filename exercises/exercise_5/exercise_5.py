"""
Exercise 4 Grasp it Assignment

:Author: Lukas Rustler
"""
from __future__ import annotations
from typing import Optional
import numpy as np


class Grasper:
    def __init__(self, client: pyCub, eye: Optional[str] = "l_eye"):
        """
        Class for grasping the ball

        :param client: instance of pyCub
        :type client: point to pyCub class
        :param eye: the eye to use
        :type eye: str
        """
        self.client = client
        self.eye = eye
        for link in self.client.links:
            if link.name == f"{eye}_pupil":
                self.eye_link_id = link.robot_link_id

    def get_rgb(self) -> np.array:
        """
        Get the RGB image from the camera

        :return: numpy array representing the image
        :rtype: np.array
        """
        return self.client.get_camera_images(self.eye)[self.eye]

    def get_depth(self) -> np.array:
        """
        Get the depth image from the camera

        :return: numpy array representing the image
        :rtype: np.array
        """
        return self.client.get_camera_depth_images(self.eye)[self.eye]

    def get_3d_point(self, u: int, v: int, d: float) -> np.array:
        """
        Function to deproject point in camera image into 3D point in world coordinates

        :param u: u pixel coordinate in the image
        :type u: int
        :param v: v pixel coordinate in the image
        :type v: int
        :param d: depth values (in meters)
        :type d: float
        :return: 3D point in world coordinates
        :rtype: np.array
        """

        ew = self.client.visualizer.eye_windows[self.eye]
        return ew.unproject(u, v, d)

    def move_fingers(self, closure: Optional[float] = 1.0, hand: Optional[str] = "right",
                     timeout: Optional[float] = 10) -> None:
        """

        :param closure: 0 = open, 1 = close
        :type closure: float
        :param hand: which hand to move
        :type hand: str
        :param timeout: time to wait for the fingers to move
        :type timeout: float
        :return:
        :rtype:
        """

        # joint to move
        hand = "r" if hand == "right" else "l"
        joints = [f"{hand}_hand_thumb_2_joint", f"{hand}_hand_thumb_3_joint",
                  f"{hand}_hand_index_2_joint", f"{hand}_hand_index_3_joint",
                  f"{hand}_hand_middle_2_joint", f"{hand}_hand_middle_3_joint",
                  f"{hand}_hand_ring_2_joint", f"{hand}_hand_ring_3_joint",
                  f"{hand}_hand_little_2_joint", f"{hand}_hand_little_3_joint"]

        # Move all the joints
        for joint in joints:
            joint_handle = self.get_joint_handle(joint)
            self.client.move_position(joint, joint_handle.lower_limit + closure *
                                      (joint_handle.upper_limit - joint_handle.lower_limit), wait=False,
                                      check_collision=False, timeout=timeout, velocity=5)

        # wait for completion and do not care about collisions
        while not self.client.motion_done(check_collision=False):
            self.client.update_simulation()

    def get_pupil_vectors(self, point: np.array) -> tuple:
        """
        Function to compute vector from pupil to ball (where the robot should be looking)
        and vector from the pupil (where to robot is looking)

        :param point: 3D position of the ball
        :type point: numpy array
        :return: vectors from pupil to ball and from pupil
        :rtype: tuple
        """

        # Get head link position and orientation
        head_state = self.client.getLinkState(self.client.robot, self.eye_link_id, computeLinkVelocity=0, computeForwardKinematics=0)
        head_pos, head_ori = head_state[0], head_state[1]

        # The direction of a vector from the ball to the head
        pupil_ball_direction = np.array(point) - head_pos
        pupil_ball_len = np.linalg.norm(pupil_ball_direction)
        pupil_ball_direction /= pupil_ball_len  # normalize

        R_l = np.eye(4)
        R_l[:3, :3] = np.reshape(self.client.getMatrixFromQuaternion(head_ori), (3, 3))
        pupil_direction = np.matmul(R_l, [0, 0, 1, 1])[:3]  # Rotate Z-axis vector to point in direction of head
        pupil_direction /= np.linalg.norm(pupil_direction)  # normalize

        return pupil_ball_direction, pupil_direction

    def get_joint_handle(self, joint_name: str) -> pyCub.Joint:
        """
        Help function to get the handle of the joint by name

        :param joint_name: name of a joint
        :type joint_name: str
        :return: handle to the joint
        :rtype: pycub.Joint
        """
        for joint in self.client.joints:
            if joint.name == joint_name:
                return joint

    @staticmethod
    def quaternion_swap(q: list | np.array, to: Optional[str] = "wxyz") -> list:
        """
        Help function to convert between different quaternion representations

        :param q: quaternion
        :type q: list or numpy array
        :param to: to which representation to convert; "wxyz" or "xyzw"; there is no check whether the input quaternion in the other format!
        :type to: str
        :return: quaternion in desired format
        :rtype: list
        """
        if to == "wxyz":
            return [q[3], q[0], q[1], q[2]]
        elif to == "xyzw":
            return [q[1], q[2], q[3], q[0]]

    def find_the_ball(self) -> tuple:
        """
        Function to find the ball in the image

        HINTS:
            - The ball is green like (0, 255, 0) in RGB
                - but there is sun in the visualizer, so the color might be slightly different
            - The class contains a method to get the RGB image

        :return: 2D position of center of the ball in image plane
        :rtype: tuple
        """

        # TODO find the ball in the image and return its 2D center


        return center

    def grasp(self, center: tuple | list | np.array) -> int:
        """
        Function to grasp the ball based on 2D center.

        HINTS:
            - The class contains a method to get the depth image
            - The class contains a method to move the fingers
            - The class contains a method to get the 3D position of the ball
            - The class contains a method to get vector from the camera (pupil) to ball
            - The ball's radius is 2.5 cm
            - It is better to move few cm above the ball before grasping it
            - It is useful to bend the wrist before grasp


        :param center: 2D position of center of the ball in image plane
        :type center: tuple
        :return:
        :rtype:
        """

        # TODO get 3D position of the ball, look at it and grasp it

        return 0
