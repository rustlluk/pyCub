"""
Exercise 3 Gaze Assignment

:Author: Lukas Rustler
"""
from icub_pybullet.pycub import pyCub
import numpy as np


def gaze(client: pyCub, head_direction: np.array, head_ball_direction: np.array) -> None:
    """
    Function that moves the neck to look at the ball based on the angle between head-to-ball and head vectors.

    :param client: instance of pyCub class
    :type client: pointer to pyCub
    :param head_direction: direction of the head; where the robot is looking
    :type head_direction: np.array
    :param head_ball_direction: vector between the head and the ball; where the robot should be looking
    :type head_ball_direction: np.array
    :return:
    :rtype:
    """

    # YOUR CODE HERE
    pass
