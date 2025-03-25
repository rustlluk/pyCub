"""
Exercise 2 Smooth Movements Assignment

:Author: Lukas Rustler
"""

from __future__ import annotations
from icub_pybullet.pycub import pyCub

def move(client: pyCub, action: str, axis: list, r: list) -> tuple:
    """
    The main move function that moves the end effector in a line or circle.

    :param client: instance of pyCub
    :type client: pointer to pyCub
    :param action: name of the action; "line" or "circle"
    :type action: str
    :param axis: axes of the action; list of 1 or 2 elements
    :type axis: list of int
    :param r: radius/lengths of the action; list of 1 or 2 elements
    :type r: list of float
    :return:
    :rtype:
    """
    # YOUR CODE HERE

    return start_pose, end_pose


if __name__ == "__main__":
    client = pyCub(config="exercise_2.yaml")

    move(client, action="line", axis=[0], r=[0.05])
