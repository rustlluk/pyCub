"""
Exercise 1 Push the Ball assignment

:Author: Lukas Rustler
"""
from icub_pybullet.pycub import pyCub

def push_the_ball(client: pyCub) -> int:
    """
    The function to push ball from the table.

    You should fill in the code such that the ball will end up as far as possible from the table.
    You should return the code right after the last movement.

    :param client: instance of pyCub
    :type client: pyCub
    :return: success code
    :rtype: int
    """
    # WRITE YOUR CODE HERE so that the robot pushes the ball away from the table
    return 0


def main():
    """
    Help main class

    :return:
    :rtype:
    """
    # load the robot with correct world/config
    client = pyCub(config="exercise_1.yaml")

    push_the_ball(client)

if __name__ == "__main__":
    main()