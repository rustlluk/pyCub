"""
Example of how to push the ball from the table using only pure joint control. It works without planner of collisions
detection/avoidance. It is not very robust, and it is laborious, but it is a good starting point for your
own experiments.

:Author: Lukas Rustler
"""
from icub_pybullet.pycub import pyCub
from typing import NoReturn


def push_the_ball(client: pyCub) -> None:
    """
    Example function to push the ball from the table with joint control.

    :param client: instance of pyCub
    :type client: pyCub
    :return:
    :rtype:
    """
    """
    """

    # move torso pitch so the robot bends over the table
    client.move_position("torso_pitch", 0.325)

    # move torso to push the ball, but do now wait for completion. Hit it a bit faster
    client.move_position("torso_yaw", -0.7, wait=False, velocity=5)

    # wait manually
    while not client.motion_done():
        client.update_simulation()

    """
    Wait could be also achieved with:
    client.wait_motion_done()
    """

    client.logger.info("Moved the ball!")


def main() -> NoReturn:
    """
    Main function to run the example

    :return:
    :rtype:
    """
    # load the robot with correct world/config
    client = pyCub(config="with_ball.yaml")

    push_the_ball(client)

    # just wait until the gui is closed
    while client.is_alive():
        client.update_simulation()


if __name__ == "__main__":
    main()
