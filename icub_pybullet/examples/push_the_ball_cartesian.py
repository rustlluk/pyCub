"""
Example of moving the robot in cartesian space to push the ball. It is more robust than the pure joint control.

:Author: Lukas Rustler
"""
from icub_pybullet.pycub import pyCub, Pose
from typing import NoReturn


def push_the_ball(client: pyCub) -> None:
    """
    Example function to move the ball with cartesian control.

    :param client: instance of pyCub
    :type client: pyCub
    :return:
    :rtype:
    """

    # Get current pose
    cur_pose = client.end_effector.get_position()

    # Copy, the pose and go 15cm lower and 10cm closer to the robot
    new_pose = Pose(cur_pose.pos, cur_pose.ori)
    new_pose.pos[2] -= 0.15
    new_pose.pos[0] -= 0.1

    # move
    client.move_cartesian(new_pose)

    # get current pose
    pose = client.end_effector.get_position()

    # assign straight to it to move hand left
    pose.pos[1] -= 0.2

    # move; do not wait for completion; and move a bit faster
    client.move_cartesian(pose, wait=False, velocity=2)

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

    # # just wait until the gui is closed
    while client.is_alive():
        client.update_simulation()


if __name__ == "__main__":
    main()