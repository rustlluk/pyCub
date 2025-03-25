"""
Exercise 4 RRMC assignment

:Author: Lukas Rustler
"""

from __future__ import annotations

class RRMC:
    def __init__(self, client: pyCub) -> None:
        """
        Constructor of the RRMC class.

        :param client: instance of pyCub
        :type client: pointer to pyCub class
        """
        self.client = client

    def get_body_part_from_skin_part(self, skin_part: str) -> str:
        """
        Help function to get the body part corresponding to a given skin part.

        :param skin_part: name of the skin part
        :type skin_part: str
        :return: name of the chain (body part)
        :rtype: str
        """
        for chain_name, chain_links in self.client.chains.items():
            if skin_part in chain_links:
                return chain_name

    def process(self) -> None:
        """
        Function to process skin event and move the robot.

        Things that can help you:
            - client.activated_skin_points - dictionary with skin part as key and list of activated points as value
            - client.activated_skin_normals - dictionary with skin part as key and list of activated normals as value
            - body_part = get_body_part_from_skin_part(client, skin_part) - returns the body part (the one you want to move) that the skin part belongs to
            - jac, joints = client.compute_jacobian(body_part, end=skin_part) - returns the Jacobian matrix and the joint indexes of the body part
            - client.move_velocity(joints, joint_vel) - moves the robot in velocity space
            - client.stop_robot(joints) - stops the given joints of the robot

        :return:
        :rtype:
        """

        # YOUR CODE HERE
