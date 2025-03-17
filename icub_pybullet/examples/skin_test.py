"""
Script to the test the skin sensors. Balls falling to the skin and turning activated point to green should be seen.

:Author: Lukas Rustler
"""
from icub_pybullet.pycub import pyCub
from typing import NoReturn


def main() -> NoReturn:
    """
    Main function to run the example

    :return:
    :rtype:
    """
    client = pyCub(config="skin_test.yaml")

    while client.is_alive():
        client.update_simulation(None)


if __name__ == "__main__":
    main()
