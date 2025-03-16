"""
Script to the test the skin sensors.

:Author: Lukas Rustler
"""
from icub_pybullet.pycub import pyCub


def main():
    client = pyCub(config="skin_test.yaml")

    while client.is_alive():
        client.update_simulation(None)


if __name__ == "__main__":
    main()
