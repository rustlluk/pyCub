"""
Script to the test the skin sensors.

:Author: Lukas Rustler
"""
import sys
sys.path.insert(0, '..')
from pycub import pyCub


if __name__ == "__main__":
    client = pyCub(config="skin_test.yaml")

    while client.is_alive():
        client.update_simulation(None)
