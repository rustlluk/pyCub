"""
Script to the test the skin sensors.

:Author: Lukas Rustler
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pycub import pyCub


if __name__ == "__main__":
    client = pyCub(config="skin_test.yaml")

    while client.is_alive():
        client.update_simulation(None)
