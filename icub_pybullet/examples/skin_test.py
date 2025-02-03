"""
Script to the test the skin sensors.

:Author: Lukas Rustler
"""
try:
    from icub_pybullet.pycub import pyCub
except:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from icub_pybullet.pycub import pyCub


def main():
    client = pyCub(config="skin_test.yaml")

    while client.is_alive():
        client.update_simulation(None)


if __name__ == "__main__":
    main()
