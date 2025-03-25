"""
Exercise 4 RRMC tester

:Author: Lukas Rustler
"""
from icub_pybullet.pycub import pyCub
import traceback


def main() -> int:
    """
    Main function to test the exercise

    :return: 0 if successful, 1 if function cannot be loaded, 2 if function throws an exception
    :rtype: int
    """

    # try to load process function from exercise_4.py
    try:
        from exercise_4 import RRMC
    except Exception as e:
        tr = str(traceback.format_exc())
        print("\x1b[31;20m" + tr)

        return 1

    for test in [0, 1, 2, 3]: # 4 different configs to test
        client = pyCub(config=f"configs/exercise_4_{test}.yaml")
        rrmc = RRMC(client)
        last_steps = None
        try:
            while client.is_alive():
                if last_steps is None or last_steps != client.steps_done:  # if simulation step was done
                    rrmc.process()

                    # we want to call RRMC only if iteration of simulation was done
                    last_steps = client.steps_done

                client.update_simulation(0.01)
        except Exception as e:
            tr = str(traceback.format_exc())
            client.logger.error(tr)
            return 2

    return 0


if __name__ == "__main__":
    main()
