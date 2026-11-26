"""
Exercise 4 Grasp it Tester

:Author: Lukas Rustler
"""

from __future__ import annotations
import os
from icub_pybullet.pycub import pyCub
import numpy as np
import traceback


def write_results(upl_dir: str, msg: str) -> None:
    with open(os.path.join(upl_dir, "results.html"), "w", encoding="utf-8") as f:
        f.write(f"<html>\n  <body>\n      {msg}\n  </body>\n</html>")


def main() -> int:
    results_path = os.path.join(os.path.realpath(os.path.dirname(os.path.basename(__file__))), "results")
    if not os.path.exists(results_path):
        os.makedirs(results_path)

    try:
        from exercise_5 import Grasper
    except Exception:
        tr = str(traceback.format_exc())
        print("\x1b[31;20m" + "Cannot load class Grasper from exercise_5.py")
        print("\x1b[31;20m" + tr)
        msg = (f"<p style='color:red'>Cannot load class Grasper from exercise_5.py</p>"
               f"<p style='color:red'>{tr}</p>")
        write_results(results_path, msg)
        return 1

    client = pyCub(config="exercise_5.yaml")
    grasper = Grasper(client)

    for i in range(10):
        client.update_simulation()

    try:
        center = grasper.find_the_ball()
        grasper.grasp(center)
    except Exception:
        tr = str(traceback.format_exc())
        client.logger.error(tr)
        msg = f"<p style='color:red'>{tr}</p>"
        write_results(results_path, msg)
        return 2

    c = client.getClosestPoints(client.other_objects[1][0], client.other_objects[2][0], np.inf)
    min_dist = np.inf
    for _ in c:
        d = _[client.contactPoints["DISTANCE"]]
        if d < min_dist:
            min_dist = d

    min_dist = np.abs(np.round(min_dist, 3))
    client.logger.info(f"The distance of ball to table is: {min_dist * 100}cm")

    msg = f"<p>The distance of ball to table is: {min_dist * 100}cm</p>"
    msg += (
        "<h2 style='color:green'>It is more than 5cm! Test passed</h2>"
        if min_dist > 0.05
        else "<h2 style='color:red'>It is less than 5cm!. Test failed</h2>"
    )
    write_results(results_path, msg)
    return 0


if __name__ == "__main__":
    main()
