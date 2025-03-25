"""
Exercise 4 Grasp it Tester

:Author: Lukas Rustler
"""

from __future__ import annotations
import os
from icub_pybullet.pycub import pyCub
import numpy as np
import traceback


def write_results(upl_dir: str , msg: str) -> None:
    """
    Function to write results to a html file

    :param upl_dir: path to the results directory
    :type upl_dir: str
    :param msg: message to be written
    :type msg: str
    :return:
    :rtype:
    """
    with open(os.path.join(upl_dir, "results.html"), "w") as f:
        f.write(f"<html>\n"
                f"  <body>\n"
                f"      {msg}\n"
                f"  </body>\n"
                f"</html>")

def main() -> int:
    """
    Main function to test the grasp function


    :return: 0 if successful, 1 if Grasper class cannot be loaded, 2 if any other exception occurs
    :rtype:
    """

    # create results directory if not exists
    results_path = os.path.join(os.path.realpath(os.path.dirname(os.path.basename(__file__))), "results")
    if not os.path.exists(results_path):
        os.makedirs(results_path)

    try:
        from exercise_5 import Grasper
    except Exception as e:
        tr = str(traceback.format_exc())
        print("\x1b[31;20m" + "Cannot load class Grasper from exercise_5.py")
        print("\x1b[31;20m" + tr)
        msg = (f"<p style='color:red'>Cannot load class Grasper from exercise_5.py</p>"
               f"<p style='color:red'>{tr}</p>")

        write_results(results_path, msg)
        return 1

    # get client and grasper
    client = pyCub(config="exercise_5.yaml")
    grasper = Grasper(client)

    #WARMUP for images
    for i in range(10):
        client.update_simulation()

    # try to find and grasp the ball
    try:
        center = grasper.find_the_ball()
        grasper.grasp(center)
    except Exception as e:
        tr = str(traceback.format_exc())
        client.logger.error(tr)
        msg = f"<p style='color:red'>{tr}</p>"
        write_results(results_path, msg)
        return 2

    # get closest points between ball and table
    c = client.getClosestPoints(client.other_objects[1][0], client.other_objects[2][0], np.Inf)
    min_dist = np.Inf
    # find the highest distance
    for _ in c:
        d = _[client.contactPoints["DISTANCE"]]
        if d < min_dist:
            min_dist = d


    # round and calculate score
    min_dist = np.abs(np.round(min_dist, 3))
    client.logger.info(f"The distance of ball to table is: {min_dist*100}cm")

    msg = f"<p>The distance of ball to table is: {min_dist*100}cm</p>"
    msg += f"<h2 style='color:green'>It is more than 5cm! Test passed</h2>" if min_dist > 0.05 else f"<h2 style='color:red'>It is less than 5cm!. Test failed</h2>"
    write_results(results_path, msg)


if __name__ == "__main__":
    main()
