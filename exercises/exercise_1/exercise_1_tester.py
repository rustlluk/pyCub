"""
Exercise 1 Push the Ball tester

:Author: Lukas Rustler
"""
from icub_pybullet.pycub import pyCub
import time
import numpy as np
import os
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


def evaluate(client: pyCub, upl_dir: str) -> None:
    """
    Function to evaluate the results of the exercise

    :param client: pyCub instance
    :type client: pyCub
    :param upl_dir: path to results directory
    :type upl_dir: str
    :return:
    :rtype:
    """
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
    score = np.round(np.min([min_dist*2, 3]), 2)
    msg = f"<h1>Results</h1>\n"\
          f"<p>The distance of ball to table is: {min_dist}m</p>\n"\
          f"<p>Your score is {score} out of 3</p>\n"
    write_results(upl_dir, msg)
    client.logger.info(f"You moved the ball {min_dist}m away from the table. Your score is {score}")


def main() -> int:
    """
    Main function to test the exercise

    :return: 0 if successful, 1 if function cannot be loaded, 2 if function throws an exception
    :rtype: int
    """

    # create results directory if not exists
    results_path = os.path.join(os.path.realpath(os.path.dirname(os.path.basename(__file__))), "results")
    if not os.path.exists(results_path):
        os.makedirs(results_path)

    client = pyCub(config="exercise_1.yaml")

    # try to load push_the_ball function from exercise_1.py
    try:
        from exercise_1 import push_the_ball
    except Exception as e:
        tr = str(traceback.format_exc())
        client.logger.error("Cannot load function push_the_ball from exercise_1.py")
        msg = (f"<p style='color:red'>Cannot load function push_the_ball from exercise_1.py</p>"
               f"<p style='color:red'>{tr}</p>")

        write_results(results_path, msg)
        return 1

    # try to push the ball
    try:
        push_the_ball(client)
    except Exception as e:
        tr = str(traceback.format_exc())
        client.logger.error(tr)
        msg = f"<p style='color:red'>{tr}</p>"
        write_results(results_path, msg)
        return 2

    # simulate for 2 seconds
    s = time.time()
    while client.is_alive():
        client.update_simulation(0.01) # simulation delay of 0.01 for some kind of regularization between computers
        if time.time() - s > 2:
            break
    evaluate(client, results_path)
    return 0


if __name__ == "__main__":
    main()
