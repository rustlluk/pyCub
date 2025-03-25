"""
Exercise 3 Gaze Tester

:Author: Lukas Rustler
"""
from __future__ import annotations
import os
import sys
from typing import Optional
from icub_pybullet.pycub import pyCub
import open3d as o3d
import numpy as np
import open3d.visualization.rendering as rendering
import time
import matplotlib.pyplot as plt
import traceback


class Controller:
    def __init__(self, client: pyCub):
        """
        Class for ball moving

        :param client: instance of pyCub
        :type client: point to pyCub class
        """
        self.client = client
        self.step = -1

        # Prepare line material for open3d
        self.mat = rendering.MaterialRecord()
        self.mat.shader = "unlitLine"
        self.mat.line_width = 10

        # look down and move arms from the view
        self.client.move_position("neck_pitch", -0.54, wait=False)
        self.client.move_position(["l_shoulder_pitch", "l_shoulder_roll", "r_shoulder_pitch", "r_shoulder_roll"],
                             [-1.5, 1.5, -1.5, 1.5], wait=False)

        while not self.client.motion_done():
            _, _ = self.draw_lines(client)
            self.client.update_simulation()

        self.step = 0

    def draw_lines(self, client: pyCub, K: Optional[float] = 2.5) -> tuple:
        """
        Draw lines representing view vector and vector from the ball to the head. At given intervals applies force to the ball.

        :param client: instance of pyCub
        :type client: pointer to pyCub class
        :param K: force applied to the ball
        :type K: float
        :return:
        :rtype:
        """
        # get ball position
        ball_pos, _ = self.client.getBasePositionAndOrientation(client.free_objects[0])

        # Get head link position and orientation
        head_state = self.client.getLinkState(self.client.robot, 96, computeLinkVelocity=0, computeForwardKinematics=0)
        head_pos, head_ori = head_state[0], head_state[1]

        # The direction of a vector from the ball to the head
        head_ball_direction = np.array(ball_pos) - head_pos
        head_ball_len = np.linalg.norm(head_ball_direction)
        head_ball_direction /= head_ball_len # normalize

        # any gui is active
        if self.client.config.gui.standard or self.client.config.gui.web:

            # remove last line
            self.client.visualizer.vis.remove_geometry("ball_line")
            # compute new line and add it to the visualizer
            lb = o3d.geometry.LineSet(o3d.utility.Vector3dVector([head_pos, head_pos + head_ball_len * head_ball_direction]),
                                      o3d.utility.Vector2iVector([[0, 1]]))
            lb.colors = o3d.utility.Vector3dVector([[1, 0, 0]])

            # draw the line from head to ball
            self.client.visualizer.vis.add_geometry("ball_line", geometry=lb, material=self.mat)

        # Direction of the gaze is the Z-axis rotated by the head orientation
        R_l = np.eye(4)
        R_l[:3, :3] = np.reshape(self.client.getMatrixFromQuaternion(head_ori), (3, 3))
        head_direction = np.matmul(R_l, [0, 0, 1, 1])[:3]  # Rotate Z-axis vector to point in direction of head
        head_direction /= np.linalg.norm(head_direction)  # normalize

        # if any gui is active
        if self.client.config.gui.standard or self.client.config.gui.web:

            # remove the last line
            self.client.visualizer.vis.remove_geometry("head_line")

            # compute new line and add it to the visualizer
            lh = o3d.geometry.LineSet(o3d.utility.Vector3dVector([head_pos, head_pos + head_ball_len * head_direction]),
                                      o3d.utility.Vector2iVector([[0, 1]]))
            lh.colors = o3d.utility.Vector3dVector([[0, 0, 1]])

            # draw the line from head
            self.client.visualizer.vis.add_geometry("head_line", geometry=lh, material=self.mat)

        # every simulated second
        if self.step%(1/self.client.config.simulation_step) == 0:

            # apply force to the ball so it moves
            signs = [-1, 1]
            f_vec = [0, 0, 0]

            # get random axis and sign of the force
            rrs = np.random.randint(0, 2, (2,))
            f_vec[rrs[0]] = K * signs[rrs[1]]

            # apply
            self.client.applyExternalForce(self.client.free_objects[0], -1, f_vec, [0, 0, 0], self.client.WORLD_FRAME)

        if self.step != -1:
            self.step += 1
        return head_direction, head_ball_direction


def gaze_eval(head_direction: np.array, head_ball_direction: np.array) -> float:
    """
    Evaluates the current angle between the two vectors

    :param head_direction: direction of the head; where the robot is looking
    :type head_direction: np.array
    :param head_ball_direction: vector between the head and the ball; where the robot should be looking
    :type head_ball_direction: np.array
    :return:
    :rtype:
    """

    # angle between the vectors
    angle = np.arctan2(np.linalg.norm(np.cross(head_direction, head_ball_direction)),
                       np.dot(head_direction, head_ball_direction))

    return angle

def visualize(errors: np.array, results_path: str, rep: int) -> None:
    """
    Function to visualize the errors

    :param errors: errors in each step
    :type errors: np.array
    :param results_path: path to the results folder
    :type results_path: str
    :param rep: repetition
    :type rep: int
    :return:
    :rtype:
    """

    plt.figure()
    plt.plot(errors)
    plt.xlabel("Step [-]")
    plt.ylabel("Error [deg]")

    # save for later
    plt.savefig(os.path.join(results_path, f"errors_{rep}.png"))


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

def main(rep_count: Optional[int] = 15, time_to_run: Optional[float] = 10, max_k: Optional[float] = 7) -> int:
    """
    Main function to test the gaze function

    :param rep_count: number of repetitions; default is 15
    :type rep_count: int
    :param time_to_run: time to run the simulation; default is 10
    :type time_to_run: float
    :param max_k: maximum force applied to the ball; default is 7
    :type max_k: float
    :return:
    :rtype:
    """
    # create results directory if not exists
    results_path = os.path.join(os.path.realpath(os.path.dirname(os.path.basename(__file__))), "results")
    if not os.path.exists(results_path):
        os.makedirs(results_path)

    try:
        from exercise_3 import gaze
    except Exception as e:
        tr = str(traceback.format_exc())
        print("\x1b[31;20m" + "Cannot load function gaze from exercise_3.py")
        msg = (f"<p style='color:red'>Cannot load function move from exercise_3.py</p>"
               f"<p style='color:red'>{tr}</p>")

        write_results(results_path, msg)
        return 1

    # define penalties and max score
    max_score = 10
    mean_ths = [5, 1, 0.55]
    mean_ths_pen = [1, 0.75, 0.5]
    max_ths = [10, 5, 1.25]
    max_ths_pen = [1, 0.5, 0.25]

    scores = []
    msg = ""

    for rep in range(rep_count):
        # The force K is defined as rep//2 + 2, but not more than 7 (it is too fast for the reference solution)
        K = np.min([(rep // 2) + 2, max_k])

        client = pyCub(config="exercise_3.yaml")
        c = Controller(client)

        errors = []
        t = time.time()
        last_steps = None
        try:
            while client.is_alive() and time.time() - t < time_to_run: # run for time_to_run seconds
                if last_steps is None or last_steps != client.steps_done: # if simulation step was done

                    # draw lines and get the gaze direction
                    head_direction, head_ball_direction = c.draw_lines(client, K)

                    # call the gaze function
                    gaze(client, head_direction, head_ball_direction)

                    # compute the error and save it
                    error = gaze_eval(head_direction, head_ball_direction)
                    errors.append(error)

                    # we want to call gaze and error only if iteration of simulation was done
                    last_steps = client.steps_done

                client.update_simulation(0.01) # 0.01 for regularization between computers
        except Exception as e:
            tr = str(traceback.format_exc())
            client.logger.error(tr)
            msg = f"<p style='color:red'>{tr}</p>"
            write_results(results_path, msg)
            return 2

        try:
            visualize(np.rad2deg(errors), results_path, rep)
        except Exception as e:
            tr = str(traceback.format_exc())
            client.logger.error(tr)
            msg = f"<p style='color:red'>{tr}</p>"
            write_results(results_path, msg)
            return 3

        # compute score using predefined thresholds
        mean_pen = np.where(np.mean(np.rad2deg(np.abs(errors))) > mean_ths)[0]
        if len(mean_pen) > 0:
            mean_pen = mean_ths_pen[mean_pen[0]] * max_score
        else:
            mean_pen = 0

        max_pen = np.where(np.max(np.rad2deg(np.abs(errors) )) > max_ths)[0]
        if len(max_pen) > 0:
            max_pen = max_ths_pen[max_pen[0]] * max_score
        else:
            max_pen = 0

        # score is temp_score / max_score to be normalized from 0 to 1
        temp_score = max(0, max_score - mean_pen - max_pen)
        msg += f"<h2>Test {rep + 1}</h2>"
        msg += f'<p><img src="errors_{rep}.png"></p>'
        msg += f"<p>Mean error: {np.mean(np.rad2deg(np.abs(errors)))} deg</p>"
        msg += f"<p>Max error: {np.max(np.rad2deg(np.abs(errors)))} deg</p>"
        msg += f"<p>Score: {temp_score / max_score}</p>\n"
        scores.append(temp_score / max_score)

        del client, c

    score = np.sum(scores)

    # print final results
    msg += f"<h2>Final score: {score}</h2>"
    print('\x1b[97;10m' + f"Your score is {score} out of {rep_count}")
    write_results(results_path, msg)
    return 0

if __name__ == "__main__":
    if len(sys.argv) > 1:
        reps = int(sys.argv[1])
    else:
        reps = 10

    main(reps)
