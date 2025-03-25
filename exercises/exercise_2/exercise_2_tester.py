"""
Exercise 2 Smooth Movements Tester

:Author: Lukas Rustler
"""

from __future__ import annotations
import os
from icub_pybullet.pycub import pyCub
import copy
import numpy as np
import matplotlib.pyplot as plt
import scipy
import traceback


class Evaluator:
    def __init__(self, client: pyCub, action: str, axis: list, r: list, test_id: int, results_path: str):
        """
        Function to evaluate the test case

        :param client: instance of pyCub
        :type client: pointer to pyCub
        :param action: name of the action; "line" or "circle"
        :type action: str
        :param axis: axes of the action; list of 1 or 2 elements
        :type axis: list of int
        :param r: radius/lengths of the action;list of 1 or 2 elements
        :type r: list of float
        :param test_id: number of test
        :type test_id: int
        :param results_path: path to the result folder
        :type results_path: str
        """
        self.client = client
        self.poses = []
        self.read = True
        self.action = action
        self.axis = axis
        self.r = r
        self.path = None
        self.dir_vector = np.zeros(3)
        self.hsd = None
        self.plane_dist = None
        self.length_diff = None
        self.steps = 1
        self.axes = np.array(["x", "y", "z"])
        self.vector = None
        self.score = 0
        self.results_path = results_path
        self.test_id = test_id

        # Init string for html result
        self.eval_string = (f"<h1>Test {test_id}</h1><p>action: {action}, axis: {axis}, r: {r}</p><p>" +
                            "".join(["-"]*50)+"</p>")

    def get_gt_trajectory(self) -> None:
        """
        Function to compute the ground truth trajectory

        :return:
        :rtype:
        """
        steps = 50
        if self.action == "line":
            self.dir_vector[self.axis] = self.r
            line = np.linspace(0, self.dir_vector, steps)  # just a basic linspace
            self.path = line
        elif self.action == "circle":
            self.dir_vector[self.axis] = 1
            th = np.linspace(0, 2 * np.pi, steps).reshape(steps, 1) # linspace of 0 to 2*pi = circle
            null_space = scipy.linalg.null_space(self.dir_vector.reshape(1, 3)) # null space of the dir_vector
            # these two vectors are perpendicular to the dir_vector, i.e., the vectors that define the plane
            u = null_space[:, 0] # first vector of the null space
            v = null_space[:, 1] # second vector of the null space

            # parametric equation of a circle in 3D
            self.path = self.r*np.multiply(np.cos(th), u) + self.r*np.multiply(np.sin(th), v)

    def get_dists(self, start_pose: Pose):
        """
        Function to compute the metrics of trajectories

        :param start_pose: start pose of the trajectory
        :type start_pose: utils.Pose
        :return:
        :rtype:
        """

        if self.action == "line": # angle between the expected and real line
            diffs = np.diff(self.poses, axis=0)
            dif_norms = np.linalg.norm(diffs, axis=1)

            use_diffs = np.ones(dif_norms.shape, dtype=bool)
            for i in range(dif_norms.size):
                if dif_norms[i] == 0:
                    use_diffs[i] = False
            diffs = diffs[use_diffs, :]
            dif_norms = dif_norms[use_diffs]

            diffs = diffs / dif_norms.reshape(-1, 1)
            mean_diff = np.mean(diffs, axis=0)
            mean_diff /= np.linalg.norm(mean_diff)
            x = np.zeros(3)
            x[self.axis] = self.r
            self.vector = x / np.linalg.norm(x)
            self.plane_dist = np.arctan2(np.linalg.norm(np.cross(mean_diff, self.vector)), np.dot(mean_diff, self.vector))

        elif self.action == "circle":  # angle between expected and real plane in which the circle is drawn
            dists = np.dot(self.poses, np.reshape(self.dir_vector, (3, 1)))-np.array(start_pose.pos)[self.axis]
            self.plane_dist = np.mean(np.abs(dists))

        if self.action == "circle":
            self.poses -= np.mean(self.poses, axis=0)
            self.length_diff = np.std(np.linalg.norm(self.poses, axis=1))  # std of points from the center -> "how circle is the circle"
            self.hsd = np.abs(np.mean(np.linalg.norm(self.poses, axis=1))-self.r[0]) # mean radius difference

        elif self.action == "line":
            self.poses -= start_pose.pos
            poses_length = 0
            for i in range(1, self.poses.shape[0]):
                poses_length += np.linalg.norm(self.poses[i, :] - self.poses[i-1, :])
            self.length_diff = np.abs(np.linalg.norm(self.path[0, :] - self.path[-1, :])-poses_length) # length difference
            self.hsd = np.mean(np.linalg.norm(np.cross(self.poses, self.poses-self.vector), axis=1)) # mean distance from the line

    def visualize(self) -> None:
        """
        Function to visualize the trajectory using matplotlib

        :return:
        :rtype:
        """
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(self.path[:, 0], self.path[:, 1], self.path[:, 2], c='b')
        ax.scatter(self.poses[:, 0], self.poses[:, 1], self.poses[:, 2], c='k')
        plt.tight_layout()
        try:
            ax.set_aspect('equal', adjustable='box')
        except:
            x_limits = ax.get_xlim()
            y_limits = ax.get_ylim()
            z_limits = ax.get_zlim()

            x_range = x_limits[1] - x_limits[0]
            y_range = y_limits[1] - y_limits[0]
            z_range = z_limits[1] - z_limits[0]

            max_range = max(x_range, y_range, z_range) / 2.0

            mid_x = np.mean(x_limits)
            mid_y = np.mean(y_limits)
            mid_z = np.mean(z_limits)

            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_zticks([])

        # plt.show()
        plt.savefig(os.path.join(self.results_path, f"test_{self.test_id}.png"))

    def evaluate(self, start_pose: Pose, end_pose: Pose):
        """
        Main evaluation function

        :param start_pose: start pose of the movement
        :type start_pose: utils.Pose
        :param end_pose: end pose of the movement
        :type end_pose: utils.Pose
        :return:
        :rtype:
        """
        self.poses = np.array([_.pos for _ in self.client.pose_logger])

        # Get the start and end index of the trajectory
        si = np.argmin(np.linalg.norm(self.poses - np.array(start_pose.pos), axis=1))
        ei = np.argmin(np.linalg.norm(self.poses - np.array(end_pose.pos), axis=1))

        self.poses = self.poses[si:ei + 1, :]

        # Compute the ground truth trajectory and distance to it
        self.get_gt_trajectory()
        self.get_dists(start_pose)

        # Evaluate
        plane_th = 0.1
        hsd_th = 0.01
        if self.action == "circle":
            length_th = self.r[0]*0.125
            hsd_th = self.r[0]*0.1
            self.eval_string += (f"<p style='color:{'green' if self.plane_dist < plane_th else 'red'}'>Distance from plane with normal {self.dir_vector}: {self.plane_dist}</p>"
                                 f"<p style='color:{'green' if self.length_diff < length_th else 'red'}'>STD of circle radius: {self.length_diff}</p>"
                                 f"<p style='color:{'green' if self.hsd < hsd_th else 'red'}'>Mean difference from radius: {self.hsd}</p>")

        elif self.action == "line":
            length_th = 0.01
            self.eval_string += (f"<p style='color:{'green' if self.plane_dist < plane_th else 'red'}'>Angle from {self.vector} vector: {self.plane_dist}</p>"
                                 f"<p style='color:{'green' if self.length_diff < length_th else 'red'}'>Difference in length: {self.length_diff}</p>"
                                 f"<p style='color:{'green' if self.hsd < hsd_th else 'red'}'>Mean difference from line: {self.hsd}</p>")

        if self.length_diff < length_th and self.hsd < hsd_th and self.plane_dist < plane_th:
            self.score = 1
            self.eval_string += "<p style='color:green'>Test passed: +1 point</p>"

        self.visualize()
        self.eval_string += f'<p><img src="test_{self.test_id}.png"></p>\n'

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
    Main test function

    :return: 0 if successful, 1 if cannot load function move from exercise_2.py, 2 if error in move function, 3 if error in evaluation
    :rtype: int
    """
    # create results directory if not exists
    results_path = os.path.join(os.path.realpath(os.path.dirname(os.path.basename(__file__))), "results")
    if not os.path.exists(results_path):
        os.makedirs(results_path)

    try:
        from exercise_2 import move
    except Exception as e:
        tr = str(traceback.format_exc())
        print("\x1b[31;20m" + "Cannot load function move from exercise_2.py")
        msg = (f"<p style='color:red'>Cannot load function move from exercise_2.py</p>"
               f"<p style='color:red'>{tr}</p>")

        write_results(results_path, msg)
        return 1

    # prepare possible tests, axes and actions
    tests = 10
    possible_actions = np.array(["line", "circle"])
    possible_axis = np.arange(3)
    possible_r = np.hstack((np.linspace(-0.04, -0.075, 8), np.linspace(0.04, 0.075, 8)))
    actions = possible_actions[np.random.randint(0, possible_actions.size, tests)]

    # Test
    score = 0
    msg = ""
    for test_id, action in enumerate(actions):
        client = pyCub(config="exercise_2.yaml")

        # shuffle axes and rs
        np.random.shuffle(possible_axis)
        np.random.shuffle(possible_r)

        # prepare axis and r
        if action == "line":
            num_axes = np.random.randint(1, 3)
            axis = possible_axis[:num_axes]
            r = possible_r[:num_axes]

        elif action == "circle":
            axis = [possible_axis[0]]
            r = [np.abs(possible_r[0])]
            if axis[0] in [1, 2]:
                r[0] = np.min([r[0], 0.04])

        # Init evaluator with the client, action, axis, r, test_id and results_path
        e = Evaluator(client, action, axis, r, test_id, results_path)

        # Call exercise function
        try:
            start_pose, end_pose = move(client, copy.deepcopy(action), copy.deepcopy(axis), copy.deepcopy(r))
        except Exception as e:
            tr = str(traceback.format_exc())
            client.logger.error(tr)
            msg = f"<p style='color:red'>{tr}</p>"
            write_results(results_path, msg)
            return 2

        # Evaluate
        try:
            e.evaluate(start_pose, end_pose)
        except Exception as e:
            tr = str(traceback.format_exc())
            client.logger.error(tr)
            msg = f"<p style='color:red'>{tr}</p>"
            write_results(results_path, msg)
            return 3

        msg += e.eval_string

        score += e.score

        del e, client

    # Print final results
    print('\x1b[97;10m' + f"Your score is {score} out of {tests}")
    msg += f"<h1>Your score is {score} out of {tests}</h1>"

    write_results(results_path, msg)

if __name__ == "__main__":
    main()
