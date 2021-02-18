import numpy as np
import autograd.numpy as anp
from autograd import grad
from time import time

from puma import Puma
from potential import potential_fnc
from utility import approaching_local_minma, distance, oscillations_detection


resolution = 0.01
resolution_1 = 0.2


def plan_potential(spos: tuple, gpos: tuple, obstacles: tuple):
    # Join x, y, z coordinates into three seperate lists
    obs_x, obs_y, obs_z = list(zip(*obstacles))
    opos = (obs_x, obs_y, obs_z)

    path = []

    pot_grad = grad(potential_fnc)

    spos = anp.array(spos, dtype=float)
    gpos = anp.array(gpos, dtype=float)

    cpos = spos
    last_dist = -1
    while (cur_dist := distance(cpos, gpos)) > resolution_1:
        if abs(cur_dist - last_dist) < 1e-12:
            print("Oscillations detected!")
            break

        vec = pot_grad(cpos, gpos, obstacles)
        vec = [-coord for coord in vec]

        x, y, z = cpos
        dx, dy, dz = vec

        cpos = anp.array([
            x + dx * resolution_1,
            y + dy * resolution_1,
            z + dz * resolution_1
        ], dtype=float)

        path.append((cpos))
        last_dist = cur_dist

    # Join x, y, z coordinates of each point in path
    # thus creating 3 lists of x, y, z vectors
    path: tuple = tuple(zip(*path))

    return spos, path, gpos, opos


def plan_robot_path(robot: Puma, dest_pos: tuple, spheres: tuple):
    pot_grad = grad(potential_fnc)

    spos = anp.array(robot.get_current_pos(), dtype=float)
    dest_pos = anp.array(dest_pos, dtype=float)

    cpos = spos
    last_dist = -1
    last_positions = list()

    step_pos = np.array([spos])

    success = True

    start = time()
    while (cur_dist := distance(cpos, dest_pos)) > robot.ee_radius / 2:

        if time() - start >= 10:
            # Is done in one second or so, so 10 is a long time
            print(
                "Timeout for path planning! 10 seconds passed, breaking."
            )
            return step_pos, not success

        minima = approaching_local_minma(cur_dist, last_dist)
        if minima:
            print(
                f"Approaching local minima at {cpos}! Stopping calculations"
            )
            return step_pos, not success

        oscil = oscillations_detection(last_positions, cpos)
        if oscil:
            print(
                f"Detected oscillations at {cpos}! Stopping calculations"
            )
            return step_pos, not success

        vec = pot_grad(cpos, dest_pos, spheres)
        vec = [-coord for coord in vec]

        x, y, z = cpos
        dx, dy, dz = vec

        cpos = anp.array([
            x + dx * resolution,
            y + dy * resolution,
            z + dz * resolution
        ], dtype=float)

        step_pos = np.vstack((step_pos, cpos))
        last_dist = cur_dist

    return step_pos, success
