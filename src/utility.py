import numpy as np
import autograd.numpy as anp

from typing import List
from random import randint, seed, uniform

from roboticstoolbox.robot.Shape import Sphere
from spatialmath.pose3d import SE3

# The number of previous positions used to check oscillations
OSCILLATIONS_DETECTION_LENGTH = 10
DISTANCE_DIFF = 1e-6


def create_obstacles(count: int, coord_lim: tuple, dim: int = 3, rand_seed=None) -> tuple:
    """Creates randomly selected points in {dim} space
        and returns list of obstacles"""
    opos = []  # list of obstacles
    seed(rand_seed)
    for i in range(count):
        while True:
            x, y, z = [randint(*coord_lim) for _ in range(dim)]
            if (x, y, z) in opos:
                continue

            opos.append((x, y, z))
            break

    return tuple(opos)


def approaching_local_minma(cur_dist, last_dist) -> bool:
    if abs(cur_dist - last_dist) < DISTANCE_DIFF:
        return True
    else:
        return False


def oscillations_detection(last_positions, cpos) -> bool:
    """Checks for oscillations in movement - checks if
        point wasnt in the same position"""
    if len(last_positions) > OSCILLATIONS_DETECTION_LENGTH:
        last_positions.pop(0)

    if list(cpos) in last_positions:
        print("Oscillations detected!")
        return True
    else:
        last_positions.append(list(cpos))

    return False


def create_spheres(count: int, positions = [], radius=0.04, random=True, rand_seed=8888) -> List[Sphere]:
    seed(rand_seed)

    if random:
        for i in range(count):
            positions.append(
                [round(uniform(-0.65, 0.65), 2) for _ in range(2)] + # x, y coords
                [uniform(0.1, 0.65)] # z coord
            )
    elif len(positions) != count:
        raise Exception(
            "Number of spheres is not equal to list of their positions"
        )

    spheres = []
    for ind in range(count):
        s_pos = positions[ind]

        coords = s_pos
        s_radius = radius
        if len(s_pos) == 4:
            # Contains radius
            coords, s_radius = s_pos[:3], s_pos[3]

        coords = SE3(coords)
        spheres.append(Sphere(s_radius, coords))

    return spheres


def collided(robot, cur_pos: np.ndarray, sphere: Sphere):
    sphere_pos = sphere.base.t

    dist = distance(cur_pos, sphere_pos)

    if dist - robot.ee_radius - sphere.radius <= 0:
        return True
    else:
        return False


def distance(p1, p2) -> float:
    x1, y1, z1 = p1
    x2, y2, z2 = p2

    dist = anp.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)
    return dist
