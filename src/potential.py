from typing import List

from utility import distance
from roboticstoolbox.robot.Shape import Sphere


rho = 0.1 # constant positive coefficient
xb = 0.15  # const. activation value for potential fnc
sigma = 0.15  # Strength of obstacles' potential field

rho1 = 5 # constant positive coefficient
xb1 = 3  # const. activation value for potential fnc
sigma1 = 5  # Strength of obstacles' potential field


def kappa(x):
    kap = 0

    if 0 < x < xb:
        kap = sigma * ((1 / x) - (1 / xb)) ** 2

    return kap


def beta(cpos, sphere: Sphere):
    return distance(cpos, sphere)


def potential_fnc(cpos, gpos, spheres: List[Sphere]):
    I = distance(cpos, gpos)
    for sphere in spheres:
        if isinstance(sphere, Sphere):
            sphere_pos = sphere.base.t
        else:
            sphere_pos = sphere

        I += rho * kappa(beta(cpos, sphere_pos))

    return I
