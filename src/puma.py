import numpy as np

from roboticstoolbox.robot.ELink import ELink
from roboticstoolbox.robot.ERobot import ERobot
from spatialmath.pose3d import SE3
from math import pi

from utility import distance


# Directory to robotics-toolbox-python's models for Swift
# Should be addjusted accordingly - find a way to do it automatically???
models_dir = "/home/MarekSzyd/Programs/robotics-toolbox-python/roboticstoolbox/models/URDF/xacro/"


class Puma(ERobot):
    def __init__(self, ee_radius, qlims):
        args = super().urdf_to_ets_args(
            models_dir + "puma560_description/urdf/puma560_robot.urdf.xacro"
        )

        super().__init__(
            args[0],
            name=args[1]
        )

        # Hardcoded value for now
        self.ee_radius = ee_radius

        self.set_limits(qlims)
        self.add_configurations()

        self.has_ee_approx = False

        self.solution = None
        self.trajectory = None
        self.ee_positions = None

    def set_limits(self, qlims):
        for ind in range(qlims.shape[1]):
            link: ELink = self[ind]
            link.qlim = qlims[:, ind]

    def add_configurations(self):
        # zero angles, L shaped pose
        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0]))

        # end-effector inside joint 2
        self.addconfiguration("qi", np.array([0, pi/2, -pi/2, 0, 0, 0]))

        # ready pose, arm up
        self.addconfiguration("qr", np.array([0, pi/2, pi/2, 0, 0, 0]))

        # straight and horizontal
        self.addconfiguration("qs", np.array([0, 0, -pi/2, 0, 0, 0]))

        # nominal table top picking pose
        self.addconfiguration("qn", np.array([0, pi/4, pi, 0, pi/4, 0]))

    def get_reach(self):
        orig_pose = self.q[:]

        self.q = self.qr
        straight_dist = distance(self.get_current_pos(), (0, 0, 0))

        self.q = self.qi
        efector_dist = distance(self.get_current_pos(), (0, 0, 0))

        self.q = orig_pose

        return straight_dist - efector_dist

    def get_start_dist_to_target(self, dest_pos):
        orig_pose = self.q[:]

        self.q = self.qi
        dist = distance(self.get_current_pos(), dest_pos)

        self.q = orig_pose

        return dist

    def get_current_pos(self):
        return self.fkine(self.q).t

    def get_current_angles(self, cur_pos):
        # Starting transformation matrix
        Te = self.fkine(self.q)
        # Transformation matrix for calculation purposes
        Tep = Te * SE3(*cur_pos)
        # Desired transformation matrix for end-effector
        eTep = Te.inv() * Tep

        qk = self.ikine_LMS(eTep, self.q, np.array([1, 1, 1, 0, 0, 0])).q

        return qk
