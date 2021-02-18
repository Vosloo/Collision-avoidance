from time import sleep
from typing import List

from roboticstoolbox.backends.Swift.Swift import Swift
from roboticstoolbox.robot.ERobot import ERobot
from roboticstoolbox.robot.Shape import Shape, Sphere

from puma import Puma
from utility import collided


class Environment(Swift):
    def __init__(self, realtime=True, display=True):
        super().__init__(realtime=realtime, display=display)

        self.robot: Puma = None
        self.spheres: List[Sphere] = []
        self.launch()

    def add_objects(self, robot: ERobot, obstacles: Shape):
        if self.robot is not None:
            raise Exception("There already exists a robot in the environment")
        else:
            self.robot = robot

        self.add(robot, readonly=True)

        for ob in obstacles:
            if ob in self.spheres:
                raise Exception("Obstacle already in the environment")
            else:
                self.spheres.append(ob)

            self.add(ob)

    def hold(self, time=-1):
        if time <= 0:
            while True:
                sleep(1)
        else:
            sleep(time)

    def step_sim(self, trajectory, dest_spheres, check_collisions=True) -> bool:
        positions = trajectory.q
        joint_angles = []

        for _, cur_pos in enumerate(positions):
            if check_collisions:
                for sphere in self.spheres[dest_spheres:]:
                    if collided(self.robot, cur_pos, sphere):
                        print(f"Collision detected at {cur_pos}")
                        return False

            qk = self.robot.get_current_angles(cur_pos)
            joint_angles.append(qk)

            self.robot.q = qk
            self.step()

        return True, joint_angles
