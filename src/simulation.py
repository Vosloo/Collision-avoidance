import numpy as np

from roboticstoolbox.tools.trajectory import mstraj

from env import Environment
from puma import Puma
from utility import create_spheres, distance
from path_planning import plan_robot_path
from plotting import plot_positions


destinations = [
    (0.6, 0.6, 0.6),
    (0.4, 0.3, 0.4),
    (0.2, 0.6, 0.1),
    (-0.3, 0.4, 0.1),
    (-0.3, -0.2, 0.1)
]


qlims = np.array([
    [0, -160, -110, -135, -266, -100, -266],
    [0, 160, 110, 135, 266, 100, 266]
])

# Holds simulation at the end
hold = True
# Plots xyz positions and joints' angle trajectories
plot_pos = True


if __name__ == "__main__":
    # Create robot - Puma560
    puma = Puma(ee_radius=0.08, qlims=qlims)

    print("Calculating trajectory for geometrical path")

    # Specific positions for testing purposes
    sphere_positions = [
        [*destinations[0], 0.008], # <--- target position spheres
        [*destinations[1], 0.008],
        [*destinations[2], 0.008],
        [*destinations[3], 0.008],
        [*destinations[4], 0.008],
        (0.52, 0.4, 0.3),
        (0.6, 0.4, 0.3),
        (0.5, 0.3, 0.45),
        (0.3, 0.3, 0.3),
        (0.3, 0.45, 0.25),
        (0.2, 0.55, 0.35),
        (-0.28, 0.1, 0.1),
        (-0.28, 0.1, 0.18)
    ]

    spheres = create_spheres(
        count=len(sphere_positions),
        positions=sphere_positions,
        random=False,
        rand_seed=8152215
    )
    # Seeds:
    # 8152215, 75125, 156126

    # Create Swift environment
    env = Environment()

    env.add_objects(puma, spheres)

    print("Starting simulation")
    for ind, dest in enumerate(destinations):

        print(
            f"Reach of robot: {puma.get_reach()}\n" +
            f"Distance to target: {puma.get_start_dist_to_target(dest)}"
        )

        if puma.get_reach() < puma.get_start_dist_to_target(dest):
            raise Exception("Destination position out of robot range! Aborting.")

        step_pos, plan_success = plan_robot_path(puma, dest, spheres[len(destinations):])

        qdmax = np.array([8] * step_pos.shape[1])
        trajectory = mstraj(step_pos, dt=0.5, tacc=0.5, qdmax=qdmax)

        success, joint_angles = env.step_sim(trajectory, len(destinations))

        if not success or not plan_success:
            print("Could not achieve desired position", end=' ')
            if not success:
                print("due to kinematics (should not happen)!")
            else:
                print("due to obstacle being too close to target location!")
        else:
            print("End-effector achieved desired position")
            print(dist := puma.get_current_pos(), distance(dist, dest))

        if plot_pos:
            plot_positions(trajectory.q, joint_angles, trajectory.t)

    if hold:
        env.hold()
