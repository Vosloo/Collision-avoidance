from path_planning import plan_potential
from plotting import plot_plan
from utility import create_obstacles

show_graph = True


def plan():
    spos = (0.0, 15.0, 15.0)  # Start point
    gpos = (15.0, 0.0, 0.0)  # Goal point

    opos = create_obstacles(100, (0, 15), rand_seed=888)
    # doesnt work: 888
    # works: 888888

    # Path generation
    try:
        start, path, goal, obstacles = plan_potential(
            spos, gpos, opos
        )
    except Exception as exc:
        print(exc.args[0])
        print("Did not reach goal position!")
        return
    else:
        print("Goal reached!")

    if show_graph:
        print("Plotting graph, please stand by.")
        plot_plan(start, path, goal, obstacles, draw_line=True)


if __name__ == '__main__':
    print("Path planning started!")
    plan()
    print("Path planning ended!")
