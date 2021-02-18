from matplotlib import pyplot as plt
import seaborn as sns

sns.set_style('darkgrid')


def plot_plan(start: tuple, path: tuple, goal: tuple,
              obstacles: tuple, s_g_size: int = 30,
              o_size: int = 15, p_size: int = 10,
              s_col='black', g_col='r', p_col='g', o_col='b',
              draw_line=False):
    """Plots start, goal, obstacles positions as well as
        path taken from start to goal point"""

    plt.figure()

    ax = plt.axes(projection='3d')
    ax.scatter(*start, s=s_g_size, c=s_col)
    ax.scatter(*path, s=p_size, c=p_col)
    ax.scatter(*goal, s=s_g_size, c=g_col)
    ax.scatter(*obstacles, s=o_size, c=o_col)

    if draw_line:
        ax.plot(*list(zip(start, goal)), c='cyan')

    plt.show()


def plot_positions(positions, joint_angles, time_units):
    joints = list(zip(*joint_angles))
    xyz = list(zip(*positions))

    start = positions[0]
    end = positions[-1]

    ax1 = plt.subplot(121, projection='3d')

    ax1.set_xlabel('X', fontsize=10)
    ax1.set_ylabel('Y', fontsize=10)
    ax1.set_zlabel('Z', fontsize=10)
    ax1.set_title('Pozycja końcówki manipulatora', fontsize=10)

    ax1.scatter(*start, s=25, c='black', label='Punkt początkowy')
    ax1.scatter(*end, s=25, c='r', label='Punkt końcowy')
    ax1.plot(*xyz, c='g')

    ax2 = plt.subplot(122)

    ax2.set_ylabel('Wartość kątowa [rad]', fontsize=10)
    ax2.set_xlabel('Czas [s]', fontsize=10)
    ax2.set_title('Wartości kątowe przegubów', fontsize=10)

    colors = [
        'maroon',
        'goldenrod',
        'royalblue',
        'olive',
        'teal',
        'navy'
    ]
    for ind, qk in enumerate(joints):
        ax2.plot(time_units, qk, label=f"q{ind}", c=colors[ind])
    
    ax1.legend(fontsize='large')
    ax2.legend(fontsize='large')

    plt.show()