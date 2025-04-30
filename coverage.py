import numpy as np
from matplotlib import pyplot as plt
from matplotlib.lines import Line2D
from cell_decomposition.ecd import PolygonEnvironment
from seed_spreading.SeedPlanner import SeedPlanner
from seed_spreading.Path_Sticher import PathSticher
from seed_spreading.robot import Robot
from seed_spreading.DynamicObstacle import DynamicObstacle
from tqdm import tqdm
import TSP


def draw_cells(cell_bounds, centroid, ax=None) -> plt.axes:
    """
    Draws the cells and centroids on the given axis.
    """

    if ax is None:
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots()

    for i in range(0, len(cell_bounds)):

        cell = np.array(cell_bounds[i])
        ax.plot(cell[:, 0], cell[:, 1], "k-")
        # Draw the last side of the cell
        x_last_side = [cell[-1][0], cell[0][0]]
        y_last_side = [cell[-1][1], cell[0][1]]
        ax.plot(x_last_side, y_last_side, "k-")

        # fill the cell and add the centroid
        ax.fill(cell[:, 0], cell[:, 1], alpha=0.2)
        ax.plot(centroid[i][0], centroid[i][1], "ro")

        # label the cell number
        ax.text(
            centroid[i][0],
            centroid[i][1],
            str(i),
            fontsize=12,
            ha="center",
            va="center",
        )

    return ax


if __name__ == "__main__":

    # ===========================================================================#
    # Get environment
    # ===========================================================================#
    env_file = "./cell_decomposition/env1.txt"

    # Build polygon environment for cell decompostion
    env = PolygonEnvironment()
    env.read_env(env_file)

    # ===========================================================================#
    # Create the cell decomposition
    # ===========================================================================#
    env.trapezoidal_decomposition()
    cell_bounds, centroids = env.get_cells()

    ax1 = draw_cells(cell_bounds, centroids)

    ax1.set_title("Exact Cell Decomposition")
    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.set_aspect("equal", adjustable="box")
    plt.savefig("figures/cell_decomposition.png")

    # ===========================================================================#
    # TODO: Traveling Salesman
    # ===========================================================================#
    traveling = TSP.TSP(env, cell_bounds, centroids)

    cell_bounds = traveling.get_ordered_polygons()
    centroids = traveling.get_ordered_centroids()

    # ===========================================================================#
    # Create the stiched seed spreading path
    # ===========================================================================#

    seed_patches = []

    for cell in cell_bounds:
        seed_planner = SeedPlanner()
        seed_planner.dx = 5
        seed_planner.dy = 5
        seed_planner.load_polygon(cell)
        seed_patches.append(seed_planner)

    sticher = PathSticher(seed_patches, centroids)
    waypoint_path = sticher.get_quilted_path([-100, -50])
    ax3 = sticher.plot_path()
    ax3.set_aspect("equal", adjustable="box")

    ax3.set_title("Cell Decomposition Populated with Seed Spreading Path")
    plt.savefig("figures/seed_spreading_path.png")

    # ==========================================================================#
    # Run the simulation of the robot
    # ==========================================================================#

    dt = 0.01
    t_span = np.arange(0, 50000, dt)
    start_point = np.array([-100, -50])

    
    R = 25
    X0 = 0
    y0 = 0
    w = 0.25 * 2*np.pi

    dyn_obs_1_path = [R*np.cos(w*t_span)  + X0, R*np.sin(w*t_span)  + y0]

    robot_init_state = np.append(start_point, [np.pi / 2, 0, 0])
    k_distance = 0.5
    k_path = 2.0
    k_angle = 5.0

    tractor = Robot(robot_init_state, waypoint_path, k_angle, k_distance, k_path)

    # Add obstacle:
    tractor.init_add_dynamic_obstacle([10, 10], 0, dyn_obs_1_path)



    robot_locations = np.empty((len(t_span) + 1, 2))
    robot_locations[:] = np.NaN

    robot_locations[0] = robot_init_state[0:2]


    print("--- RUNNING SIMULATION ---")
    for i in tqdm(range(0, len(t_span))):

        tractor.update_control()
        tractor.update_state(dt)

        robot_locations[i + 1] = tractor.state[0:2]

        if tractor.path_index >= len(tractor.path):
            # print(tractor.t_current)
            break                           

    ax3.plot(robot_locations[:, 0], robot_locations[:, 1], "b")
    # ax3.plot(dyn_obs_1_path[0][0:i], dyn_obs_1_path[1][0:i], color="orange")

    ax3.set_title("Robot Path")
    plt.savefig("figures/robot_path.png")

    lines = [
        Line2D([0], [0], color="k", lw=2),
        Line2D([0], [0], color="r", lw=2),
        Line2D([0], [0], color="b", lw=2),
        Line2D([0], [0], color="orange", lw=2),
    ]

    labels = [
        "Cell Boundaries",
        "Robot Waypoint Path",
        "Robot Path",
        "Dynamic Obstacle Path",
    ]
    ax3.legend(lines, labels, loc="upper right")

    ax3.set_xlabel("X")
    ax3.set_ylabel("Y")

    plt.show()
