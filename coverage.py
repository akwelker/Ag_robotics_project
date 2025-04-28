import numpy as np
from cell_decomposition.ecd import PolygonEnvironment
from seed_spreading.SeedPlanner import SeedPlanner

if __name__ == "__main__":

    # Get environment
    env_file = "./cell_decomposition/env3.txt"

    # Build polygon environment for cell decompostion
    env = PolygonEnvironment()
    env.read_env(env_file)

    # Perform trapezoidal decompostion and then get cells and centroids
    env.trapezoidal_decomposition()
    cell_bounds, centroids = env.get_cells()

    # TODO: Traveling Salesman

    # TODO: Path stitcher

    # TODO: when creating seed planner, dx dy, should be set accordingly if cell width is too small
    # Example building seed planner for 1 cell:
    # cell = np.array(cell_bounds[0])
    # seed_planner = SeedPlanner()
    # seed_planner.dx = 1
    # seed_planner.dy = 1
    # seed_planner.load_polygon(cell)
    # seed_planner.get_seed_path((env.x_min, env.y_min), (env.x_min, env.y_max))
    # seed_planner.print_path(show=True)
