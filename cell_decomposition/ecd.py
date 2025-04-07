import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString
from shapely.ops import unary_union, split

_BOUNDS = "Bounds:"
_OBSTACLE = "Obstacle:"
_START = "Start:"
_GOAL = "Goal:"


class PolygonEnvironment:
    """
    A simple class to store polygon obstacle environments
    """

    def __init__(self):
        """
        Create the storage types needed for the class
        """
        self.polygons = []
        self.goal = None
        self.start = None
        self.line_parser = {
            _BOUNDS: self.parse_bounds,
            _OBSTACLE: self.parse_obstacle,
            _START: self.parse_start,
            _GOAL: self.parse_goal,
        }
        self.cells = None
        self.centroids = None

    def read_env(self, env_file_path):
        """
        Read in a map from file that has the form.
        It can read in lines of the four types listed which start which are of the form
        <typename>: vals ...
        The for options are:
        Bounds: x_min x_max y_min y_max
        Goal: goal_q1 goal_q2 ...
        Start: start_q_1 start_q_2 ...
        Obstacle: x1 y1 x2 y2 x3 y3 [x4 y5...xn yn]
        """
        env_file = open(env_file_path, "r")
        file_infos = env_file.readlines()
        for l in file_infos:
            line_info = l.strip().split()
            if line_info[0].startswith("#"):
                continue
            self.line_parser[line_info[0]](line_info[1:])

    def parse_bounds(self, line_data):
        """
        Parse map boundaries
        """
        self.x_min = float(line_data[0])
        self.x_max = float(line_data[1])
        self.y_min = float(line_data[2])
        self.y_max = float(line_data[3])
        self.lims = np.array([[self.x_min, self.x_max], [self.y_min, self.y_max]])

    def parse_obstacle(self, line_data):
        """
        Parse a polygon obstacle line
        """
        vals = [float(x) for x in line_data]
        pts = []
        # Parse pair of values into points for obstacle vertices
        while len(vals) > 0:
            pts.append(np.array(vals[:2]))
            vals = vals[2:]

        if len(pts) < 3:
            print("Need at least 3 points to define an obstacle")
            return
        obstacle = np.array(pts)
        self.polygons.append(obstacle)

    def parse_goal(self, line_data):
        """
        Parse a goal location
        """
        self.goal = np.array([float(l) for l in line_data])

    def parse_start(self, line_data):
        """
        Parse a start location
        """
        self.start = np.array([float(l) for l in line_data])

    def exact_cell_decomposition(self):
        """
        Decompose workspace into exact cells.
        """
        # Convert environment bounds to workspace
        workspace = Polygon(
            [
                (self.x_min, self.y_min),
                (self.x_max, self.y_min),
                (self.x_max, self.y_max),
                (self.x_min, self.y_max),
            ]
        )

        # Get obstacles as Polygons
        obstacles = []
        for obs in self.polygons:
            obstacles.append(Polygon(obs))

        # Begin with single cell of environment
        cells = [workspace.difference(unary_union(obstacles))]

        # Sort by x coords
        x_coords = {self.x_min, self.x_max}
        for obs in self.polygons:
            for coord in obs:
                x_coords.add(coord[0])
        x_coords = sorted(x_coords)

        # Split cells for each x value
        for x in x_coords:
            if x == self.x_min or x == self.x_max:
                continue
            vert_line = LineString([(x, self.y_min), (x, self.y_max)])
            new_cells = []
            for cell in cells:
                # Split cell on intersection
                if cell.intersects(vert_line):
                    new_split = split(cell, vert_line)
                    for geom in new_split.geoms:
                        new_cells.append(geom)
                else:
                    new_cells.append(cell)
            cells = new_cells

        # Get cell bounds and centroids
        cell_bounds = []
        centroids = []
        for cell in cells:
            coords = list(cell.exterior.coords)
            if coords[0] == coords[-1]:
                coords = coords[:-1]
            cell_bounds.append(np.array(coords))
            centroids.append(np.array([cell.centroid.x, cell.centroid.y]))

        self.cells = cell_bounds
        self.centroids = centroids
        return cell_bounds, centroids

    def plot_cells(self):
        """
        Plot the cell decomposition.
        """
        if self.cells is None or self.centroids is None:
            print("Environment needs cell. Has cell decomposition been performed?")
            return

        # Plot obstacles
        if self.polygons is not None:
            for obs in self.polygons:
                obs = np.vstack([obs, obs[0]])
                plt.plot(obs[:, 0], obs[:, 1], "r-", zorder=10)

        # Plot cells
        for cell in self.cells:
            cell = np.vstack([cell, cell[0]])
            plt.plot(cell[:, 0], cell[:, 1], "g-")

        # Plot centroids
        centroids = np.array(self.centroids)
        plt.plot(centroids[:, 0], centroids[:, 1], "bo")

        # Show plot
        plt.title("Exact Cell Decomposition")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.show()


if __name__ == "__main__":
    env = PolygonEnvironment()
    env.read_env("./cell_decomposition/env0.txt")
    env.exact_cell_decomposition()
    env.plot_cells()
