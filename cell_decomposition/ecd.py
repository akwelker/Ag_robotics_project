"""
Ethan Quinlan  Ag Robotics Project     Spring 25

ecd.py -- Performs exact cell decomposition for a given environment.
Implemented trapezoidal decomp from https://github.com/tjdwill/TrapezoidalDecomposition/blob/main/trapezoidal_decomposition.py
"""

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString as ShapelyLineString, Polygon as ShapelyPolygon
from shapely.ops import unary_union, polygonize
from polygon import Point, Edge, Polygon
from os import path
from DynamicObstacle import DynamicObstacle

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
        self.dynobs = []

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

        """
        Next read in the dynamic obstacles, stored in separate files:
        DymanicObstacle: <width>, <height>, <start_time>
        DynamicObstacle: 5,9,562
        and then below that all the coordinates it visits:
        """
        for i in range(0, 20):
            # Read in obstacle file:
            obs_file = f"cell_decomposition/environment.txt_dynobs_{i}.txt"
            if (path.exists(obs_file)):
                actual_file = open(obs_file, "r")
                print(f"Found obstacle {i}, adding...")
                # Initialize data storage:
                dimensions = None
                start_time = None
                dynobs_path = []
                # Read the line & keep track of which line
                file_infos = actual_file.readlines()
                file_idx = 0
                for l in file_infos:
                    # Extract first line info:
                    if file_idx == 0:
                        raw_vals = l.strip().split()
                        raw_vals = raw_vals[1].split(",")
                        print(raw_vals)
                        dimensions = [raw_vals[0], raw_vals[1]]
                        start_time = raw_vals[2]
                    else:
                        dynobs_path.append([raw_vals[0], raw_vals[1]])
                    file_idx += 1

                this_obs = DynamicObstacle(dimensions, start_time, path)
                self.dynobs.append(this_obs)


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

    def trapezoidal_decomposition(self):
        """
        Divide env into vertical segments
        """
        vertical_lines = []
        obstacles = []

        # Get obstacles as Polygons
        for polygon in self.polygons:
            obstacles.append(Polygon(polygon))

        for obstacle in obstacles:
            # Filter out current obstacle
            filtered = [x for x in obstacles if x is not obstacle]

            for vertex in obstacle.vertices:
                # Get the rays that shoot up and down
                top_ray = Edge(vertex, Point(vertex.x, self.y_max))
                bottom_ray = Edge(vertex, Point(vertex.x, self.y_min))
                rays = (top_ray, bottom_ray)

                for i in range(len(rays)):
                    line = rays[i]
                    self_intersects = False

                    # Check self intersections
                    for edge in obstacle.edges:
                        intersection = line.intersects(edge)

                        if intersection is None or intersection == vertex:
                            continue
                        elif (i == 0) and (
                            obstacle.touches(intersection) and intersection.y > vertex.y
                        ):
                            self_intersects = True
                        elif (i == 1) and (
                            obstacle.touches(intersection) and intersection.y < vertex.y
                        ):
                            self_intersects = True

                    if self_intersects:
                        continue
                    else:
                        intersection_ys = []
                        intersection_found = False

                        # Check other obstacles
                        for other in filtered:
                            for edge in other.edges:
                                intersection = line.intersects(edge)

                                if (
                                    (intersection is None)
                                    or (not edge.on_edge(intersection))
                                    or (intersection == vertex)
                                    or (i == 0 and intersection.y < vertex.y)
                                    or (i == 1 and intersection.y > vertex.y)
                                ):
                                    continue
                                else:
                                    intersection_ys.append(intersection.y)
                                    intersection_found = True

                        if not intersection_found:
                            vertical_lines.append(line)
                        else:
                            if i == 0:
                                intersection_y = min(intersection_ys)
                            else:
                                intersection_y = max(intersection_ys)
                            vertical_lines.append(
                                Edge(vertex, Point(vertex.x, intersection_y))
                            )

        self.vertical_lines = vertical_lines
        return vertical_lines

    def get_cells(self):
        """
        Get cells and centroids from vertical lines and obstacles
        """
        # Convert environment bounds to workspace
        workspace = ShapelyPolygon(
            [
                (self.x_min, self.y_min),
                (self.x_max, self.y_min),
                (self.x_max, self.y_max),
                (self.x_min, self.y_max),
            ]
        )

        # Get workspace edges
        workspace_edges = []
        workspace_coords = list(workspace.exterior.coords)
        for i in range(len(workspace_coords) - 1):
            p = workspace_coords[i]
            q = workspace_coords[i + 1]
            workspace_edges.append(ShapelyLineString([p, q]))

        # Get obstacle edges
        obstacles = []
        obstacle_edges = []
        for obstacle in self.polygons:
            obstacle_coords = [(float(x), float(y)) for x, y in obstacle]
            obs_poly = ShapelyPolygon(obstacle_coords)
            obstacles.append(obs_poly)
            for i in range(len(obstacle_coords)):
                p = obstacle_coords[i]
                q = obstacle_coords[(i + 1) % len(obstacle_coords)]
                obstacle_edges.append(ShapelyLineString([p, q]))

        # Check for vertical lines
        if self.vertical_lines == None:
            print(
                "Environment needs vertical edges. Has cell trapezoidal_decomposition been performed?"
            )
            return

        # Get vertical line edges
        vertical_edges = []
        for line in self.vertical_lines:
            vertical_edges.append(
                ShapelyLineString([(line.p.x, line.p.y), (line.q.x, line.q.y)])
            )

        # Combine edges
        edges = workspace_edges + obstacle_edges + vertical_edges
        cells = list(polygonize(unary_union(edges)))

        # Get cell bounds and centroids
        cell_bounds = []
        centroids = []
        for cell in cells:
            # Don't include obstacles
            if any(obstacle.contains(cell.centroid) for obstacle in obstacles):
                continue

            # Get bounds
            coords = np.array(cell.exterior.coords[:-1])
            cell_bounds.append(coords)

            # Get ceontroid
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
    env.read_env("./cell_decomposition/env3.txt")
    env.trapezoidal_decomposition()
    env.get_cells()
    env.plot_cells()
