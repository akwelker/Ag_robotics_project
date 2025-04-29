'''
Carlos Carvajal     Motion Planning        Spring 25

TSP.py: File to identify path by brute forcing traveling salesman problem
'''

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import numpy as np
import matplotlib.pyplot as plt
import itertools
import seed_spreading.SeedPlanner as sp
import cell_decomposition.ecd as ecd

_ENV = "./cell_decomposition/env3.txt"

class TSP:
    def __init__(self, env:ecd.PolygonEnvironment) -> None:
        '''
        Args:
            decomposition: An ExactCellDecomposition object containing cells and obstacles.
        '''
        self.env = env
        env.read_env(_ENV)
        env.trapezoidal_decomposition()
        
        # Extract cells and obstacles
        self.cells, self.centroids = env.get_cells()
        self.obstacles = env.obstacles
        self.n = len(self.cells)
        self.distance_matrix = np.full((self.n, self.n), np.inf)
        
        # Preprocess obstacle edges ONCE
        self.obstacle_edges = []
        for obstacle in self.obstacles:
            self.obstacle_edges += self.get_edges_from_polygon(obstacle)
        
        # Build connectivity graph
        self.build_distance_matrix()

    def build_distance_matrix(self) -> None:
        '''Builds a distance matrix based on free paths between centroids.'''
        for i in range(self.n):
            for j in range(self.n):
                if i == j:
                    continue

                start = self.cells[i]['centroid']
                end = self.cells[j]['centroid']

                if self.is_path_free(start, end):
                    self.distance_matrix[i][j] = np.linalg.norm(start - end)

    def is_path_free(self, start:np.ndarray, end:np.ndarray) -> bool:
        '''Checks if a straight path from start to end intersects any obstacles.'''
        path_edge = sp.Polygon_Edge(tuple(start), tuple(end))

        for edge in self.obstacle_edges:
            intersects, _ = path_edge.check_intersection(edge)
            if intersects:
                return False
        return True

    def get_edges_from_polygon(self, polygon:np.ndarray) -> list:
        '''Helper to generate edges from polygon vertices.'''
        edges = []
        for i in range(len(polygon)):
            start = polygon[i]
            end = polygon[(i+1)%len(polygon)]
            edges.append(sp.Polygon_Edge(tuple(start), tuple(end)))
        return edges

    def brute_force_tsp(self) -> list:
        '''Solves TSP brute force and returns ordered list of cells.'''

        best_order = None
        best_distance = np.inf

        # assume cell 0 = start, cell n-1 = end
        middle_indices = list(range(1, self.n-1))

        for perm in itertools.permutations(middle_indices):
            route = [0] + list(perm) + [self.n-1]

            total_dist = 0
            valid_route = True

            for i in range(len(route)-1):
                d = self.distance_matrix[route[i]][route[i+1]]
                if np.isinf(d):
                    valid_route = False
                    break
                total_dist += d

            if valid_route and total_dist < best_distance:
                best_distance = total_dist
                best_order = route

        if best_order is None:
            raise ValueError("No valid path found!")

        ordered_cells = [self.cells[i] for i in best_order]
        return ordered_cells

    def get_ordered_centroids(self) -> np.ndarray:
        '''Returns an array of centroids in optimal visiting order.'''
        ordered_cells = self.brute_force_tsp()
        return np.array([cell['centroid'] for cell in ordered_cells])

    def get_ordered_polygons(self) -> list:
        '''Returns list of polygons in optimal visiting order.'''
        ordered_cells = self.brute_force_tsp()
        return [cell['polygon'] for cell in ordered_cells]
    



if __name__ == "__main__":
    env = ecd.PolygonEnvironment()
    env.read_env("./cell_decomposition/env3.txt")
    # Create a Path
    planner = TSP(env)

    # Get the centroids in order
    ordered_centroids = planner.get_ordered_centroids()

    # Get the polygons in order
    ordered_polygons = planner.get_ordered_polygons()

    print(ordered_centroids, ordered_polygons)