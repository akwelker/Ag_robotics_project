'''
Carlos Carvajal     Motion Planning        Spring 25

TSP.py: File to identify path by brute forcing traveling salesman problem
'''

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import numpy as np
import heapq
import matplotlib.pyplot as plt
import itertools
import seed_spreading.SeedPlanner as sp
import cell_decomposition.ecd as ecd


class TSP:
    def __init__(self, env, polygon_bounds, centroid_bounds) -> None:
        '''
        Args:
            env: a PolygonEnvironment object containing cells and obstacles.
        '''
        
        # Extract cells and obstacles
        polygons = polygon_bounds 
        centroids = centroid_bounds
        self.cells = [
            {'polygon': poly, 'centroid': centroid}
            for poly, centroid in zip(polygons, centroids)
        ]
        self.n = len(self.cells)
        print(self.n)
        self.obstacles = env.polygons
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

    def greedy_tsp(self) -> list:
        '''Greedy TSP (nearest neighbor) for large n. Skips unreachable cells.'''
        visited = [0]
        remaining = set(range(1, self.n))
        current = 0

        while remaining:
            valid_neighbors = [j for j in remaining if not np.isinf(self.distance_matrix[current][j])]

            if not valid_neighbors:
                print(f"Warning: No reachable unvisited neighbor from {current}. Skipping unreachable cells: {remaining}")
                break  # Or continue with fallback logic

            next_node = min(valid_neighbors, key=lambda j: self.distance_matrix[current][j])
            visited.append(next_node)
            remaining.remove(next_node)
            current = next_node

        # Try to end at the last cell if it's reachable
        if self.n - 1 not in visited and not np.isinf(self.distance_matrix[current][self.n - 1]):
            visited.append(self.n - 1)

        return [self.cells[i] for i in visited]
    
    def relaxed_tsp(self) -> list:
        '''Greedy TSP with fallback that ensures every cell is visited, even if revisits are required.'''
        visited = set()
        path = []
        current = 0
        visited.add(current)
        path.append(current)

        while len(visited) < self.n:
            # Find shortest path to the *nearest unvisited* cell (even through visited ones)
            next_cell, sub_path = self.get_next_reachable_cell(current, visited)
            if next_cell is None:
                raise ValueError("No path found to reach all cells!")

            # Add intermediate steps to path (may include revisits)
            for node in sub_path[1:]:  # skip current node
                path.append(node)
                visited.add(node)
            current = path[-1]

        return [self.cells[i] for i in path]

    def get_next_reachable_cell(self, start: int, visited: set) -> tuple:
        '''Finds shortest path from start to nearest unvisited node using Dijkstra over distance matrix.'''
        dist = [np.inf] * self.n
        prev = [None] * self.n
        dist[start] = 0
        heap = [(0, start)]

        while heap:
            cost, u = heapq.heappop(heap)
            if u not in visited:
                # Reconstruct path
                path = []
                while u is not None:
                    path.insert(0, u)
                    u = prev[u]
                return path[-1], path  # target, full path

            for v in range(self.n):
                if np.isinf(self.distance_matrix[u][v]):
                    continue
                alt = dist[u] + self.distance_matrix[u][v]
                if alt < dist[v]:
                    dist[v] = alt
                    prev[v] = u
                    heapq.heappush(heap, (alt, v))

        return None, []  # no reachable unvisited cell


    def get_ordered_centroids(self) -> np.ndarray:
        '''Returns an array of centroids in optimal visiting order.'''
        ordered_cells = self.relaxed_tsp()
        return np.array([cell['centroid'] for cell in ordered_cells])

    def get_ordered_polygons(self) -> list:
        '''Returns list of polygons in optimal visiting order.'''
        ordered_cells = self.relaxed_tsp()
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

    print("ordered centroids: ",ordered_centroids)
    print("ordered polygons: ",ordered_polygons)