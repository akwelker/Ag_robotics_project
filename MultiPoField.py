# Polygon Field Generator
# Class for layering potential fields

# -- Imports
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union, split, nearest_points


class Vector:
    def __init__(self, direction, magnitude) -> None:
        self.direction = direction / np.linalg.norm(direction)
        self.magnitude = magnitude


# Contains 2D grids where each cell contains a vector
# Organized in layers, where each layer is created from one polygon
# then you can add them together and export it as a final potential field
class PotentialFieldStack:

    # Retrieve the item at a given, descrete point:
    

    # Pass it a size to exist in. 
    def __init__(self, bounds) -> None:
        self.bounds = bounds
        self.layers = []
        pass


    def AddLayerFromPath(self, pathLayout):
        pass


    def AddLayerFromPoint(self, coordinates):
        pass


    # Adds a layer from a polygon:
    def AddLayerFromPolygon(self, polygon, ):
        # Define the grid for the potential field:
        x_grid = np.arange(self.bounds[0][0], self.bounds[0][1]+1)
        y_grid = np.arange(self.bounds[1][0], self.bounds[1][1]+1)

        # Create a mesh grid of for the potential field:
        xx, yy = np.meshgrid(x_grid, y_grid)    # xx looks like array of arrays, [ 0 0 0 0 0 0 ]
        vector_zero = Vector([0,0], 0)                  # Create a template vector of zeros to fill it
        potential_field = [] # Create the mesh grid of vectors
        for i in range(xx.shape[0]):
            potential_field.append([])
            for j in range(xx.shape[1]):
                potential_field[i].append(vector_zero)
                

        # Go through and generate the field:
        for i in range(xx.shape[0]):
            for j in range(xx.shape[1]):

                # Create the point on the field:
                point = Point(xx[i,j], yy[i,j])

                # Check to see if it's in the polygon, if so, make it zero
                if polygon.contains(point) : potential_field[i][j] = Vector([0,0], 0)
                else:
                    # Get thec losest point in the polygon to the point
                    nearest = nearest_points(polygon, point)[0]
                    # Get the distance and vector away from it:
                    distance = point.distance(nearest)

                    # Check if it's zero
                    if (distance == 0) : 
                        potential_field[i][j] = Vector([0,0], 0)
                        continue

                    # Get direciton and add vector
                    direction = [point.x - nearest.x, point.y - nearest.y]
                    potential_field[i][j] = Vector(direction, self.gain/distance)
                    

        self.layers.append(potential_field)

    # 
    def PlotPotentialField(self):
        if self.layers.count == 0 : return

        potential_field = self.layers[0]
        x = np.arange(self.bounds[0][0], self.bounds[0][1]+1)
        y = np.arange(self.bounds[1][0], self.bounds[1][1]+1)

        u = np.zeros((x.size, x.size), dtype=float)
        v = np.zeros((y.size, y.size), dtype=float)

        for i in range(x.size):
            for j in range(y.size):
                vector = potential_field[i][j]
                u[i, j] = vector.direction[0] * vector.magnitude
                v[i, j] = vector.direction[1] * vector.magnitude

        plt.quiver(x, y, u, v, angles='xy', scale_units='xy', scale=1, color='blue')
        plt.show()

    
    


if __name__ == "__main__":
    test_poly_points = [[5, -15], [25, 30], [5, 20]]
    test_poly = Polygon(test_poly_points)
    bounds = [[-100, 100], [-50, 150]]
    po = PotentialFieldStack(bounds, 1)
    po.AddLayerFromPolygon(test_poly)
    po.PlotPotentialField()