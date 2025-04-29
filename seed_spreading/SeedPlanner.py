'''
Adam Welker     Motion Planning        Spring 25

Seed_Planner.py: Contains a Class that given a file that given a description
of a polygon, find a seed spreading path using approximate cell decomposition.
'''


import numpy as np
import matplotlib.pyplot as plt


class SeedPlanner:

    def __init__(self) -> None:
        '''
        Initialize the SeedPlanner class.
        
        '''
        self.dx = 0.5
        self.dy = 0.5

    def load_polygon(self, vertices:np.ndarray)-> None:
        '''
        Takes a list of verticies and comes up with the polygon object.
        The main thing this does is make a list of the edges of the polygon.

        args:
            vertices: A list of verticies that make up the polygon. The vertices
            should be in the order of the polygon (clockwise or counter-clockwise).
            They will be in the form [(x, y), (x, y), ...].

        returns:
            None
        '''

        self.vertices = vertices
        self.edges = []

        self.max_x = np.max(vertices[:, 0])
        self.min_x = np.min(vertices[:, 0])

        self.max_y = np.max(vertices[:, 1])
        self.min_y = np.min(vertices[:, 1])

        for i in range(len(vertices)):
            start = vertices[i]
            end = vertices[(i + 1) % len(vertices)]

            edge = Polygon_Edge(tuple(start), tuple(end))

            self.edges.append(edge)


    def check_collision(self, box_points:np.ndarray) -> bool:
        '''
        Check if the point is in collision with the polygon.

        This is done by making a polygon from the poitns in the box and checking
        if each edge of the polygon intersects with the edges of the polygon.

        args:
            box_points: A list of points that make up the box. The points should be
            in the order of the polygon (clockwise or counter-clockwise). They will
            be in the form [(x, y), (x, y), ...].
        
        returns:
            True if the box is in collision with the polygon, False otherwise.
        '''

        # Make a polygon from the points in the box
        box_poly = SeedPlanner()
        box_poly.load_polygon(box_points)

        box_edges = box_poly.edges

        # Check if each edge of the box intersects with the edges of the polygon
        for box_edge in box_edges:

            for polygon_edge in self.edges:
                intersects, intersection = polygon_edge.check_intersection(box_edge)

                if intersects:
                    return True
                
        return False
        
        


    def get_seed_path(self, start:np.array, end:np.array) -> np.ndarray:
        '''
        Get the seed spreader path for the polygon patch.
        '''


        # Find the direction of the start->end vector

        dir_vec = np.array(end) - np.array(start)

        compass_dir = self.get_compass_direction(dir_vec)

        path = []


        try:
            if compass_dir == "N" or compass_dir == "S":

                path1 = self.get_vert_path(start, end, mirrored=False)
                path2 = self.get_vert_path(end, start, mirrored=True)

                # path is the path that has a start point closer to the start point

                if np.linalg.norm(path1[0] - start) < np.linalg.norm(path2[0] - start):
                    path = path1
                else:
                    path = path2

            elif compass_dir == "E" or compass_dir == "W":
                    
                    path1 = self.get_horiz_path(start, end, mirrored=False)
                    path2 = self.get_horiz_path(end, start, mirrored=True)
        
                    # path is the path that has a start point closer to the start point
        
                    if np.linalg.norm(path1[0] - start) < np.linalg.norm(path2[0] - start):
                        path = path1
                    else:
                        path = path2

        except:
            if len(path) == 0:
                print("No path found")
                path = np.array([start, end])
        
        self.path = path
        return path

        


    def get_vert_path(self, start, goal, mirrored=False)-> np.ndarray:
        '''
        Get the seed spreader path for the polygon patch
        assuming that we plow vertically.

        args:
            start: The start point of the path (x, y). -- This isn't the actual
            start of the path, but rather where the robot enters the patch. It should
            be somewhat close the the start of the path. 
            
            goal: The goal point of the path (x, y). -- Not the end of path.
            See above.

        returns:
            A list of points that make up the path.
        '''

        path = []

        # Make an array of grid points that box the polygon
        y_points = np.arange(self.min_y - self.dy, self.max_y + self.dy, self.dy)
        x_points = np.arange(self.min_x - self.dx, self.max_x + self.dx, self.dx)

        grid_x, gird_y = np.meshgrid(x_points, y_points, indexing='ij')

        
        i_range = np.arange(len(x_points))
        j_range = np.arange(len(y_points))

        if not mirrored:
            i_range = np.flip(i_range)

        if start[1] > goal[1]:
            j_range = np.flip(j_range)

        # Loop through the grid points and check if they are in collision 
        # don't add them to the path


        for j in j_range:

            for i in i_range:
                x = grid_x[i][j]
                y = gird_y[i][j]

                if self.is_point_in_polygon((x, y)):

                    # Check if the point is in collision with the polygon
                    box_points = np.array([[x - self.dx/2, y - self.dy/2], 
                                            [x - self.dx/2, y + self.dy/2],
                                            [x + self.dx/2, y - self.dy/2],
                                            [x + self.dx/2, y + self.dy/2]])
                        
                    if not self.check_collision(box_points):
                        path.append((x, y))

            i_range = np.flip(i_range)

        # Now we have a list of points that are not in collision with the polygon.

        return np.array(path)


    def get_horiz_path(self, start, goal, mirrored = False)-> np.ndarray:

        '''
        Get the seed spreader path for the polygon patch
        assuming that we plow vertically.

        args:
            start: The start point of the path (x, y). -- This isn't the actual
            start of the path, but rather where the robot enters the patch. It should
            be somewhat close the the start of the path. 
            
            goal: The goal point of the path (x, y). -- Not the end of path.
            See above.

        returns:
            A list of points that make up the path.
        '''

        path = []

        # Make an array of grid points that box the polygon
        y_points = np.arange(self.min_y - self.dy/2, self.max_y + self.dy/2, self.dy)
        x_points = np.arange(self.min_x - self.dx/2, self.max_x + self.dx/2, self.dx)

        grid_x, gird_y = np.meshgrid(x_points, y_points, indexing='ij')

        
        i_range = np.arange(len(x_points))
        j_range = np.arange(len(y_points))

        if mirrored:
            j_range = np.flip(j_range)

        if start[0] > goal[0]:
            i_range = np.flip(i_range)

        # Loop through the grid points and check if they are in collision 
        # don't add them to the path


        for i in i_range:

            for j in j_range:
                x = grid_x[i][j]
                y = gird_y[i][j]

                if self.is_point_in_polygon((x, y)):

                    # Check if the point is in collision with the polygon
                    box_points = np.array([[x - self.dx/2, y - self.dy/2], 
                                        [x - self.dx/2, y + self.dy/2],
                                        [x + self.dx/2, y - self.dy/2],
                                        [x + self.dx/2, y + self.dy/2]])
                    
                    if not self.check_collision(box_points):
                        path.append((x, y))

            j_range = np.flip(j_range)

        # Now we have a list of points that are not in collision with the polygon.

        return np.array(path)
    
    def is_point_in_polygon(self, point:tuple) -> bool:
        '''
        Check if the point is in the polygon.

        args:
            point: The point to check (x, y).

        returns:
            True if the point is in the polygon, False otherwise.
        '''

        
        # Make and edge with the point and some near infinite point
        edge = Polygon_Edge(point, (1e6, 1e6))
        count = 0
        for polygon_edge in self.edges:
            intersects, intersection = polygon_edge.check_intersection(edge)

            if intersects:
                count += 1
                

        if count != 1:
            return False
        else:
            return True

        

    def print_path(self, path=None, show=False)->plt.axes:   
        '''
        Print the path to the screen.

        args:
            path, an iterable path. if not given use the self.path object.

        returns:
            None

        '''

        plot, ax = plt.subplots()

        for edge in self.edges:
           
            x = [edge.start[0], edge.end[0]]
            y = [edge.start[1], edge.end[1]]

            ax.plot(x, y, 'k-o')
        
        if path is None:

            path = self.path

        if len(path) > 0:

            ax.plot(path[:, 0], path[:, 1], 'r-o')
        
        if show == True:
            plt.show()

        return ax

        ### Auxilary Functions -- stuff to help with the math ###

    def get_compass_direction(self, vec)->str:
        """
        Gets the compass direction of a vector.

        args: vec -- a 2D vector

        returns: direction -- a string of the compass direction
        """
        
        vec_angle = np.arctan2(vec[1], vec[0])

        if vec_angle > np.pi/4 and vec_angle < 3*np.pi/4:
            return "N"
        elif np.abs(vec_angle) > 3*np.pi/4:
            return "W"
        elif vec_angle < -np.pi/4 and vec_angle > -3*np.pi/4:
            return "S"
        elif np.abs(vec_angle) < np.pi/4:
            return "E"
        else:
            raise ValueError("Vector is not a valid direction. Sei la cara")



class Polygon_Edge:
    '''
    Class that represents an Edge (or side) of a polygon.
    The class contains the start and end points. It also contains information
    about how the edge is represented as a linear function (slope, y-intercept, etc).
    the class also has functions to check if a line intersects with the edge.
    '''

    def __init__(self, start:tuple, end:tuple) -> None:
        '''
        Initialize the Polygon_Edge class.

        args:
            start: The start point of the edge (x, y).
            end: The end point of the edge (x, y).
        
        returns:
            None
        '''
        self.start = start
        self.end = end

        # Calculate the slope and y-intercept of the line
        if start[0] == end[0]:
            self.slope = None
            self.y_intercept = None

            self.x_intercept = start[0]
        else:
            self.slope = (end[1] - start[1]) / (end[0] - start[0])
            self.y_intercept = start[1] - self.slope * start[0]


    

        

    def check_intersection(self, line_edge) -> tuple:
        '''
        Finds out if the line intersects with the edge (self) object. This is done
        by checking if the lines intersect using Ax = B and some basic lin alg.
        Then, we check if the intersection point is withint the bounds of the edge.
        and the line.

        args:
            line_edge: A Polygon_Edge object that represents the line.

        returns:
            (True, (Ix, Iy)) if the line intersects with the edge, 
            (False, None) otherwise.
        '''

        try:

            # Turn the line into a polygon edge object        
            A = np.array([[-self.slope, 1], [-line_edge.slope, 1]])
            b = np.array([self.y_intercept, line_edge.y_intercept])
            intersection = np.linalg.solve(A, b)

                
            Ix = intersection.item(0)
            Iy = intersection.item(1)

        except Exception as e:

            Ix = None
            Iy = None

             
            if self.slope is None and line_edge.slope is None:

                # Both lines are vertical
                if self.x_intercept == line_edge.x_intercept:
                    # Lines are coincident

                    edge_y_min = min(self.start[1], self.end[1])
                    edge_y_max = max(self.start[1], self.end[1])
                    line_y_min = min(line_edge.start[1], line_edge.end[1])
                    line_y_max = max(line_edge.start[1], line_edge.end[1])

                    if edge_y_max > line_y_min:

                        Ix = self.x_intercept
                        Iy = self.edge_y_max

                    if line_y_max > edge_y_min:

                        Ix = self.x_intercept
                        Iy = line_y_max
        
            elif self.slope is None:

                # self is vertical, line_edge is not
                Ix = self.x_intercept
                Iy = line_edge.slope * Ix + line_edge.y_intercept
                
            elif line_edge.slope is None:

                # line_edge is vertical, self is not
                Ix = line_edge.x_intercept
                Iy = self.slope * Ix + self.y_intercept


            elif self.slope == line_edge.slope: # lines are parallel

                if self.y_intercept == line_edge.y_intercept:

                    min_x_edge = min(self.start[0], self.end[0])
                    max_x_edge = max(self.start[0], self.end[0])
                    min_x_line = min(line_edge.start[0], line_edge.end[0])
                    max_x_line = max(line_edge.start[0], line_edge.end[0])

                    if max_x_line > min_x_edge:

                        Ix = max_x_edge
                        Iy = self.y_intercept

                    if max_x_edge > min_x_line:
                            
                            Ix = max_x_line
                            Iy = line_edge.y_intercept

            if Ix is None or Iy is None:
                return (False, None)

        # Now check if the intersection point is within the bound of both
        # the edge and the line_edge

        edge_y_min = min(self.start[1], self.end[1])
        edge_y_max = max(self.start[1], self.end[1])

        edge_x_min = min(self.start[0], self.end[0])
        edge_x_max = max(self.start[0], self.end[0])

        line_y_min = min(line_edge.start[1], line_edge.end[1])
        line_y_max = max(line_edge.start[1], line_edge.end[1])

        line_x_min = min(line_edge.start[0], line_edge.end[0])
        line_x_max = max(line_edge.start[0], line_edge.end[0])

        # Check if the intersection point is within the bounds of the edge

        within_edge_x = edge_x_min <= Ix and Ix <= edge_x_max
        within_edge_y = edge_y_min <= Iy and Iy <= edge_y_max

        within_line_x = line_x_min <= Ix and Ix <= line_x_max
        within_line_y = line_y_min <= Iy and Iy <= line_y_max

        if within_edge_x and within_edge_y and within_line_x and within_line_y:
            return (True, (Ix, Iy))
        
        else:

            return (False, None)
        

    
        



#=========================#
#       Main Method
#=========================#
if __name__ == "__main__":

    polyon_points = [(142.5898, 0),
                     (142.2931, 0),
                     (142.2931, 16.35214409),
                     (142.5898, 16.25214)]
    
    polyon_points = np.array(polyon_points)
    seed_planner = SeedPlanner()
    seed_planner.dx = 0.1
    seed_planner.dy = 2
    seed_planner.load_polygon(polyon_points)
    seed_planner.get_seed_path((142.4, 0), (142.5, 2))
    seed_planner.print_path()
    
