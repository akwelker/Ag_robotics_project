'''
Adam Welker     Motion Planning     Spring 25

Path_Sticher.py -- This class is able to take a list of polygons in
the order that they should be visited, and can return a path the visits the entirety
of the list. It will not check for collisions between the polygons. It only
"Stiches" seed spreading paths between a list of polygons.
'''

import numpy as np
import matplotlib.pyplot as plt

from SeedPlanner import SeedPlanner


class PathSticher:

    seed_patches = []
    starts = []
    end = []



    def __init__(self, seed_patches:list = None, centroids:np.ndarray = None)-> None:
        '''
        Class Constructor. Loads the seed patches into the class

        args:
            seed_patches: list of Seed Planner objects. 

        Returns:
            None
        '''
        
        if seed_patches is not None:

            self.seed_patches = seed_patches

        if centroids is not None:

            self.centroids = centroids


    def load_seed_patches(self, seed_patches:list)-> None:
        """
        Load the seed patches into the class
        """

        self.seed_patches = seed_patches

    def load_centroids(self, centroids:list)-> None:
        """
        Load the centroids into the class
        """

        self.centroids = centroids
        


    def get_quilted_path(self, start_point:np.ndarray)->np.ndarray:
        """
        Gets the stitched path between the polygons. Also saves the path
        as a variable in self

        args: none

        returns: path -- an numpy array in the form [(x0,y0) ..... (xn, yn)]
        """

        quilted_path = np.copy(start_point) # start with the first point

        # begin with the first patch
        quilted_path = np.vstack(self.seed_patches[0].get_seed_path(start_point,self.centroids[0]))

        # loop through the rest of the patches

        for i in range(1, len(self.seed_patches)):

            # get the path from the last point to the next patch
            path = self.seed_patches[i].get_seed_path(quilted_path[-1],self.centroids[i])
            
            # add the path to the quilted path
            quilted_path = np.vstack((quilted_path, path))

        
        self.quilted_path = quilted_path

        return quilted_path

        

    def plot_path(self, ax = None, show:bool=False)-> plt.axes:
        """
        Plot the stitched path.

        args: - show: A boolean indicated whether we should show the plot 
                     as the method finishes

        returns: ax -- the axis with the path mapped out
        """

        if ax is None:

            fig, ax = plt.subplots()
            ax.set_aspect('equal')
        
        # Plot all the polygons

        for i in range(len(self.seed_patches)):
            
            patch = self.seed_patches[i]
            ax.plot(patch.vertices[:,0],patch.vertices[:,1], 'k-o')
            
            #Close the polygon plot
            length = len(patch.vertices)
            last_side_x = [patch.vertices[length-1,0], patch.vertices[0,0]]
            last_side_y = [patch.vertices[length-1,1], patch.vertices[0,1]]
            ax.plot(last_side_x, last_side_y, 'k-o')

        # Plot the path

        ax.plot(self.quilted_path[:,0], self.quilted_path[:,1], 'r-o')
        ax.set_title("Stitched Path")

        ax.set_xlabel("X")
        ax.set_ylabel("Y")


        if show:
            plt.show()

        return ax


            


##########################
#------- Main Method------
##########################

if __name__ == '__main__':

    octagon_points = [(0.0, 4.3),
                (2.9, 3.7),
                (4.5, 1.2),
                (4.1, -1.9),
                (2.6, -4.1),
                (0.0, -4.4),
                (-2.8, -3.4),
                (-4.2, 1.0)]
    
    octagon_points = np.array(octagon_points)


    triangle_points = [(7.997, -3.074),  # mirrored point 1
                        (7.397, 0.026),    # mirrored point 2
                        (10, 10)]
    
    triangle_points = np.array(triangle_points)

    square = [
            (4.5, 1.2),  # shared point 1
            (4.1, -1.9), # shared point 2
            (7.997, -3.074),  # mirrored point 1
            (7.397, 0.026)    # mirrored point 2
            ]
    
    square = np.array(square)
    

    Octagon_centroid = (0.39, -0.17)

    Triangle_centroid = [np.mean(triangle_points[:,0]), np.mean(triangle_points[:,1])]

    Square_centroid = (2.75, -0.55)

    centroids = np.array([Octagon_centroid, Square_centroid,Triangle_centroid])

    octogon_patch = SeedPlanner()
    octogon_patch.load_polygon(octagon_points)

    triangle_patch = SeedPlanner()
    triangle_patch.load_polygon(triangle_points)

    square_patch = SeedPlanner()
    square_patch.load_polygon(square)


    patches = [octogon_patch, square_patch, triangle_patch]

    sewer = PathSticher(patches, centroids)

    sewer.get_quilted_path(np.array([-10,0]))

    sewer.plot_path()

    plt.show()
