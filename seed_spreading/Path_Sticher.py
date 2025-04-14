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
        quilted_path = np.hstack(self.seed_patches[0].get_path(start_point,self.centroids[0]))

        # loop through the rest of the patches

        for i in range(1, len(self.seed_patches)):

            # get the path from the last point to the next patch
            path = self.seed_patches[i].get_path(quilted_path[-1],self.centroids[i])
            
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

            for j in self.seed_patches[i].vertices:

                ax.plot(j[:,0], j[:,1], 'k-o')

        # Plot the path

        ax.plot(self.quilted_path[:,0], self.quilted_path[:,1], 'r-o')
        ax.set_title("Stitched Path")

        ax.set_xlabel("X")
        ax.set_ylabel("Y")


        if show:
            plt.show()

        return ax


            


