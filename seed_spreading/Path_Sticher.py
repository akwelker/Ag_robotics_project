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



    def __init__(self, seed_patches:list = None)-> None:
        '''
        Class Constructor. Loads the seed patches into the class

        args:
            seed_patches: list of Seed Planner objects. 

        Returns:
            None
        '''
        
        if seed_patches is not None:

            self.seed_patches = seed_patches


    def load_seed_patches(self, seed_patches:list)-> None:
        """
        Load the seed patches into the class
        """

        self.seed_patches = seed_patches
        
        


    def get_quilted_path(self)->np.ndarray:
        """
        Gets the stitched path between the polygons. Also saves the path
        as a variable in self

        args: none

        returns: path -- an numpy array in the form [(x0,y0) ..... (xn, yn)]
        """
        pass

    def plot_path(self, show:bool=False)-> plt.axes:
        """
        Plot the stitched path.

        args: - show: A boolean indicated whether we should show the plot 
                     as the method finishes

        returns: ax -- the axis with the path mapped out
        """
        pass


    def find_starts_and_ends(self)-> bool:
        """
        Finds the start and end points for each seed spreader Pather

        args: none

        returns: success - whether the ideal start and e
                           end points could be determined

        """
        pass

