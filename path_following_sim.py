'''
Adam Welker   Ag Robotics Project     Spring 25

path_following_sim.py -- Runs a basic sim to show the ability of the robot 
class to simulate a field robot
'''

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Rectangle
from sys import path
from seed_spreading.SeedPlanner import SeedPlanner
from seed_spreading.Path_Sticher import PathSticher
from seed_spreading.robot import Robot
from seed_spreading.DynamicObstacle import DynamicObstacle



# Let's take the irregular octagonal path from the example in the seed spreader

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

waypoint_path = sewer.get_quilted_path(np.array([-10,0]))

start_point = [-10,0]
end_point = [10,0]   

waypoint_path = np.vstack([np.array(start_point), waypoint_path])
waypoint_path = np.vstack([np.array(waypoint_path), end_point])

dt = 0.001

t_span = np.arange(0,1000,dt)


# Dynamic Obstacle Creation:
A = 2
w = 0.25 * 2*np.pi
dyn_obs_1_path = [A*np.cos(w*t_span), A*np.sin(w*t_span)]

ax = sewer.plot_path()

robot_init_state = np.append(start_point, [np.pi/2,0,0])
k_distance = 0.5
k_path = 2.0
k_angle = 5.0


tractor = Robot(robot_init_state,waypoint_path,k_angle, k_distance, k_path)

# Add obstacle:
tractor.init_add_dynamic_obstacle([3,4], 350, dyn_obs_1_path)


robot_locations = np.empty((len(t_span) + 1, 2))
robot_locations[:] = np.NaN

robot_locations[0] = robot_init_state[0:2]

# Initialize the obstacle and the robot patches
robot_patch = Circle((robot_locations[0][0], robot_locations[0][1]), 0.2, color='blue')
ax.add_patch(robot_patch)
obstacle_patch = Circle((dyn_obs_1_path[0][0], dyn_obs_1_path[1][0]), 0.2, color='red')
ax.add_patch(obstacle_patch)


for i in range(0,len(t_span)):

    tractor.update_control()
    tractor.update_state(dt)

    robot_locations[i+1] = tractor.state[0:2]

    if tractor.path_index >= len(tractor.path):
        print(tractor.t_current)
        break

    
    if i % 50 == 0:

        # Update the robot and obstacle patches
        robot_patch.set_center((robot_locations[i+1][0], robot_locations[i+1][1]))

        if i < len(dyn_obs_1_path[0]):
            obstacle_patch.set_center((dyn_obs_1_path[0][i], dyn_obs_1_path[1][i]))
        
        # Update the plot
        plt.draw()
        plt.pause(0.001)

plt.show()