'''
Adam Welker   Ag Robotics Project     Spring 25

path_following_sim.py -- Runs a basic sim to show the ability of the robot 
class to simulate a field robot
'''

import numpy as np
from matplotlib import pyplot as plt
from sys import path
from SeedPlanner import SeedPlanner
from robot import Robot


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

seed_planner = SeedPlanner()
seed_planner.load_polygon(octagon_points)

start_point = np.array([-10, 0])
end_point = np.array([10, 0])

waypoint_path = seed_planner.get_seed_path(start_point, end_point)

waypoint_path = np.vstack([np.array(start_point), waypoint_path])
waypoint_path = np.vstack([np.array(waypoint_path), end_point])


ax = seed_planner.print_path(path=waypoint_path, show=False)

robot_init_state = np.append(start_point, [np.pi/2,0,0])
k_distance = 0.5
k_path = 2.0
k_angle = 5.0


tractor = Robot(robot_init_state,waypoint_path,k_angle, k_distance, k_path)

dt = 0.001

t_span = np.arange(0,1000,dt)

robot_locations = np.empty((len(t_span) + 1, 2))
robot_locations[:] = np.NaN

robot_locations[0] = robot_init_state[0:2]

for i in range(0,len(t_span)):

    tractor.update_control()
    tractor.update_state(dt)

    robot_locations[i+1] = tractor.state[0:2]

    if tractor.path_index >= len(tractor.path):

        break

ax.plot(robot_locations[:,0], robot_locations[:,1], 'b')
plt.show()