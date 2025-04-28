'''
Adam Welker     Ag Robotics Project     Spring 25

robot.py -- python infrastructure for a simple path following robot. The
robot is holonomic, 2-DOF, and can follow a set path (while avoiding obstacles).
'''

import numpy as np

class Robot:

    lin_max_speed = 1.0 # The speed of the robot (units/s)
    ang_max_speed = 1*np.pi # The angular speed of the robot (radians/s)

    path = None  # The path that the robot is following
    
    state = None # State of the robot (x, y, theta, velocity, theta_dot)
    k_steering = None # The turning gain for the robot
    k_distance = None # The distance gain for the robot
    k_path = None # Potential Field gain


    def __init__(self, init_state:np.ndarray, path:np.ndarray, 
                 k_steering:float, k_distance:float, k_path:float)-> None:
        '''
        Initialize the robot with a given state, path, and gains.

        args: init_state: The initial state of the robot (x, y, theta in radians)
              path: The path that the robot is following (a list of points)
              k_steering: The steering gain for the robot
                k_distance: The distance gain for the robot

        '''

        try:

            assert init_state.shape == (5,)
            assert path.shape[1] == 2
            assert isinstance(k_steering, float) and k_steering > 0
            assert isinstance(k_distance, float) and k_distance > 0
            assert isinstance(k_path, float) and k_path > 0

        except AssertionError:

            raise ValueError("Invalid input to robot constructor")
        
        self.state = init_state
        self.path = path
        self.k_steering = k_steering
        self.k_distance = k_distance
        self.k_path = k_path

        self.path_index = 0 # The index of the path that the robot is following

    def _construct_path_potential(self) -> np.ndarray:
        '''
        Constructs the potential field needed for the robot to follow the path
        '''
        if self.path_index >= len(self.path):

            return np.array([0,0])

        # Get the current path point
        path_point = self.path[self.path_index]
        
        previous_point = None

        if self.path_index > 0:
            previous_point = self.path[self.path_index - 1]
        else:
            previous_point = self.speed[0:2]

        # Find the distance the the previous point to the current state

        path_error = self.state[0:2] - previous_point

        # Now rotate that into the path frame

        q = path_point - previous_point

        Chi = np.arctan2(q[1], q[0]) - np.pi/2

        R_path = np.array([[np.cos(Chi), -np.sin(Chi)],
                        [np.sin(Chi), np.cos(Chi)]]).T
        
        e = R_path @ path_error

        # Now we can calculate the potential field
        path_pot_vector = np.array([-self.k_path * e.item(0), np.linalg.norm(q) - e.item(1)])

        # Now rotate that back into the world frame

        path_pot_vector = R_path.T @ path_pot_vector

        return path_pot_vector
    

    def update_control(self):
        '''
        Will determine the command speed of the robot based on the location of 
        the robot
        '''

        # Check if we've reached the current waypoint or last waypoint
        if self.path_index >= len(self.path):

            self.state[3] = 0
            self.state[4] = 0

            return

        if np.linalg.norm(self.state[0:2] - self.path[self.path_index]) < 0.1:
            self.path_index += 1

            # Check if we've reached the current waypoint or last waypoint
            if self.path_index >= len(self.path):

                self.state[3] = 0
                self.state[4] = 0

                return

        
        path_pot_vector = self._construct_path_potential()
        ## add the vector from the obstacles to the vector in path_pot_vect

        theta_d = np.arctan2(path_pot_vector[1], path_pot_vector[0])
        e_theta = theta_d - self.state[2]

        if e_theta > np.pi:

            over_shoot = e_theta - np.pi
            e_theta = -np.pi + over_shoot

        elif e_theta < -np.pi:

            over_shoot = e_theta + np.pi
            e_theta = np.pi + over_shoot


        e_dist  = np.linalg.norm(self.state[0:2] - self.path[self.path_index])

        v = self.k_distance * e_dist * np.cos(e_theta)
        v = max([0, v])

        w = self.k_steering * e_theta

        # Limit the speed of the robot to the max speed
        if np.abs(v) > self.lin_max_speed:
            v = np.sign(v) * self.lin_max_speed
        if np.abs(w) > self.ang_max_speed:
            w = np.sign(w) * self.ang_max_speed

        self.state[3] = v
        self.state[4] = w

        return


    def update_state(self, dt: float) -> None:
        '''
        Update the state of the robot using the control inputs and the time step

        args: dt: The time step to update the robot state by
        '''

        # Get the control inputs
        dx = self.state[3] * np.cos(self.state[2]) * dt
        dy = self.state[3] * np.sin(self.state[2]) * dt
        dtheta = self.state[4] * dt

        # Update the state of the robot
        self.state[0] += dx
        self.state[1] += dy
        self.state[2] += dtheta

        return