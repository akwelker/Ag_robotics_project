import numpy as np


class DynamicObstacle:

    # Internal Variables:
    speed = 0.1
    cuttoff_distance = 20

    def __init__(self, dimensions, t_start, positions) -> None:
        self.dimensions = dimensions
        self.t_start = t_start
        self.positions = positions

    # Evaluates the potential field produced by a given obstacle
    # at a given point in time
    def evaluate_potential(self, t_current, robot_state, distance_gain):
        
        # Return zero if obstacle hasn't appeared
        if (t_current < self.t_start) : return np.array([0, 0])

        # Get the current robot step:
        t_eval = (int)((t_current-self.t_start) * self.speed)

        # make sure the thing vanishesat the end:
        if (t_eval > len(self.positions)) : return np.array([0, 0])

        # Compute positions:
        robot_pos = [ robot_state[0], robot_state[1] ]
        obstacle_pos = [ self.positions[0][t_eval], self.positions[1][t_eval] ]

        # Return zero if outside radius
        if self.get_distance(robot_pos, obstacle_pos) > self.cuttoff_distance : return np.array([0, 0])

        # Find closest point:
        points = [
            [obstacle_pos[0], obstacle_pos[1]],
            [obstacle_pos[0] + self.dimensions[0], obstacle_pos[1]],
            [obstacle_pos[0], obstacle_pos[1] + self.dimensions[1]],
            [obstacle_pos[0] + self.dimensions[0], obstacle_pos[1] + self.dimensions[1]]
        ]

        # Get the closest point to here
        best_point = points[0]
        best_dist = self.get_distance(robot_pos, best_point)
        for point in points:
            this_dist = self.get_distance(robot_pos, point)
            if this_dist < best_dist:
                best_point = point
                best_dist = this_dist

        # Evaluate vector pointing away:
        unit_vector_away = self.normalize_vector(robot_pos, best_point)

        # Set magnitude based on function and return (currently k_dist / distance)
        return np.array(
            [unit_vector_away[0] * (distance_gain / best_dist), 
             unit_vector_away[1] * (distance_gain / best_dist)]
             )

    # Gets the distance, SIMPLY
    def get_distance(self, p1, p2):
        return np.sqrt(((p1[0]-p2[0])**2) + ((p1[1]-p2[1])**2))
    
    def normalize_vector(self, point1, point2):
        # Calculate the components of the vector
        x_component = point2[0] - point1[0]
        y_component = point2[1] - point1[1]
        
        # Calculate the magnitude of the vector
        magnitude = np.sqrt(x_component**2 + y_component**2)
        
        # Handle zero vector case
        if magnitude == 0:
            raise ValueError("Cannot normalize a zero vector.")
        
        # Normalize the components
        normalized_x = x_component / magnitude
        normalized_y = y_component / magnitude
        
        return [ normalized_x, normalized_y ]
