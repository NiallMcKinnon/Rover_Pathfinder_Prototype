import matplotlib.pyplot as plt
import random
import math

class PathPlanner:

    def __init__(self):
        
        # Matplotlib stuff:
        self.graph_size = 50
        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlim([0, self.graph_size])
        self.ax.set_ylim([0, self.graph_size])
        self.num_obstacles = 3 * self.graph_size#300
        plt.axis("scaled")
        
        # These are parameters that can change
        self.resolution = 0.1 # 0.05
        self.robot_radius = 1.5
        self.tolerance = self.robot_radius

        # Get current and goal positions
        self.current_point = [0, 0] # Will come from odometry
        self.goal_point = [self.graph_size, self.graph_size] # Will come from goal path

        # On the rover, obstacles will come from the laser scan points
        self.obstacles = []
        self.build_hazards(self.num_obstacles)
        self.plot_hazards(self.obstacles)

        self.move_invalid_goal()
        self.waypoints = [self.current_point, self.goal_point]
        
        self.iterations = 0

        # Keep building path until it is free of hazards:
        path_finished = False
        while not path_finished:

            self.iterations += 1

            self.resolution = self.resolution * (1 + (self.iterations / 5000))

            path_finished = self.build_path()

            print(self.iterations, self.resolution)

            if self.iterations > 5000:
                break
        
        self.plot_path(self.waypoints, 'lime')
            
        # print(self.obstacles)
        plt.show()


    def calc_distance(self, start_point, end_point):

        # Return the distance between two points:
        return math.sqrt((end_point[0] - start_point[0])**2 + (end_point[1] - start_point[1])**2)


    def build_path(self):

        for idx in range(len(self.waypoints)):

            if idx < len(self.waypoints)-1:

                # Get current and next point from [points]:
                current_point = self.waypoints[idx]
                next_point = self.waypoints[idx+1]

                # Calculate slope angle of the line:
                slope_angle = self.calc_slope_angle(current_point, next_point)

                # Calculate distance between points:
                distance = self.calc_distance(current_point, next_point)

                # Step through the distance between the two points:
                step = self.resolution
                while step < distance:

                    # Calculate x and y at each distance along the line segment:
                    x = (step * math.cos(slope_angle)) + current_point[0]
                    y = (step * math.sin(slope_angle)) + current_point[1]

                    step += self.resolution

                    # Check if the point is too close to an object:
                    for object in self.obstacles:

                        if not self.valid_point([x, y], [object]):

                            # Find the closest point that is valid:
                            new_point = self.find_valid_point(object, slope_angle, (x, y))

                            # If there is a direct path from the new point to the current point, skip all points in-between:
                            if self.valid_segment(self.current_point, new_point) and idx > 0:

                                self.waypoints = [self.current_point, new_point, self.goal_point]
                                # return True # This MIGHT improve the program. Rover testing needed.
                            
                            # Otherwise, insert the new point into the list:
                            else:
                                self.waypoints.insert(idx+1, new_point)

                            if idx > 2:
                                
                                # Check if there is a direct path to an earlier point (not start point), and skip points in-between if so:
                                for idx_2 in range(len(self.waypoints[:idx])-2):

                                    if self.valid_segment(self.waypoints[idx_2], new_point):

                                        self.waypoints = self.waypoints[:idx_2+1] + [new_point] + [self.goal_point]

                            # Return False if a point was added (path isn't done)
                            return False 
        
        # Return True if no point needed to be added (path is done)
        return True


    def find_valid_point(self, center_point, slope_angle, intercept):

        # Calculate the inverse of the existing path's slope (perpendicular line):
        inverse_slope_angle = -((2*math.pi) - slope_angle - (math.pi / 2))

        # Find the closest valid point along the perpendicular line going each direction
        point_pos = self.find_valid_point_helper(center_point, inverse_slope_angle, 1)
        point_neg = self.find_valid_point_helper(center_point, inverse_slope_angle, -1)

        # Return whichever point is closer to the collision point
        if math.dist(intercept, point_neg) < math.dist(intercept, point_pos):
            return point_neg
        else:
            return point_pos


    def move_invalid_goal(self):

        # Find the slope angle of the path:
        slope_angle = self.calc_slope_angle(self.current_point, self.goal_point)

        # If the goal position is in a hazard zone, move it towards the rover until it is safe:
        if not self.valid_point(self.goal_point, self.obstacles):

            new_goal = self.find_valid_point_helper(self.goal_point, slope_angle, -1)
            self.goal_point = new_goal


    def find_valid_point_helper(self, start_point, slope_angle, direction, extra=0): # extra may not be needed

        # Set the step to negative or positive depending on input"
        step = self.tolerance * direction
        
        while True:

            # Calculate x and y along line at a distance from the start point:
            x = (step * math.cos(slope_angle)) + start_point[0]
            y = (step * math.sin(slope_angle)) + start_point[1]

            # Increase distance for next iteration:
            step += (self.resolution * direction)

            # If the point is valid, return it (extra is if the user wants additional MoE, may be removed)
            if self.valid_point([x, y], self.obstacles):

                x = ((step+extra*direction) * math.cos(slope_angle)) + start_point[0]
                y = ((step+extra*direction) * math.sin(slope_angle)) + start_point[1]

                return [x, y]


    def valid_segment(self, start_point, end_point):

        # Find the angle of the line's slope:
        slope_angle = self.calc_slope_angle(start_point, end_point)

        # Find the distance of the line segment:
        distance = self.calc_distance(start_point, end_point)

        # Iterate along the line:
        step = self.resolution
        while step < distance:
            
            # Calculate a point at the distance along the line:
            x = (step * math.sin(slope_angle)) + start_point[0]
            y = (step * math.cos(slope_angle)) + start_point[1]
            step += self.resolution

            # If the point collides with an obstacle, return True
            if not self.valid_point([x, y], self.obstacles):
                
                return False

        return True


    def valid_point(self, point, obstacles):
        
        # Check if the point is too close to each known obstacle:
        for object in obstacles:
            
            if point[0]-self.tolerance <  object[0] < point[0]+self.tolerance:

                if point[1]-self.tolerance <  object[1] < point[1]+self.tolerance:

                    distance = math.dist(point, object)

                    # If too close, return True:
                    if distance < self.tolerance:
                        
                        return False
                    
                    # return False
        
        # If all obstacles have been checked, return False:
        return True
        
    
    def calc_slope_angle(self, point_1, point_2):

        # ORDER MATTERS!
        # return math.atan2((point_2[0] - point_1[0]), (point_2[1] - point_1[1]))
        return math.atan2((point_2[1] - point_1[1]), (point_2[0] - point_1[0]))

    
    def build_hazards(self, amount):
        
        for _ in range(amount):

            y = random.uniform(2.0, self.graph_size-2.0)
            x = random.uniform(2.0, self.graph_size-2.0)

            self.obstacles.append([x, y])

    
    def plot_hazards(self, obstacles):

        for object in obstacles:

            plt.plot(object[0], object[1], 'k.')

            cir = plt.Circle((object[0], object[1]), radius=self.tolerance, color='r', fill=False)
            self.ax.add_patch(cir)

    
    def plot_path(self, waypoints, color):

        if len(waypoints) > 1:

            x_ends = [waypoints[0][0], waypoints[-1][0]]
            y_ends = [waypoints[0][1], waypoints[-1][1]]
            plt.plot(x_ends, y_ends, color="grey", linestyle='--')

            for idx in range(len(waypoints)-1):

                x_values = [waypoints[idx][0], waypoints[idx+1][0]]
                y_values = [waypoints[idx][1], waypoints[idx+1][1]]
                plt.plot(x_values, y_values, color=color)


def main():

    PathPlanner()


main()
