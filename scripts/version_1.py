import matplotlib.pyplot as plt
import random
import math

class PathPlanner:

    def __init__(self):

        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlim([0, 10])
        self.ax.set_ylim([0, 10])

        self.start_point = [0, 0]
        self.end_point = [10, 10]
        self.waypoints = [self.start_point, self.end_point]

        self.resolution = 0.1
        self.robot_radius = 1.0
        self.tolerance = self.resolution + self.robot_radius

        self.num_obstacles = 1
        self.obstacles = [[5,5]]

        self.collision_point_placeholder = [None, None]
        self.hit_obstacle_placeholder = [None, None]
        self.new_point_placeholder = [None, None]

        self.build_hazards(self.num_obstacles)
        self.plot_hazards(self.obstacles)

        self.figure.canvas.draw()
        
        

        # print(self.bool_segment_collision(self.waypoints[0], self.waypoints[1], self.obstacles))
        path_finished = False
        i = 0
        while not path_finished:
            i += 1
            path_finished = self.build_path(self.waypoints, self.obstacles)
            print(i)
            if i > 10000:
                break

        # self.build_path(self.waypoints, self.obstacles)
        
        self.plot_path(self.waypoints, 'lime')
            
        # plt.plot(self.collision_point_placeholder[0], self.collision_point_placeholder[1], 'r*')
        # plt.plot(self.hit_obstacle_placeholder[0], self.hit_obstacle_placeholder[1], 'c*')
        plt.show()


    def build_path(self, waypoints, obstacles):

        for idx in range(len(waypoints)):

            if idx < len(waypoints)-1:

                # Get current and next point from [points]:
                current_point = waypoints[idx]
                next_point = waypoints[idx+1]
                prev_point = waypoints[idx-1]

                # Calculate slope and slope angle of the line:
                delta_y = next_point[1] - current_point[1]
                delta_x = next_point[0] - current_point[0]
                slope_angle = math.atan2(delta_y, delta_x)

                # Calculate distance between points:
                distance = math.sqrt(delta_x**2 + delta_y**2)

                # Step through the distance between the two points:
                step = 0.01
                while step < distance:

                    # Calculate x and y at each distance along the line segment:
                    x = (step * math.cos(slope_angle)) + current_point[0]
                    y = (step * math.sin(slope_angle)) + current_point[1]
                    step += 0.01

                    # Check if the point is too close to an object:
                    for object in obstacles:
                        if self.bool_point_collision([x, y], [object]):
                            # Adjust path
                            new_point = self.find_valid_point(object, slope_angle, obstacles, (x, y))

                            if not self.bool_segment_collision(self.start_point, new_point, self.obstacles):
                                # print(True)
                                # self.plot_path([self.start_point, new_point], 'blue')
                                pass

                            self.waypoints.insert(idx+1, new_point)

                            return False     
        return True


    def find_valid_point(self, center_point, slope_angle, obstacles, intercept):

        inverse_slope_angle = -((2*math.pi) - slope_angle - (math.pi / 2))

        point_pos = []
        step_pos = 0.0
        i_pos = 0
        while True:
            x_pos = (step_pos * math.cos(inverse_slope_angle)) + center_point[0]
            y_pos = (step_pos * math.sin(inverse_slope_angle)) + center_point[1]

            step_pos += 0.01

            if not self.bool_point_collision([x_pos, y_pos], obstacles):
                point_pos = [x_pos, y_pos]
                break
            i_pos += 1
        
        # Iterate along line in negative direction:
        point_neg = []
        step_neg = 0.0
        i_neg = 0
        while True:
            x_neg = (step_neg * math.cos(inverse_slope_angle)) + center_point[0]
            y_neg = (step_neg * math.sin(inverse_slope_angle)) + center_point[1]

            step_neg -= 0.01

            if not self.bool_point_collision([x_neg, y_neg], obstacles):
                point_neg = [x_neg, y_neg]
                break
            i_neg += 1

        # Return whichever point is closer to the collision point
        if math.dist(intercept, point_neg) < math.dist(intercept, point_pos):
            return point_neg
        else:
            return point_pos


    def bool_segment_collision(self, start_point, end_point, obstacles):

        # Find the angle of the line's slope:
        slope_angle = self.calc_slope_angle(start_point, end_point)

        # Find the distance of the line segment:
        distance = abs(math.dist(start_point, end_point))

        # Iterate along the line:
        step = self.resolution
        while step < distance:
            
            # Calculate a point at the distance along the line:
            x = (step * math.cos(slope_angle)) + start_point[0]
            y = (step * math.sin(slope_angle)) + start_point[1]
            step += 0.01

            # If the point collides with an obstacle, return True
            if self.bool_point_collision([x, y], self.obstacles):
                # self.collision_point_placeholder[0] = x
                # self.collision_point_placeholder[1] = y
                self.plot_path([start_point, end_point], 'yellow')

                return True

        # If all points have beeen checked, return False:
        self.plot_path([start_point, end_point], 'blue')
        return False


    def bool_point_collision(self, point, obstacles):
        
        # Check if the point is too close to each known obstacle:
        for object in obstacles:
            distance = math.dist(point, object)

            # If too close, return True:
            if distance <= self.tolerance:
                self.hit_obstacle_placeholder[0] = object[0]
                self.hit_obstacle_placeholder[1] = object[1]
                return True
        
        # If all obstacles have been checked, return False:
        return False
        
    
    def calc_slope_angle(self, point_1, point_2):

        # ORDER MATTERS!
        return math.atan2((point_2[0] - point_1[0]), (point_2[1] - point_1[1]))

    
    def build_hazards(self, amount):
        
        for _ in range(amount):
            y = random.uniform(2.0, 8.0)
            x = random.uniform(2.0, 8.0)

            self.obstacles.append([x, y])

    
    def plot_hazards(self, obstacles):

        for object in obstacles:

            plt.plot(object[0], object[1], 'kx')

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