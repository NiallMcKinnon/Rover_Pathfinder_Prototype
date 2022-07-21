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

        self.resolution = 0.5
        self.robot_radius = 1.0
        self.tolerance = self.resolution + self.robot_radius

        self.num_obstacles = 10
        self.obstacles = []#[[5,5], [2.58,3.91]]
        self.obstacles = [[7.090018298267958, 3.58303278903386], [4.950333971280379, 6.3496316471921945], [3.4932379927713377, 6.305043760466324], [3.336887098609428, 7.125788927028509], [7.080406777335156, 4.12111008897571], [6.055975358355124, 4.052035177497239], [6.882448811882153, 7.216264672841174], [6.321502399514811, 3.5518295203883894], [5.507498943193545, 4.903870657366257], [4.8597129979170095, 7.703357581202944]]
        # self.obstacles = [[4.56277143841519, 6.667699043080962], [5.795157122053895, 2.2266715610914036], [4.856357952157622, 3.2503315547691107], [5.465485895648444, 7.451916521210541], [2.212781298012021, 2.8476637428841522], [3.336427722101196, 5.323132193819994], [3.686054353915985, 5.632400392863696], [4.24514545296904, 7.426497401085545], [4.0514123955522106, 3.5699823668543127], [6.543141911904349, 3.4335129359087153]]
        # self.obstacles = [[4.444548441185281, 7.548260541696738], [4.065820068482433, 3.0692701023699485], [5.151909249180091, 4.501030358054784], [2.3832845562724976, 6.831362859863418], [6.906803119497914, 5.947626831317375], [4.0396978927738125, 6.12964659058213], [3.101132429912618, 7.12226439932759], [4.7425041228198275, 3.381218888460464], [2.395438882481338, 2.2713272067281505], [3.0232766955023767, 5.829863178376327]]
        self.collision_point_placeholder = [None, None]
        self.hit_obstacle_placeholder = [None, None]
        self.new_point_placeholder = [None, None]

        # self.build_hazards(self.num_obstacles)
        self.plot_hazards(self.obstacles)

        self.figure.canvas.draw()
        
        

        # print(self.valid_segment(self.waypoints[0], self.waypoints[1], self.obstacles))
        path_finished = False
        i = 0
        while not path_finished:
            i += 1
            path_finished = self.build_path()
            print(i)
            if i > 5000:
                break

        # self.build_path()
        
        self.plot_path(self.waypoints, 'lime')
            
        # plt.plot(self.collision_point_placeholder[0], self.collision_point_placeholder[1], 'r*')
        # plt.plot(self.hit_obstacle_placeholder[0], self.hit_obstacle_placeholder[1], 'c*')
        print(self.obstacles)
        plt.axis("scaled")
        plt.show()


    def build_path(self):

        for idx in range(len(self.waypoints)):

            if idx < len(self.waypoints)-1:

                # Get current and next point from [points]:
                current_point = self.waypoints[idx]
                next_point = self.waypoints[idx+1]

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
                    # x = (step * math.sin(slope_angle)) + current_point[0]
                    # y = (step * math.cos(slope_angle)) + current_point[1]
                    step += 0.01

                    # Check if the point is too close to an object:
                    for object in self.obstacles:
                        if not self.valid_point([x, y], [object]):
                            # Adjust path
                            new_point = self.find_valid_point(object, slope_angle, (x, y))

                            if self.valid_segment(self.start_point, new_point) and idx > 0:

                                # self.plot_path([self.start_point, new_point], 'blue')
                                self.waypoints = [self.start_point, new_point, self.end_point]
                                
                            else:
                                self.waypoints.insert(idx+1, new_point)

                            if idx > 2:

                                for idx_2 in range(len(self.waypoints[:idx])-2):
                                    if self.valid_segment(self.waypoints[idx_2], new_point):# and idx < len(self.waypoints[:idx]):
                                        # self.plot_path([self.waypoints[idx_2], new_point], 'blue')
                                        self.waypoints = self.waypoints[:idx_2+1] + [new_point] + [self.end_point]

                            return False     
        return True


    def find_valid_point(self, center_point, slope_angle, intercept):

        inverse_slope_angle = -((2*math.pi) - slope_angle - (math.pi / 2))

        point_pos = []
        step_pos = self.tolerance
        i_pos = 0
        while True:
            x_pos = (step_pos * math.cos(inverse_slope_angle)) + center_point[0]
            y_pos = (step_pos * math.sin(inverse_slope_angle)) + center_point[1]

            step_pos += 0.01

            if self.valid_point([x_pos, y_pos], self.obstacles):
                point_pos = [x_pos, y_pos]
                break
            i_pos += 1
        
        # Iterate along line in negative direction:
        point_neg = []
        step_neg = -self.tolerance
        i_neg = 0
        while True:
            x_neg = (step_neg * math.cos(inverse_slope_angle)) + center_point[0]
            y_neg = (step_neg * math.sin(inverse_slope_angle)) + center_point[1]

            step_neg -= 0.01

            if self.valid_point([x_neg, y_neg], self.obstacles):
                point_neg = [x_neg, y_neg]
                break
            i_neg += 1

        # Return whichever point is closer to the collision point
        if math.dist(intercept, point_neg) < math.dist(intercept, point_pos):
            return point_neg
        else:
            return point_pos


    def valid_segment(self, start_point, end_point):

        # Find the angle of the line's slope:
        slope_angle = self.calc_slope_angle(start_point, end_point)

        # Find the distance of the line segment:
        distance = math.dist(start_point, end_point)

        # self.plot_path([start_point, end_point], 'black')


        # Iterate along the line:
        step = self.resolution
        while step < distance:
            
            # Calculate a point at the distance along the line:
            x = (step * math.sin(slope_angle)) + start_point[0]
            y = (step * math.cos(slope_angle)) + start_point[1]
            # plt.plot(x, y, 'y*')
            step += 0.01

            # If the point collides with an obstacle, return True
            if not self.valid_point([x, y], self.obstacles):
                
                # self.collision_point_placeholder[0] = x
                # self.collision_point_placeholder[1] = y
                # self.plot_path([end_point, start_point], 'black')
                return False

        return True


    def valid_point(self, point, obstacles):
        
        # Check if the point is too close to each known obstacle:
        for object in obstacles:
            distance = math.dist(point, object)

            # If too close, return True:
            if distance < self.tolerance:
                self.hit_obstacle_placeholder[0] = object[0]
                self.hit_obstacle_placeholder[1] = object[1]
                
                return False
        
        # If all obstacles have been checked, return False:
        return True
        
    
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