import cv2
import time
import numpy as np
import pyvisgraph as vg
from shapely.geometry import Polygon
from shapely.ops import unary_union

########## FUNCTIONS ##########

# Function to find the vector pointing outwards from a corner
def find_vector_farthest(corner, corners):    
    corners = corners.reshape(-1, 2)

    # Find the index of the corner in the list of corners
    corner_index = np.where((corners == corner).all(axis=1))[0][0]

    # Get the total number of corners
    num_corners = len(corners)

    # Get the indices of the previous and next corners in a circular manner
    prev_corner_index = (corner_index - 1) % num_corners
    next_corner_index = (corner_index + 1) % num_corners

    # Get the previous and next corners
    prev_corner = corners[prev_corner_index]
    current_corner = corners[corner_index]
    next_corner = corners[next_corner_index]
    
    # Calculate vectors from the current corner to the previous and next corners
    vector_prev = normal(np.float32(current_corner - prev_corner))
    vector_next = normal(np.float32(current_corner - next_corner))
    vector_farthest = normal(vector_prev + vector_next)

    return vector_farthest

# Function to return the normal of a vector
def normal(v):
    length = np.sqrt(v[0]**2+v[1]**2)
    v[0] = v[0]/float(length)
    v[1] = v[1]/float(length)
    return v

########## CLASSES ##########
class Robot:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

class Goal:
    def __init__(self, x, y):
        self.x = x
        self.y = y

# Main class used to interface with vision
class Vision:
    # Initialise cam, frame, copy of frame, robot, scale, graph,
    # obstacle vertices, shortest_path
    def __init__(self):
        self.cam = cv2.VideoCapture(0)
        valid, self.frame = self.cam.read()
        if not valid:
            print("Error reading frame.")
        self.copy = self.frame.copy()
        self.found_robot , self.robot, self.scale = self.find_robot()
        _, self.vertices, self.graph = self.find_graph()
        _, self.goal = self.find_goal()
        self.shortest_path = []

    def __del__(self):
        # Release the camera and close all windows
        self.cam.release()
        cv2.destroyAllWindows()
        # Delete the robot and goal objects
        del self.robot
        del self.goal

    # Function to update the frame, and the various entities if found 
    def update(self):

        valid, self.frame = self.cam.read()
        if not valid:
            print("Error reading frame.")
        self.copy = self.frame.copy()
        found_robot, robot, scale = self.find_robot()
        if found_robot:
            self.scale = scale
            self.robot = robot
        found_graph, vertices, graph = self.find_graph()
        if found_graph:
            self.vertices = vertices
            self.graph = graph
        found_goal, goal = self.find_goal()
        if found_goal:
            self.goal = goal
        if found_robot and found_goal and found_graph:
            self.shortest_path = self.find_shortest_path()

    # Show the processed image
    def show(self):
        cv2.imshow("Processed Frame", self.copy)
    
    #Function to make and draw the shortest path
    def find_shortest_path(self):
        shortest = self.graph.shortest_path(vg.Point(self.robot.x,self.robot.y),
                                            vg.Point(self.goal.x,self.goal.y))

        shortest_np = np.array([(point.x, point.y) for point in shortest], dtype=np.int32)
        # Draw shortest path
        for i in range(len(shortest_np)-1):
            cv2.line(self.copy, shortest_np[i], shortest_np[i+1], (10,10,10),1)
        return shortest_np

    # Function to find robot location and frame scale
    def find_robot(self):
        # Convert the image from BGR to RGB (OpenCV loads images in BGR by default)
        rgb_image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

        # Define the lower and upper bounds for the red color in RGB
        lower_red = np.array([130, 30, 80])
        upper_red = np.array([220, 100, 160])

        # Create a binary mask using inRange function
        red_mask = cv2.inRange(rgb_image, lower_red, upper_red)

        # Apply the binary mask to the original image
        result = cv2.bitwise_and(self.frame, self.frame, mask=red_mask)

        # Convert the result to grayscale
        gray_result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        gray_result = cv2.bilateralFilter(gray_result,5,15,15)
        contours, _ = cv2.findContours(gray_result, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        min_area = 10
        max_area = 10000
        centroid = [0,0]

        total = 0
        nose_size = 0
        # Draw the contours
        for i, contour_i in enumerate(contours):
            contour_area_i = cv2.contourArea(contour_i)
            if min_area < contour_area_i < max_area:
                total +=1
                nose_size = max(nose_size, contour_area_i)

        if total == 2:  #Here we have found the robot
            for _, contour_i in enumerate(contours):
                contour_area_i = cv2.contourArea(contour_i)

                if min_area < contour_area_i < max_area:
                    # Selecting the nose side of the robot with i = 0
                    if contour_area_i == nose_size:
                        i = 0
                    else:
                        i = 1
                    color = (0, 0, 255)
                    cv2.drawContours(self.copy, [contour_i], 0, color, 2)
                    M = cv2.moments(contour_i)
                    centroid[i] = np.array([int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])])
            cv2.line(self.copy, centroid[0], centroid[1], (0,0,255),2)

            # The real distance between the two markers on the robot is around 8.5 cm. so we set a scale
            # to know how distant the objects are from the camera.
            scale = np.linalg.norm(centroid[0] - centroid[1])/8.5

            return True, Robot((centroid[0][0] + centroid[1][0])/2.0, (centroid[0][1] + centroid[1][1])/2.0,
                        -np.arctan2(centroid[0][1]-centroid[1][1],centroid[0][0]-centroid[1][0])), scale
        else:
            return False, Robot(0,0,0), 0 # Default value if robot not found
    
    # Function to find the obstacles and make the visibility graph
    def find_graph(self):
        #Convert the frame to grayscale
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        # Apply some preprocessing
        gray = cv2.bilateralFilter(gray,5,15,15)
        # Apply a binary threshold to identify black pixels
        _, binary = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY)
        # Find contours in the binary image along with hierarchy
        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Set contour area thresholds
        min_area = 1000
        max_area = 100000

        # Approximation accuracy
        epsilon = 0.025

        # Store points and obstacles
        points = []
        obstacles = []
        found_obstacles = False

        for contour_i in contours:
            contour_area_i = cv2.contourArea(contour_i)

            if min_area < contour_area_i < max_area:
                found_obstacles = True
                # Draw the contours
                color = (0, 255, 150)
                cv2.drawContours(self.copy, [contour_i], 0, color, 2)

                # Find the corners of the contour
                corners = cv2.approxPolyDP(contour_i, epsilon * cv2.arcLength(contour_i, True), True)
                obstacle_i = []

                # Draw circles at each corner
                for corner in corners:
                    # Find the vector pointing outward from the corner
                    vector_farthest = find_vector_farthest(corner[0], corners)
                    # Place a point at a distance of 10 cm from the corner using the 
                    # SCALE we have found earlier from the robot
                    new_point = corner[0] + self.scale * 10 * vector_farthest
                    points.append(new_point)
                    obstacle_i.append((int(new_point[0]),int(new_point[1])))
                if len(obstacle_i) >= 3:
                    obstacles.append(Polygon(obstacle_i))


        # Build the visibility graph for the given points and obstacles
        g = vg.VisGraph()

        # Error catching because geometry of obstacles might be bad if a non-obstacle
        # is detected
        try:
            if found_obstacles:
                # Combining obstacles if they intersect
                exclude_list = []
                for i in range(len(obstacles)):
                    for j in range(i+1, len(obstacles)):
                        if obstacles[i].intersects(obstacles[j]):
                            exclude_list.append(i)
                            obstacles[j] = unary_union([obstacles[i],obstacles[j]])

                # Making a new list of non-intersecting polygons to use for graph making
                new_obstacles = []
                for i, obstacle in enumerate(obstacles):
                    if i not in exclude_list:
                        coords = list(obstacle.exterior.coords)
                        new_obstacles.append([vg.Point(coord[0], coord[1]) for coord in coords])

                # Draw the points
                for point in points:
                    cv2.circle(self.copy, point.astype(int), 5, (255, 0, 0), -1)
                g.build(new_obstacles)
        except:
            return False, points, g
        return found_obstacles, points, g

    # Function to find the goal location
    def find_goal(self):
        # Convert the image from BGR to RGB
        rgb_image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

        # Define the lower and upper bounds for the goal color in RGB (assuming green)
        lower_green = np.array([60, 130, 100])
        upper_green = np.array([100, 180, 150])

        # Create a binary mask
        green_mask = cv2.inRange(rgb_image, lower_green, upper_green)
        # Apply some preprocessing
        green_mask = cv2.bilateralFilter(green_mask,5,15,15)
        # Find contours
        contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 100:  # threshold for goal size
                # Draw the contours
                color = (0, 255, 0)
                cv2.drawContours(self.copy, [contour], 0, color, 2)
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return True, Goal(cx, cy)

        return False, Goal(0, 0)  # Default value if goal not found
    
