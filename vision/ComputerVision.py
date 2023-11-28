import cv2
import time
import numpy as np
import pyvisgraph as vg

########## FUNCTIONS ##########
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

class Vision:
    # Initialise cam, frame, copy of frame, robot, scale, graph, obstacle vertices
    def __init__(self):
        self.cam = cv2.VideoCapture(0)
        valid, self.frame = self.cam.read()
        if not valid:
            print("Error reading frame.")
        self.copy = self.frame.copy()
        self.robot, self.scale = self.find_robot()
        self.graph, self.vertices = self.find_graph()
        self.goal = self.find_goal()

    def __del__(self):
        # Release the camera and close all windows
        self.cam.release()
        cv2.destroyAllWindows()

    def update(self):
        valid, self.frame = self.cam.read()
        if not valid:
            print("Error reading frame.")
        self.robot, self.scale = self.find_robot()
        self.graph, self.vertices = self.find_graph()
        self.goal = self.find_goal()
    
    def show(self):
        cv2.imshow("Processed Frame", self.copy)
    
    def find_shortest_path(self):
        shortest = self.graph.shortest_path(vg.Point(self.robot.x,self.robot.y),
                                            vg.Point(self.goal.x,self.goal.y))

        shortest_np = np.array([(point.x, point.y) for point in shortest], dtype=np.int32)

        return shortest_np

    # Function to find robot location and frame scale
    def find_robot(self):
        # Convert the image from BGR to RGB (OpenCV loads images in BGR by default)
        rgb_image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

        # Define the lower and upper bounds for the red color in RGB
        lower_red = np.array([130, 0, 0])
        upper_red = np.array([255, 100, 100])

        # Create a binary mask using inRange function
        red_mask = cv2.inRange(rgb_image, lower_red, upper_red)

        # Apply the binary mask to the original image
        result = cv2.bitwise_and(self.frame, self.frame, mask=red_mask)

        # Convert the result to grayscale
        gray_result = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

        contours, _ = cv2.findContours(gray_result, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        min_contour_area = 100
        max_contour_area = 1000
        centroid = [0,0]
        # Draw the contours
        for i, contour_i in enumerate(contours):
            contour_area_i = cv2.contourArea(contour_i)

            if max_contour_area > contour_area_i > min_contour_area:
                color = (0, 0, 255)
                cv2.drawContours(self.copy, [contour_i], 0, color, 2)
            M = cv2.moments(contour_i)
            centroid[i] = np.array([int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])])
        cv2.line(self.copy, centroid[0], centroid[1], (0,0,255),2)

        # The real distance between the two markers on the robot is 5 cm. so we set a scale
        # to know how distant the objects are from the camera.
        scale = np.linalg.norm(centroid[0] - centroid[1])/5

        return Robot(centroid[0][0] + centroid[1][0]/2, centroid[0][0] + centroid[1][0]/2,
                     np.arctan2(centroid[0][0]-centroid[1][0],centroid[1][0]-centroid[0][0])), scale
    
    def find_graph(self):
        #Convert the frame to grayscale
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        # Apply a binary threshold to identify black pixels
        _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)

        # Find contours in the binary image along with hierarchy
        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Set contour area thresholds
        min_area = 1000
        max_area = 200000

        # Approximation accuracy
        epsilon = 0.025

        # Store points and obstacles
        points = []
        obstacles = []

        for _, contour_i in enumerate(contours):
            contour_area_i = cv2.contourArea(contour_i)

            if min_area < contour_area_i < max_area:
                # Draw the contours
                color = (0, 255, 0)
                cv2.drawContours(self.copy, [contour_i], 0, color, 2)

                # Find the corners of the contour
                corners = cv2.approxPolyDP(contour_i, epsilon * cv2.arcLength(contour_i, True), True)
                obstacle_i = []

                # Draw circles at each corner
                for corner in corners:
                    # Find the vector pointing outward from the corner
                    vector_farthest = find_vector_farthest(corner[0], corners)
                    # Place a point at a distance of 5 cm from the corner using the 
                    # SCALE we have found earlier from the robot
                    new_point = corner[0] + self.scale * 5 * vector_farthest
                    points.append(new_point)
                    obstacle_i.append(vg.Point(new_point[0],new_point[1]))
                obstacles.append(obstacle_i)

        # Build the visibility graph for the given points and obstacles
        g = vg.VisGraph()
        g.build(obstacles)

        return points, g

    def find_goal(self):
                # Convert the image from BGR to RGB
        rgb_image = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

        # Define the lower and upper bounds for the goal color in RGB (assuming green)
        lower_green = np.array([50, 100, 50])
        upper_green = np.array([70, 255, 70])

        # Create a binary mask
        green_mask = cv2.inRange(rgb_image, lower_green, upper_green)

        # Find contours
        contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > 100:  # threshold for goal size
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return Goal(cx, cy)

        return Goal(0, 0)  # Default value if goal not found
    
