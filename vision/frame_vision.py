import cv2
import time
import numpy as np

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

def normal(v):
    length = np.sqrt(v[0]**2+v[1]**2)
    v[0] = v[0]/float(length)
    v[1] = v[1]/float(length)
    return v

########## MAIN CODE ##########

frame = cv2.imread("Capture.png")

#Convert the frame to grayscale
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Apply a binary threshold to identify black pixels
_, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY)

# Find contours in the binary image along with hierarchy
contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Set contour area thresholds
min_contour_area = 1000
max_contour_area = 200000

# Approximation accuracy
epsilon = 0.025

# Store points at a distance of 50 pixels from the corners
points = []

# Draw all contours on the original image with a consistent color
for i, contour_i in enumerate(contours):
    contour_area_i = cv2.contourArea(contour_i)

    if max_contour_area > contour_area_i > min_contour_area:
        # Process the contour as needed
        # ...

        # Draw the contours
        color = (0, 255, 0)
        cv2.drawContours(frame, [contour_i], 0, color, 2)

        # Find the corners of the contour
        corners = cv2.approxPolyDP(contour_i, epsilon * cv2.arcLength(contour_i, True), True)

        # Draw circles at each corner
        for corner in corners:
            # Find the vector pointing outward from the corner
            vector_farthest = find_vector_farthest(corner[0], corners)
            # Place a point at a distance of 40 pixels from the corner
            new_point = corner[0] + 40 * vector_farthest
            points.append(new_point)

# Draw the points
for point in points:
    cv2.circle(frame, tuple(point.astype(int)), 5, (255, 0, 0), -1)

# Display the result
cv2.imshow("Processed Frame", frame)

# Close all windows if the 'q' key is pressed
cv2.waitKey(0)
cv2.destroyAllWindows()
