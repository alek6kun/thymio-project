from vision.ComputerVision import Vision
import time
import cv2

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

vis = Vision()
# Initialize a variable to store the last time an image was acquired
last_image_time = time.time()

# Empty list to store all sensor measurements
camera_readings = []

while True:
    # Check if it's been at least 0.05 second since the last image acquisition
    if time.time() - last_image_time < 0.05:
        continue
    vis.show()

    vis.update()
    
    z = np.array([vis.robot.x, vis.robot.y, vis.robot.angle]) # Camera reading
    camera_readings.append(z)

    # Update the last image acquisition time
    last_image_time = time.time()

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


camera_readings = np.array(camera_readings)
print(camera_readings.shape)

column_variances = np.var(camera_readings, axis=0) 
print(column_variances)


del vis