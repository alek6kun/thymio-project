from vision.ComputerVision import Vision
import time
import cv2

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

vis = Vision()
# Initialize a variable to store the last time an image was acquired
last_image_time = time.time()

########################### Filtering setup ###########################
## General parameters
dimension_x = 3 # State dimension
dimension_z = 3 # Measurement dimension
R = 0.02 # [m] The radius of the Thymio's wheels (a remuserer) [TODO]
d = 0.095 # [m] The wheelbase of the Thymio
dt = 0.05 # [s] Time delta between steps

# Creating the filter
f = KalmanFilter(dim_x=dimension_x, dim_z=dimension_z) # state and measurement variables are x, y and theta

## Filter parameters
# Initial state (get from camera? or we know) [TODO]
f.x = np.array([0., 0., 0.])
# State transition matrix
f.F = np.array([[1,0,0],
                [0,1,0],
                [0,0,1]])   
# Control transition matrix
B = np.array([[np.cos(f.x[2])*(dt/2), np.cos(f.x[2])*(dt/2)],
              [np.sin(f.x[2])*(dt/2), np.sin(f.x[2])*(dt/2)],
              [(dt/d), (-dt/d)]]) * R  
#Control input
u = np.array([[0],
              [0]]) # Replace with motor velocities [TODO]        
# Measurement function
f.H = np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])
# Covariance matrix (to replace with actual values) [TODO]
f.P = np.array([[1000., 0., 0.],
                [0., 1000., 0.],
                [0., 0., 1000.]])
# Measurement noise [TODO]
f.R = Q_discrete_white_noise(dim=dimension_z,dt=dt,var=0.3)
# Process noise (defined here as general white noise, must review)[TODO]
f.Q = Q_discrete_white_noise(dim=dimension_x, dt=dt, var=0.13) 


while True:
    # Check if it's been at least 0.05 second since the last image acquisition
    if time.time() - last_image_time < 0.05:
        continue
    vis.show()

    vis.update()
    
    z = np.array([[vis.robot.x, vis.robot.y, vis.robot.angle]]) # Camera reading
    print(z)
    f.predict(B = B, u = u) # Predict step of the Kalman filter
    if vis.found_robot: # Only update if the robot was found
        f.update(z) # Update step based on measurements from the camera
    #print(f.x[0,0],f.x[1,0])
    vis.copy = cv2.circle(vis.copy,(int(f.x[0,0]),int(f.x[1,0])),20,(255,0,0),3)
    
    # Update the last image acquisition time
    last_image_time = time.time()

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

del vis

