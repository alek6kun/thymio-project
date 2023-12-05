from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from vision.ComputerVision import Vision
import numpy as np


#vis = Vision()

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
u = np.array([[150],
              [80]]) # Replace with motor velocities [TODO]        
# Measurement function
f.H = np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])
# Covariance matrix (to replace with actual values) [TODO]
f.P = np.array([[1000., 0., 0.],
                [0., 1000., 0.],
                [0., 0., 1000.]])
# Measurement noise [TODO]
camera_variances = (1.13554018e-01, 1.93571267e-01, 2.02748876e-05)
f.R = np.diag(np.array(camera_variances))
# Process noise (defined here as general white noise, must review)[TODO]
f.Q = Q_discrete_white_noise(dim=dimension_x, dt=0.1, var=0.13) 
i = 0

while True: # Replace true with a condition [TODO]
    z = np.array([[40, 20, 10]]) # Replace with camera reading [TODO]
    f.predict() # Predict step of the Kalman filter
    f.update(z) # Update step based on measurements from the camera
    i+=1
    print(i)
    
# The new estimate is stored in f.x
print(f.x[:,0])
