# Imports
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

## General parameters
dimension_x = 3 # State dimension
dimension_z = 3 # Measurement dimension
R = 0.02 # The radius of the Thymio's wheels (a remuserer) [TODO]
d = 0.095 # The wheelbase of the Thymio
dt = 0.01 # Time delta between steps

# Creating the filter
f = KalmanFilter(dim_x=dimension_x, dim_z=dimension_z) # state and measurement variables are x, y and theta


## Filter parameters
# Initial state (get from camera? or we know)
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
u = np.array([[2],
              [2]]) # Replace with motor velocities [TODO]        
# Measurement function
f.H = np.array([[1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]])
# Covariance matrix (to replace with actual values) [TODO]
f.P = np.array([[1000., 0., 0.],
                [0., 1000., 0.],
                [0., 0., 1000.]])
# Measurement noise [TODO]
f.R = Q_discrete_white_noise(dim=dimension_z,dt=0.1,var=0.3)
# Process noise (defined here as general white noise, must review)[TODO]
f.Q = Q_discrete_white_noise(dim=dimension_x, dt=0.1, var=0.13) 
i=0
while i<14: # Replace true with a condition [TODO]
    z = np.array([[1, 1, 10]]) # Replace with camera reading [TODO]
    f.predict(B=B, u = u) # Predict step of the Kalman filter
    f.update(z) # Update step based on measurements from the camera
    i+=1
    
# The new estimate is stored in f.x
print(f.x[:,0])