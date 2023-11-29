# Imports
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

f = KalmanFilter(dim_x=3, dim_z=3) # state and measurement values are x, y and theta

##################### Filter parameters #####################
# Initial state (get from camera? or we know)
f.x = np.array([[0],  # x
                [0],  # y
                [0]]) # theta
# State transition matrix
f.F = np.array([[1,0,0],
                [0,1,0],
                [0,0,1]])             
# Measurement function
f.H = np.array([1,1,1])
# Covariance matrix (to replace with actual values)
f.P = np.diag([1000])
# Measurement noise
f.R = np.array([])

f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13) # Process nois (defined here as general white noise, must review)[TODO]

while True: # Replace true with a condition [TODO]
    z = 0 # Replace with camera reading [TODO]
    f.predict() # Predict step of the Kalman filter
    f.update(z) # Update step based on measurements from the camera
    
f.x # This is where the new optimal estimate is stored.