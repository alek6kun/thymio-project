# Imports
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
import numpy as np

f = KalmanFilter(dim_x=3, dim_z=3) # we have x,y and theta of the robot both from the measurement and the model
f.x = np.array([[0,0,0],
                [0,0,0],]) # Initial position and velocity (get from camera? or we know)

f.F = np.array([]) # State transition matrix  [TODO]
               
f.H = np.array([]) # Measurement function [TODO]

f.P = np.array([[1000.,    0.],
                [   0., 1000.] ]) # Covariance matrix (to replace with actual values) [TODO]

f.R = np.array([]) # Measurement noise [TODO]

f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13) # Process nois (defined here as general white noise, must review)[TODO]

while True: # Replace true with a condition [TODO]
    z = 0 # Replace with camera reading [TODO]
    f.predict() # Predict step of the Kalman filter
    f.update(z) # Update step based on measurements from the camera
    
f.x # This is where the new optimal estimate is stored.