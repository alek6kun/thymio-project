from filterpy.kalman import KalmanFilter
import numpy as np
import cv2


## General parameters
dimension_x = 3 # State dimension
dimension_z = 3 # Measurement dimension

thymio_speed_to_mms = 19.73913043478261/50 # Conversion from motor units to mm/s 
R = 0.021 # [m] The radius of the Thymio's wheels
d = 0.095 # [m] The wheelbase of the Thymio
dt = 0.05 # [s] Time delta between steps

# Creating the filter
f = KalmanFilter(dim_x=dimension_x, dim_z=dimension_z) # state and measurement variables are x, y and theta

## Filter parameters
# Initial state
f.x = np.array([0., 0., 0.])
# State transition matrix
f.F = np.eye(3)        
# Measurement function
f.H = np.eye(3)
# Initial covariance matrix
f.P = np.eye(3)*10
# Measurement noise
camera_variances = [1.13554018e-01, 1.93571267e-01, 2.02748876e-05]
f.R = np.diag(camera_variances)
# Process noise
process_variances = [3.8751605996417765e-10, 3.8751605996417765e-10, 2.9656863710880975e-09]
f.Q = np.diag(process_variances)


def run_filter(speed_right, speed_left, prev_angle, vis):
    global f
    scale = vis.scale # Scale to go from m to camera coordinates
    # Converting the motors to m/s
    speed_right = (speed_right * 0.001) / thymio_speed_to_mms
    speed_left = (speed_left * 0.001) / thymio_speed_to_mms
    # Converting to camera coordinates
    speed_left *= scale
    speed_right *= scale
    Radius = R * scale
    wheelbase *= scale
    # Defining control input and control transition matrix
    u = np.array([[speed_right],
                  [speed_left]])
    B = np.array([[np.cos(prev_angle)*(dt/2), np.cos(prev_angle)*(dt/2)],
                    [np.sin(prev_angle)*(dt/2), np.sin(prev_angle)*(dt/2)],
                    [(dt/wheelbase), (-dt/wheelbase)]]) * Radius
    # Getting camera measurements
    z = np.array([vis.robot.x,vis.robot.y,vis.robot.angle]) 
    print(u)
    print(B)
    f.predict(u=u, B=B)
    # Only update if we have new camera measurements
    if vis.found_robot:
        f.update(z)
        
    cv2.circle(vis.copy,(int(f.x[0,0]),int(f.x[1,0])),20,(255,0,0),3)
    return f.x[:,0] # Return the kalman filtered state