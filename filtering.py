from filterpy.kalman import KalmanFilter
import numpy as np
import cv2
import math


## General parameters
dimension_x = 3 # State dimension
dimension_z = 3 # Measurement dimension

motor_scale = 43.52 # [Motor_space/(rad/s)] Scale of motor speeds in motor space to rad/s
R = 0.021 # [m] The radius of the Thymio's wheels
d = 0.095 # [m] The wheelbase of the Thymio
dt = 0.137 # [s] Time delta between steps

# Creating the filter
f = KalmanFilter(dim_x=dimension_x, dim_z=dimension_z) # state and measurement variables are x, y and theta

## Filter parameters
# State transition matrix
f.F = np.eye(3)        
# Measurement function
f.H = np.eye(3)
# Initial covariance matrix
f.P = np.eye(3) * 100
# Measurement noise
camera_variances = [2.13554018e-01, 2.93571267e-01, 6.02748876e-05]
f.R = np.diag(camera_variances)
# Process noise
process_variances = [3.8751605996417765e-01, 3.8751605996417765e-01, 2.9656863710880975e-03]
f.Q = np.diag(process_variances)


def run_filter(speed_right, speed_left, prev_angle, vis):
    global f

    camera_scale = vis.scale # [camera_coordinates/m]
    # Converting the motors to rad/s
    speed_right = speed_right / motor_scale
    speed_left = speed_left / motor_scale
    
    # Defining control input and control transition matrix
    u = np.array([[speed_right],
                  [speed_left]])
    B = np.array([[np.cos(prev_angle)*(dt/2), np.cos(prev_angle)*(dt/2)],
                    [np.sin(prev_angle)*(dt/2), np.sin(prev_angle)*(dt/2)],
                    [(-dt/d), (dt/d)]]) * R
    
    # Getting camera measurements and conveting to [m]
    measurement = np.array([vis.robot.x/camera_scale,vis.robot.y/camera_scale,-vis.robot.angle])
    
    # Predict step of kalman filter with control input
    f.predict(u = u, B = B)
    # Only update if we have new camera measurements
    if vis.found_robot:
        f.update(measurement)

    # Defining the estimate in camera coordinates
    estimate = np.array([f.x[0,0] * camera_scale, f.x[1,0] * camera_scale, f.x[2,0]])

    # Plotting an ellipse to show evolution of uncertainty along x and y
    cv2.ellipse(vis.copy, (int(estimate[0]), int(estimate[1])), (int(200 * f.P[0,0]), int(200 * f.P[1,1])), math.degrees(f.x[2,0]),0,360,(255,0,0),3)
    # Plotting a line to show the angle estimate
    cv2.line(vis.copy, (int(estimate[0]), int(estimate[1])), (int(estimate[0] + 100*np.cos(estimate[2])), int(estimate[1] + 100*np.sin(estimate[2]))), (255,0,0), 3)
    
    return estimate # Return the kalman filtered state