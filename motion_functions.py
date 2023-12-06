import math
#USED TO CHECK FOR COLLISION AT REGULAR INTERVALS, Source : ex week 8
from threading import Timer
import time
import numpy as np

#Constants
Ts = 0.01
SPEED = 50
SEUIL_OBSTACLE = 100
SEUIL_KIDNAPPED = 100
tol = 30
#VALUES THAT NEED TO BE TUNED BEGIN: 
##We add correction factors to tune the speed symmetry
SPEED_LEFT = SPEED +1 
SPEED_RIGHT = SPEED

thymio_speed_to_mms = 19.73913043478261*1000/50 ##TO FIND THE MM/s speed, divide motors_speed by this, thix value has to be tuned for each thymio
rotation_factor = 110*np.pi/360 
#VALUES TO BE TUNED END
obstacle_detected = 0
kidnapped = 0

def compute_movement(current_pos, obj, current_angle): 
    # current_angle_deg = angle between thymio front axis and x-axis
    # Extracting coordinates from current_pos and obj
    current_x, current_y = current_pos
    obj_x, obj_y = obj

    # Calculate distance between current position and the target object
    #distance = math.sqrt((obj_x - current_x)**2 + (obj_y - current_y)**2)

    # Calculate angle in radians
    angle_radians = math.atan2(obj_y - current_y, obj_x - current_x)
    
    


    
    return  angle_radians



class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer     = None
        self.interval   = interval
        self.function   = function
        self.args       = args
        self.kwargs     = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False

def get_sensors(node):
    
        global obstacle_detected, kidnapped
        prox_values = node["prox.horizontal"][:5]

        obstacle_detected = any(value > SEUIL_OBSTACLE for value in prox_values)
        
        ground_values = node["prox.ground.reflected"]
        
        if ground_values[0] < SEUIL_KIDNAPPED or ground_values[1] < SEUIL_KIDNAPPED:
            kidnapped = True 
            print(f"kidnapped")
        else:
            kidnapped = False
        return kidnapped, obstacle_detected
            
async def drive(node): 
    v = {
        "motor.left.target": [int(SPEED_LEFT*3)],
        "motor.right.target": [int(SPEED_RIGHT*3)],
    }
    await node.set_variables(v)
    
async def stop(node): 
    v = {
        "motor.left.target": [0],
        "motor.right.target": [0],
    }
    await node.set_variables(v)
    
async def rotate(angle_diff,node) : 
    if angle_diff <0 :
        v = {
            "motor.left.target": [int(-SPEED_LEFT)],
            "motor.right.target": [int(SPEED_RIGHT)],
        }
    else :
        v = {
        "motor.left.target": [int(SPEED_LEFT)],
        "motor.right.target": [int(-SPEED_RIGHT)],
    }
    await node.set_variables(v)
    
async def drive_back(node):
    v = {
        "motor.left.target": [-int(SPEED_LEFT*4)],
        "motor.right.target": [-int(SPEED_RIGHT*4)],
    }
    await node.set_variables(v)
    

def close_coordinates(x, y, w, z):
    distance = math.sqrt((w - x)**2 + (z - y)**2)
    return distance <= tol