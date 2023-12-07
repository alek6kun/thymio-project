import math

# Constants
SPEED = 75
THRESHOLD_OBSTACLE = 100
THRESHOLD_KIDNAPPED = 100
tol = 30

# We add correction factors to tune the speed symmetry
SPEED_LEFT = SPEED +1 
SPEED_RIGHT = SPEED


def compute_movement(current_pos, obj): 
    # Extracting coordinates from current_pos and obj
    current_x, current_y = current_pos
    obj_x, obj_y = obj
    # Calculate angle in radians
    angle_radians = math.atan2(obj_y - current_y, obj_x - current_x)
    return  angle_radians


def get_sensors(node):
    # Identifies whether the robot has been kidnapped or not, and whether or not an unplanned obstacle is in its path
    # returns Two booleans : kidnapped and obstacle_detected

    #Read horizontal sensor values
    prox_values = node["prox.horizontal"][:5]

    obstacle_detected = any(value > THRESHOLD_OBSTACLE for value in prox_values)

    #Reads ground sensor reflected values
    ground_values = node["prox.ground.reflected"]
    
    if ground_values[0] < THRESHOLD_KIDNAPPED or ground_values[1] < THRESHOLD_KIDNAPPED:
        kidnapped = True 
    else:
        kidnapped = False
    return kidnapped, obstacle_detected
         
            
async def drive(node): 
    #Makes the robot drive forward
    v = {
        "motor.left.target": [int(SPEED_LEFT*3)],
        "motor.right.target": [int(SPEED_RIGHT*3)],
    }
    await node.set_variables(v)
  
    
async def stop(node): 
    #Stops the motors
    v = {
        "motor.left.target": [0],
        "motor.right.target": [0],
    }
    await node.set_variables(v)
    
    
async def rotate(angle_diff,node) : 
    #Makes the robot rotate
    #If angle <0 : rotates counter-clockwise
    if angle_diff <0 :
        v = {
            "motor.left.target": [int(-SPEED_LEFT)],
            "motor.right.target": [int(SPEED_RIGHT)],
        }
    else :
    #Rotates clockwise
        v = {
        "motor.left.target": [int(SPEED_LEFT)],
        "motor.right.target": [int(-SPEED_RIGHT)],
    }
    await node.set_variables(v)
    
    
async def drive_back(node):
    #Makes the robot drive backward
    v = {
        "motor.left.target": [-int(SPEED_LEFT*4)],
        "motor.right.target": [-int(SPEED_RIGHT*4)],
    }
    await node.set_variables(v)
    
    
def close_coordinates(x, y, w, z):
    #Checks whether (x,y) ~= (w,z), returns True if its the case
    distance = math.sqrt((w - x)**2 + (z - y)**2)
    return distance <= tol