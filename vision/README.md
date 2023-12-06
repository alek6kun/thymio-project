## How to use the computer vision file:

In the main.py, do
```python
from ComputerVision import Vision
```

Initialise an object of vision class, which can be used to for various tasks
```python
vis = Vision()

# Returns list of coordinates from robot to goal
vis.find_shortest_path()

# Updates localisations
vis.update()

# Shows vision output
vis.show()

# Individual variables can also be accessed, e.g.
vis.robot.x  # The x coordinate of the robot
```

List of useful variables:
- robot : contains x, y and angle of robot. Angle is given in the normal
  trigonometric standard (+pi to -pi, 0 at x axis pointing right)
- scale : contains the scale of the image: pixels/real length (cm)
- vertices : contains the coordinates of the visibility graph vertices
- goal : contains x and y of goal
- shortest_path : list of coordinates of points to follow for shortest path 
  to goal
- found_robot, found_graph, found_goal : booleans returning true if the select
  entities are found

Other variables, for completeness:
- cam : object of class cv2.VideoCapture
- frame : one frame of the video capture
- copy : copy of frame used to show computer vision output
- graph : object of class pyvisgraph.VisGraph(), used for path finding

To delete the object of vision class, do:
```python
del vis
```

## Tweaks that might need to be made

- Change the webcam ID in the cv2.VideoCapture() function in the __init__() function
- Change the lower and upper bounds for the colors that are detected in the find_robot() and find_goal() functions.
- Change the black detection threshold in the cv2.threshold() function in find_graph.
