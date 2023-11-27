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
- robot : contains x, y and angle of robot
- scale : contains the scale of the image: pixels/real length (cm)
- vertices : contains the coordinates of the visibility graph vertices
- goal : contains x and y of goal

Other variables, for completeness:
- cam : object of class cv2.VideoCapture
- frame : one frame of the video capture
- copy : copy of frame used to show computer vision output
- graph : object of class pyvisgraph.VisGraph(), used for path finding