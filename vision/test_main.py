from ComputerVision import Vision
import time
import cv2

vis = Vision()
# Initialize a variable to store the last time an image was acquired
last_image_time = time.time()

while True:
    # Check if it's been at least 0.05 second since the last image acquisition
    if time.time() - last_image_time < 0.05:
        continue
    vis.show()
    path = vis.shortest_path
    vis.update(path)

    # Update the last image acquisition time
    last_image_time = time.time()

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

del vis

