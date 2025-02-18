import pybullet as p
import time

# Connect to PyBullet
physicsClient = p.connect(p.GUI)

# Add a debug point
p.addUserDebugPoints(
    pointPositions=[[2, 2, 2]],
    pointColorsRGB=[[1, 0, 0]],
    pointSize=10
)

# Keep the window open
while True:
    time.sleep(1)