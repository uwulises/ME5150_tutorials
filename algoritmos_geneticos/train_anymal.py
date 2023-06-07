import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240.0)

# Load KR6 robot arm and table
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("../anymal/urdf/anymal.urdf", basePosition = [0, 0, 0.63], useFixedBase = True)

    

# Run the simulation
while True:

   
    
    p.stepSimulation()

    