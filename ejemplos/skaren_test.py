import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to the PyBullet simulator
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # Set the search path to find the plane.urdf file


p.setGravity(0,0,-9.81) # Set the gravity to be in the negative z direction
p.setRealTimeSimulation(1) # Set the simulation to be real time

planeId = p.loadURDF("plane.urdf") # Load the plane into the simulation

# Load the roboticArmURDF into the simulation
robotic_arm = p.loadURDF("modelos/manipuladores/skaren/skaren.urdf", basePosition=[0,0,0], useFixedBase=True)

while True:
    # Step the simulation
    p.stepSimulation()
    time.sleep(1/240)  # Control the simulation speed