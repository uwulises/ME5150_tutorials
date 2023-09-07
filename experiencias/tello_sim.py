import pybullet as p
import pybullet_data
import numpy as np
# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)
p.setRealTimeSimulation(0)

planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("modelos/drones/djitello/djitello.urdf", basePosition=[0, 0, 0.3], useFixedBase=False)

dn = 500

def main():
    while True:
        p.stepSimulation()
        

if __name__ == "__main__":
    
    main()