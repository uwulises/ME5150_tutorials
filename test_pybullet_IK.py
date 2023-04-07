import pybullet as p
import pybullet_data
import time
import numpy as np

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True #Para que el brazo no flote o se desplace
# Load KR6 robot arm and table
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("kr6_2.urdf", basePosition=[0, 0, 0],useFixedBase=useFixedBase)

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

#target position
xyz=[1,0,0.8]
#calculate joint target
target= p.calculateInverseKinematics(robotId, 5, targetPosition=xyz)

# Run the simulation
while True:
    p.stepSimulation()
    p.setJointMotorControlArray(robotId, range(6), p.POSITION_CONTROL, targetPositions=target)
    p.stepSimulation()
    time.sleep(0.1)
    

    

