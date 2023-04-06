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


# Get the joints info
jointInfo0 = p.getJointInfo(robotId, 0)
jointInfo1 = p.getJointInfo(robotId, 1)
jointInfo2 = p.getJointInfo(robotId, 2)
jointInfo3 = p.getJointInfo(robotId, 3)
jointInfo4 = p.getJointInfo(robotId, 4)
jointInfo5 = p.getJointInfo(robotId, 5)

# A1 -> A6
path_KR6_2 = [[0,0,0,0,0,0], [1.57,0,0,0,0,0], [-1.57,0,0,0,0,0], [0,0,0,0,0,0]]

# Ciclos por instrucción
rate = 400

# Run the simulation
while True:
    p.stepSimulation()

    for i in range(len(path_KR6_2)):
        j = i*rate

        while (j/(rate*(i+1)) <= 1):
            # Set the position of the robot arm
            p.setJointMotorControlArray(robotId, range(6), p.POSITION_CONTROL, targetPositions=path_KR6_2[i])
            # Step the simulation
            p.stepSimulation()
            
            j+=1

