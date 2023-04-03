import pybullet as p
import pybullet_data
import time

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True #Para que el brazo no flote o se desplace
# Load UR5 robot arm and table
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("kr6_2.urdf", [0, 0, 0],useFixedBase=useFixedBase)

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

# Move the arm to the starting position
p.resetJointState(robotId, 0, -0.5)
p.resetJointState(robotId, 1, -1.0)
p.resetJointState(robotId, 2, 1.0)
p.resetJointState(robotId, 3, -1.57)
p.resetJointState(robotId, 4, 1.57)
p.resetJointState(robotId, 5, 0.0)


# Run the simulation
while True:
    p.stepSimulation()
    # Move the arm towards the object
    #jointPos = p.calculateInverseKinematics(robotId, 6, targetPos)
    #p.setJointMotorControlArray(robotId, [0, 1, 2, 3, 4, 5], p.POSITION_CONTROL, jointPos)

# Disconnect from PyBullet

p.disconnect()
