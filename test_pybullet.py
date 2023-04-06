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

# Add a slider to the GUI for controlling the position of the joints
sliderJointId0= jointInfo0[0]
sliderJointId1= jointInfo1[0]
sliderJointId2= jointInfo2[0]
sliderJointId3= jointInfo3[0]
sliderJointId4= jointInfo4[0]
sliderJointId5= jointInfo5[0]

p.addUserDebugParameter("Joint 1 Position", -3.14, 3.14, 0)
p.addUserDebugParameter("Joint 2 Position", -3.14, 3.14, 0)
p.addUserDebugParameter("Joint 3 Position", -3.14, 3.14, 0)
p.addUserDebugParameter("Joint 4 Position", -3.14, 3.14, 0)
p.addUserDebugParameter("Joint 5 Position", -3.14, 3.14, 0)
p.addUserDebugParameter("Joint 6 Position", -3.14, 3.14, 0)

# Run the simulation
while True:
    p.stepSimulation()
    # Get the current position of the slider
    joint1Pos = p.readUserDebugParameter(sliderJointId0)
    joint2Pos = p.readUserDebugParameter(sliderJointId1)
    joint3Pos = p.readUserDebugParameter(sliderJointId2)
    joint4Pos = p.readUserDebugParameter(sliderJointId3)
    joint5Pos = p.readUserDebugParameter(sliderJointId4)
    joint6Pos = p.readUserDebugParameter(sliderJointId5)

    joints= [joint1Pos, joint2Pos, joint3Pos, joint4Pos, joint5Pos, joint6Pos]

    # Set the position of the robot arm
    p.setJointMotorControlArray(robotId, range(6), p.POSITION_CONTROL, targetPositions=joints)

    # Step the simulation
    p.stepSimulation()
