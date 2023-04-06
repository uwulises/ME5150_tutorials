import pybullet as p
import pybullet_data
import time

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True #Para que el brazo no flote o se desplace
# Load UR5 robot arm and table
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("kr6_2.urdf", basePosition=[0, 0, 0],useFixedBase=useFixedBase)

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)


# Get the joint info for the first joint of the robot arm
jointInfo0 = p.getJointInfo(robotId, 0)
jointInfo1 = p.getJointInfo(robotId, 1)
jointInfo2 = p.getJointInfo(robotId, 2)
jointInfo3 = p.getJointInfo(robotId, 3)
jointInfo4 = p.getJointInfo(robotId, 4)
jointInfo5 = p.getJointInfo(robotId, 5)

# Add a slider to the GUI for controlling the position of the first joint
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

    # Set the position of all joints of the robot arm
    p.setJointMotorControlArray(robotId, range(6), p.POSITION_CONTROL, targetPositions=joints)

    # effectorId = p.getBodyInfo(robotId)[1]
    # effectorPos, effectorOrn = p.getBasePositionAndOrientation(effectorId)
    # p.addUserDebugLine(effectorPos, [0, 0, 1], lineWidth=5, lifeTime=0)
    # p.addUserDebugText(f"Effector position: {effectorPos}", [0, 0, 3], lifeTime=0)


    # Step the simulation
    p.stepSimulation()

# Disconnect from PyBullet

p.disconnect()
