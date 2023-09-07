import pybullet as p
import pybullet_data
import time
import numpy as np

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)
# Load KR6 robot arm and table
planeId = p.loadURDF("plane.urdf")
baseId = p.loadURDF("../modelos/manipuladores/kuka/steel_base.urdf", basePosition = [0, 0, 0.0125], useFixedBase = useFixedBase)
robotId = p.loadURDF("../modelos/manipuladores/kuka/kr6_2.urdf", basePosition = [0, 0, 0.035], useFixedBase = useFixedBase)
tableId = p.loadURDF("table/table.urdf", basePosition = [1.5, 0, 0], useFixedBase = useFixedBase)


# tool coordinate position
n_tcp = 5

# Create sliders for X, Y, Z, A, B, and C
x_slider = p.addUserDebugParameter("X", 0, 2, 1)
y_slider = p.addUserDebugParameter("Y", -1, 1, 0)
z_slider = p.addUserDebugParameter("Z", 0, 1.5, 1.3)
a_slider = p.addUserDebugParameter("A", -3.14, 3.14, 0)
b_slider = p.addUserDebugParameter("B", -3.14, 3.14, 0)
c_slider = p.addUserDebugParameter("C", -3.14, 3.14, 0)

p.addUserDebugLine([0, 0, 0], [0, 0, 0.2], [1, 0, 0], lineWidth=5, parentObjectUniqueId=robotId, parentLinkIndex=5)
p.addUserDebugLine([0, 0, 0], [0, 0.2, 0], [0, 1, 0], lineWidth=5,parentObjectUniqueId=robotId, parentLinkIndex=5)
p.addUserDebugLine([0, 0, 0], [0.2, 0, 0], [0, 0, 1], lineWidth=5,parentObjectUniqueId=robotId, parentLinkIndex=5)

# Run the simulation
while True:

    # get the current slider values
    x = p.readUserDebugParameter(x_slider)
    y = p.readUserDebugParameter(y_slider)
    z = p.readUserDebugParameter(z_slider)
    a = p.readUserDebugParameter(a_slider)
    b = p.readUserDebugParameter(b_slider)
    c = p.readUserDebugParameter(c_slider)
    # target position
    xyz = [x,y,z]
    # target orientation
    ori = p.getQuaternionFromEuler([c,b,a])
    # calculate joint target
    target = p.calculateInverseKinematics(robotId, endEffectorLinkIndex = n_tcp, targetPosition = xyz, targetOrientation = ori)
    p.setJointMotorControlArray(robotId, range(6), p.POSITION_CONTROL, targetPositions = target)