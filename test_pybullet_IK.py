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
tableId = p.loadURDF("table/table.urdf", basePosition=[1.5, 0, 0],useFixedBase=useFixedBase)
# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

# create sliders for X, Y, Z, A, B, and C
x_slider = p.addUserDebugParameter("X", -0.5, 2, 1)
y_slider = p.addUserDebugParameter("Y", -0.5, 0.5, 0)
z_slider = p.addUserDebugParameter("Z", 0, 1.5, 1)
a_slider = p.addUserDebugParameter("A", -3.14, 3.14, 0)
b_slider = p.addUserDebugParameter("B", -3.14, 3.14, 0)
c_slider = p.addUserDebugParameter("C", -3.14, 3.14, 0)

# Run the simulation
while True:

    # get the current slider values
    x = p.readUserDebugParameter(x_slider)
    y = p.readUserDebugParameter(y_slider)
    z = p.readUserDebugParameter(z_slider)
    a = p.readUserDebugParameter(a_slider)
    b = p.readUserDebugParameter(b_slider)
    c = p.readUserDebugParameter(c_slider)
    #target position
    xyz=[x,y,z]
    #target orientation
    ori = p.getQuaternionFromEuler([a,b,c])
    #calculate joint target
    target= p.calculateInverseKinematics(robotId, 5, targetPosition=xyz, targetOrientation=ori)
    p.stepSimulation()
    p.setJointMotorControlArray(robotId, range(6), p.POSITION_CONTROL, targetPositions=target)
    p.stepSimulation()
    time.sleep(0.1)
    

    

