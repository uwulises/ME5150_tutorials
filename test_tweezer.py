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
p.setTimeStep(1 / 240)

# Load KR6 robot arm and table
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("brazos/kuka_model/kr6_2.urdf", basePosition = [0, 0, 0], useFixedBase = useFixedBase)
tableId = p.loadURDF("table/table.urdf", basePosition = [1.5, 0, 0], useFixedBase = useFixedBase)
tweezerId = p.loadURDF("efectores/tweezer/tweezer.urdf", basePosition = [1.5, 0, 1])
cubeId = p.loadURDF("objetos/cubo.urdf", basePosition = [1.2, 0, 0.7])

time.sleep(1)

# Join gripper to robot
joint_axis_gripper = [0, 0, 0]
gripper_parentFramePosition = [0, 0, 0]
gripper_childFramePosition = [0, 0, -0.036]
gripper_childFrameOrientation = p.getQuaternionFromEuler([-np.pi/2,-np.pi/2,0])
joint_constraint = p.createConstraint(robotId, 5, childBodyUniqueId = tweezerId, childLinkIndex= -1, jointType= p.JOINT_FIXED, jointAxis= joint_axis_gripper, parentFramePosition=gripper_parentFramePosition,
                                     childFramePosition=gripper_childFramePosition, childFrameOrientation=gripper_childFrameOrientation)

# tool coordinate position
n_tcf = 5

# Create sliders for X, Y, Z, A, B, and C
x_slider = p.addUserDebugParameter("X", 0, 2, 1)
y_slider = p.addUserDebugParameter("Y", -1, 1, 0)
z_slider = p.addUserDebugParameter("Z", 0, 1.5, 1)
a_slider = p.addUserDebugParameter("A", -3.14, 3.14, 0)
b_slider = p.addUserDebugParameter("B", -3.14, 3.14, 0)
c_slider = p.addUserDebugParameter("C", -3.14, 3.14, 0)

# Create button for end effector activation

effector_slider = p.addUserDebugParameter("End effector", -0.03, 0, 0)

# Run the simulation
while True:

    # get the current slider values
    x = p.readUserDebugParameter(x_slider)
    y = p.readUserDebugParameter(y_slider)
    z = p.readUserDebugParameter(z_slider)
    a = p.readUserDebugParameter(a_slider)
    b = p.readUserDebugParameter(b_slider)
    c = p.readUserDebugParameter(c_slider)
    eff = p.readUserDebugParameter(effector_slider)

    # target position
    xyz = [x,y,z]
    # target orientation
    ori = p.getQuaternionFromEuler([a,b,c])
    
    # calculate joint target
    target = p.calculateInverseKinematics(robotId, endEffectorLinkIndex = n_tcf, targetPosition = xyz, targetOrientation = ori)
    p.setJointMotorControlArray(robotId, range(6), p.POSITION_CONTROL, targetPositions = target)
    p.setJointMotorControlArray(tweezerId, range(2), p.POSITION_CONTROL, targetPositions = [eff, -eff])
    p.stepSimulation()