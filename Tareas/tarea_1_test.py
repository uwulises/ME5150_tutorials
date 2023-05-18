import pybullet as p
import pybullet_data
import time
import numpy as np

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

# Load environment
planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("table/table.urdf", basePosition = [0.3, 0, 0], useFixedBase = True)
cajaId = p.loadURDF("objetos/caja.urdf", basePosition = [0.5, 0.2, 0.7]) #Caja para dejar el objeto manipulado
robotId = p.loadURDF("brazos/scara_fcfm_model/scara.urdf", basePosition = [0, 0, 0.63], useFixedBase = True)

# tool coordinate position
n_tcf = 2

# Load end effector
tweezerId = p.loadURDF("efectores/scara/tweezer/tweezer.urdf", basePosition = [0, 0.6, 0.04])

joint_axis_gripper = [0, 0, 0]
gripper_parentFramePosition = [0, 0, 0]
gripper_childFramePosition = [0, 0, 0] #ubicar
gripper_childFrameOrientation = p.getQuaternionFromEuler([0,0,0])
joint_constraint = p.createConstraint(robotId, parentLinkIndex=2, childBodyUniqueId = tweezerId,
                            childLinkIndex= -1, jointType= p.JOINT_FIXED, jointAxis= joint_axis_gripper, 
                            parentFramePosition=gripper_parentFramePosition,
                            childFramePosition=gripper_childFramePosition, childFrameOrientation=gripper_childFrameOrientation)

# Create sliders for X, Y, Z, A, B, and C
x_slider = p.addUserDebugParameter("X", 0, 2, 1)
y_slider = p.addUserDebugParameter("Y", -1, 1, 0)
z_slider = p.addUserDebugParameter("Z", 0, 1.5, 1)
c_slider = p.addUserDebugParameter("C", -3.14, 3.14, 0)
eff_slider = p.addUserDebugParameter("eff", -0.03,0, 0)

# Simulation cicles for instruction
rate = 2000
time.sleep(1)

while True:
# Run the simulation
    # get the current slider values
    x = p.readUserDebugParameter(x_slider)
    y = p.readUserDebugParameter(y_slider)
    z = p.readUserDebugParameter(z_slider)
    c = p.readUserDebugParameter(c_slider)
    eff = p.readUserDebugParameter(eff_slider)

    # target position
    xyz = [x,y,z]
    
    # calculate joint target
    target = p.calculateInverseKinematics(robotId, endEffectorLinkIndex = n_tcf, targetPosition = xyz)
    p.setJointMotorControlArray(tweezerId, range(3), p.POSITION_CONTROL, targetPositions = [c, eff, -eff])
    p.setJointMotorControlArray(robotId, range(n_tcf+1), p.POSITION_CONTROL, targetPositions = target)
    p.stepSimulation()
