import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2


# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

# Load SCARA robot arm and table
planeId = p.loadURDF("plane.urdf")
tableId = p.loadURDF("table/table.urdf", basePosition = [0.3, 0, 0], useFixedBase = useFixedBase)
robotId = p.loadURDF("brazos/scara_fcfm_model/scara.urdf", basePosition = [0, 0, 0.63], useFixedBase = useFixedBase)

"""-------- Item 1: Cargar objeto que quieran tomar y ubicar sobre la mesa --------- """

"""-------- Item 2: Cargar gripper tweezer, ubicar, orientar y restringir 
            correctamente en la punta del efector del SCARA               ---------- """

# tool coordinate position
n_tcf = 2

# Join gripper to robot
tweezerId = p.loadURDF("efectores/scara/tweezer/tweezer.urdf", basePosition = [0, 0.6, 0.2])

joint_axis_gripper = [0, 0, 0]
gripper_parentFramePosition = [0, 0, 0]
gripper_childFramePosition = [-0.013, 0.107, 0]
gripper_childFrameOrientation = p.getQuaternionFromEuler([0,0,0])
joint_constraint = p.createConstraint(robotId, 2, childBodyUniqueId = tweezerId, childLinkIndex= -1, jointType= p.JOINT_FIXED, jointAxis= joint_axis_gripper, parentFramePosition=gripper_parentFramePosition,
                                     childFramePosition=gripper_childFramePosition, childFrameOrientation=gripper_childFrameOrientation)


""" ------  Item 3: Crear una rutina de pick and place, donde el robot tome el
            objeto que eligieron y lo deje en la caja sobre la mesa, mínimo 40 poses. 
            
            path_SCARA recibe vectores [x, y, z, c, eff]                       --------- """ 

path_SCARA = [[1, 0, 1, 0.01], [1.05, 0, 0, 0]]

# Simulation cicles for instruction
rate = 2000

# Run the simulation
for i in range(len(path_SCARA)):
    j = 1
    while (j/rate <= 1):

        # target position
        xyz = path_SCARA[i][:3]
        eff = path_SCARA[i][3]
        
        # calculate joint target
        #target = p.calculateInverseKinematics(robotId, endEffectorLinkIndex = n_tcf, targetPosition = xyz)
        """-------- Item 4: Agregar línea que controla estado del efector ---------"""

        #p.setJointMotorControlArray(robotId, range(n_tcf+1), p.POSITION_CONTROL, targetPositions = target)
        p.stepSimulation()

        j+=1
while True:
    p.stepSimulation()







