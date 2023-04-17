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
punteroId = p.loadURDF("efectores/puntero/puntero.urdf")

# Join gripper to robot
joint_axis_gripper = [0, 0, 0]
gripper_parentFramePosition = [0, 0, 0]
gripper_childFramePosition = [0, 0, -0.01]
gripper_childFrameOrientation = p.getQuaternionFromEuler([0,-np.pi/2,0])
joint_constraint = p.createConstraint(robotId, 5, childBodyUniqueId= punteroId, childLinkIndex= -1, jointType= p.JOINT_FIXED, jointAxis= joint_axis_gripper, parentFramePosition=gripper_parentFramePosition,
                                      childFramePosition=gripper_childFramePosition, childFrameOrientation=gripper_childFrameOrientation)

# --------- CARGAR PELOTA DE FUTBOL Y POSICIONAR EN EL CENTRO DE LA MESA ------------
# Considera lo siguiente:
# nombre urdf: soccerball.urdf // escala: globalScaling = 0.2 // no debe estar fijo en el espacio



#------ CREAR UNA SERIE DE POSES QUE REALICEN UNA CIRCUNFERENCIA EN EL PLANO DE LA MESA, SIN TOCAR EL CUBO, MINIMO 20 POSE ----------
# Pose array
path_KR6_2 = [[1, 0, 1, 0, 1.57, 0], [1.05, 0, 1, 0, 2, 0]]

#------ FIN TAREA -------

# tool coordinate position
n_tcf = 5

# Simulation cicles for instruction
rate = 2000

# Run the simulation
while True:
    p.stepSimulation()

    for i in range(len(path_KR6_2)):
        j = 1
        while (j/rate <= 1):
            # target position
            xyz = path_KR6_2[i][:3]
            # target orientation
            ori = p.getQuaternionFromEuler(path_KR6_2[i][3:6])
            
            # calculate joint target
            target = p.calculateInverseKinematics(robotId, endEffectorLinkIndex = n_tcf, targetPosition = xyz, targetOrientation = ori)
            p.setJointMotorControlArray(robotId, range(6), p.POSITION_CONTROL, targetPositions = target)
            p.stepSimulation()

            j+=1
    