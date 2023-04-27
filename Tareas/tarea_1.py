"""---------    Tarea de Auxiliar 1, Parte 2                            -------
                Recomendación: revisar TareaForm2.py y scara_example.py -------"""

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

"""-------- Item 1: Cargar objeto que quieran tomar y ubicarlo sobre la mesa (basePosition)--------- """


"""-------- Item 2: Ubicar y orientar twezzer correctamente en la punta del efector del SCARA  ---------- """

joint_axis_gripper = [0, 0, 0]
gripper_parentFramePosition = [0, 0, 0]
gripper_childFramePosition = [0.1, 0.1, -0.9] #ubicar
gripper_childFrameOrientation = p.getQuaternionFromEuler([0,0,0])
joint_constraint = p.createConstraint(robotId, parentLinkIndex=2, childBodyUniqueId = tweezerId,
                            childLinkIndex= -1, jointType= p.JOINT_FIXED, jointAxis= joint_axis_gripper, 
                            parentFramePosition=gripper_parentFramePosition,
                            childFramePosition=gripper_childFramePosition, childFrameOrientation=gripper_childFrameOrientation)

""" ------  Item 3: Crear una rutina de pick and place, donde el robot tome el
            objeto que eligieron y lo deje en la caja sobre la mesa, mínimo 40 poses.

            Recomendación 1: Crear una función para rellenar puntos intermedios.
            Recomendación 2: Piense en cada acción que se debe realizar y programe cada
            una para crear cada una. Podría ser útil usar funciones.
            
            path_SCARA recibe vectores [x, y, z, c, eff]                                        --------- 
            Nota: el efector rota en su base para cambiar la orientación de los dedos del gripper""" 

path_SCARA = [[0.6, 0, 0.9, 0, 0]]

# Simulation cicles for instruction
rate = 2000
time.sleep(1)

while True:
# Run the simulation
    for i in range(len(path_SCARA)):
        j = 1
        while (j/rate <= 1):

            # target position
            xyz = path_SCARA[i][:3]
            c = path_SCARA[i][3]
            eff = path_SCARA[i][4]
            
            # calculate joint target
            target = p.calculateInverseKinematics(robotId, endEffectorLinkIndex = n_tcf, targetPosition = xyz)
            """-------- Item 4: Agregar línea que controla estado del efector, indicación: el efector tiene 3 joints ---------"""

            """ ------- FIN DE LA TAREA -------"""
            p.setJointMotorControlArray(robotId, range(n_tcf+1), p.POSITION_CONTROL, targetPositions = target)
            p.stepSimulation()
            j+=1
