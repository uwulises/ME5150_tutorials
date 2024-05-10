#Importacion de librerias
import pybullet as p
import pybullet_data
import time
import numpy as np

def translation_x(x):
    return np.array([[1, 0, 0, x], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def translation_y(y):
    return np.array([[1, 0, 0, 0], [0, 1, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])

def translation_z(z):
    return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, z], [0, 0, 0, 1]])

def rot_x(qx_deg):
    qx = np.deg2rad(qx_deg)
    return np.array([[1, 0, 0, 0], [0, np.cos(qx), -np.sin(qx), 0], [0, np.sin(qx), np.cos(qx), 0], [0, 0, 0, 1]])

def rot_y(qy_deg):
    qy = np.deg2rad(qy_deg)
    return np.array([[np.cos(qy), 0, np.sin(qy), 0], [0, 1, 0, 0], [-np.sin(qy), 0, np.cos(qy), 0], [0, 0, 0, 1]])

def rot_z(qz_deg):
    qz = np.deg2rad(qz_deg)
    return np.array([[np.cos(qz), -np.sin(qz), 0, 0], [np.sin(qz), np.cos(qz), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def calculate_joint_position(q1, q2, q3):
    """
    TODO: Realizar las operaciones de traslación y rotación que permiten 
    calcular la posición final del efector para el manipulador SCARA.
    Mostrar la posición de cada eje coordenado, para verificar las operaciones.
    """
    # Para multiplicar matrices A y B, usar np.linalg.multi_dot([A, B])
    # Todo está en metros o grados

    pass

# Prueba con diferentes valores de q1, q2, q3
q1 = 0 # [-90 90] grados
q2 = 0 # [-45 45] grados
q3 = 0 # [0 0.2] metros, 0 es el z más alto
calculate_joint_position(q1, q2, q3)


# DESCOMENTA ESTA SECCIÓN PARA COMPARAR TUS RESULTADOS, Ctrl + } o Edit>Toggle Line Comment

# # Initialize PyBullet
# physicsClient = p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())
# useFixedBase = True

# # Set gravity and time step
# p.setGravity(0, 0, -9.81)
# p.setTimeStep(1 / 240)

# # Load SCARA robot arm and table
# planeId = p.loadURDF("plane.urdf")
# initialori = p.getQuaternionFromEuler([0, 0, np.deg2rad(90)]) # initial orientation of the robot
# scara_arm = p.loadURDF("../modelos/manipuladores/scara/scara.urdf", basePosition = [0, 0, 0], baseOrientation = initialori, useFixedBase = useFixedBase)

# # #Inicializacion de la simulacion
# while True:
#     target_scara_pos = [q1, q2, -0.1 - q3]
#     p.setJointMotorControlArray(scara_arm, range(3), p.POSITION_CONTROL, targetPositions = target_scara_pos)
#     p.stepSimulation()

#     pos = p.getLinkState(scara_arm, 2)[0]
#     print(pos)
