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


def calculate_joint_position(q1, q2, q3, q4, q5, q6):
    """
    TODO: Realizar las operaciones de traslación y rotación que permiten 
    calcular la posición final del efector para el manipulador SCARA.
    Mostrar la posición de cada eje coordenado, para verificar las operaciones.
    """
    # Para multiplicar matrices A y B, usar np.linalg.multi_dot([A, B])
    # Todo está en metros o grados

    pass

# Prueba con diferentes valores de q1, q2, q3
q1 = 0
q2 = 0 
q3 = 0
q4 = 0
q5 = 0
q6 = 0
calculate_joint_position(q1, q2, q3, q4, q5, q6)


# DESCOMENTA ESTA SECCIÓN PARA COMPARAR TUS RESULTADOS, Ctrl + } o Edit>Toggle Line Comment

# Initialize PyBullet to move the kuka robot with the q_i angles given
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

# Load KR6 robot arm and table
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("modelos/manipuladores/kuka/kr6_2.urdf", basePosition=[0, 0, 0],useFixedBase=useFixedBase)
# Get the joints info
jointInfo0 = p.getJointInfo(robotId, 0)
jointInfo1 = p.getJointInfo(robotId, 1)
jointInfo2 = p.getJointInfo(robotId, 2)
jointInfo3 = p.getJointInfo(robotId, 3)
jointInfo4 = p.getJointInfo(robotId, 4)
jointInfo5 = p.getJointInfo(robotId, 5) 

# Define the joint indices for each link of the robot
num_joints = p.getNumJoints(robotId)
link_indices = [joint_index for joint_index in range(num_joints)]

while True:
    angles=[q1,q2,q3,q4,q5,q6]
    # Set the joint angles of the robot
    for i, angle in enumerate(np.deg2rad(angles)):
        p.setJointMotorControl2(robotId, link_indices[i], p.POSITION_CONTROL, targetPosition=angle)
    # Get the end effector position
    end_effector_pos, _ = p.getLinkState(robotId, link_indices[-1])[:2]
    print('End Effector Position:', end_effector_pos)
    # Step the simulation
    p.stepSimulation()
    time.sleep(1/240)  # Control the simulation speed


