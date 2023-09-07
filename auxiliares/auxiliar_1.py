#Importacion de librerias
import pybullet as p
import pybullet_data
import time
import numpy as np

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity and time step
p.setGravity(0, 0, -9.81)
#p.setTimeStep(1 / 240) # Para el dron se recomienda trabajar con stepSimulation
p.setRealTimeSimulation(1) # 0 Desactiva la simulacion en tiempo real

# Cargar ambiente de simulacion
planeId = p.loadURDF("plane.urdf")

# Cargar robot y accesorios

#KUKA
base_kuka = p.loadURDF("../modelos/manipuladores/kuka/steel_base.urdf", basePosition = [0, 1, 0.0125], useFixedBase = True)
kuka_arm = p.loadURDF("../modelos/manipuladores/kuka/kr6_2.urdf", basePosition = [0, 1, 0.035], useFixedBase = True)

#SCARA
mesa_scara = p.loadURDF("../modelos/manipuladores/scara/base_scara.urdf",
                     basePosition=[0, -1, 0.69], useFixedBase=True)
initialori = p.getQuaternionFromEuler([0, 0, np.deg2rad(90)]) # initial orientation of the robot
scara_arm = p.loadURDF("../modelos/manipuladores/scara/scara.urdf",
                     basePosition=[0, -1, 0.69], baseOrientation=initialori,useFixedBase=True)

#Omnibase, se mueve libre por el plano, por lo que useFixedBase = False
omnibase = p.loadURDF("../modelos/base_movil/omnibase.urdf", basePosition = [0, 0, 0.0125], useFixedBase = False)

#Dron con 4 helices

#ciclo basico de la simulacion

while True:
    

    #movimientos basados en control de posicion, con angulos en radianes y velocidades en radianes/segundo

    #KUKA 6 joints, A1 A2 A3 A4 A5 A6
    target_kuka_pos=[0,0,0,0,0,0]
    p.setJointMotorControlArray(kuka_arm, range(6), p.POSITION_CONTROL, targetPositions = target_kuka_pos)

    #SCARA 3 joints, q0 q1 q2
    target_scara_pos=[0,0,0]
    p.setJointMotorControlArray(scara_arm, range(3), p.POSITION_CONTROL, targetPositions = target_scara_pos)

    #Omnibase 4 joints con control de velocidad, pero hay 4 joints extras fixed
    target_vel_omnibase=[0,0,0,0,0,0,0,0]
    p.setJointMotorControlArray(omnibase, range(8), p.VELOCITY_CONTROL, targetVelocities = target_vel_omnibase)
    

    #basico para que la simulacion con step funcione
    p.stepSimulation()