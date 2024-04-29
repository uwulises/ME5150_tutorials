
"""
NOMBRE: _____________________________________
"""
import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to the PyBullet simulator
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # Set the search path to find the plane.urdf file

p.setGravity(0,0,-9.81) # Set the gravity to be in the negative z direction
p.setRealTimeSimulation(1) # Set the simulation to be real time

planeId = p.loadURDF("plane.urdf") # Load the plane into the simulation
robotic_arm = p.loadURDF("../modelos/manipuladores/scarapris/scarapris.urdf", basePosition = [0, 0, 0], useFixedBase = True)
vaso = p.loadURDF("../modelos/objetos/vaso.urdf", basePosition = [0.2, -0.15, 0], useFixedBase = True)

# tool coordinate position
n_tcf = 4

# TODO: Crear una serie de puntos XYZ que representen una trayectoria 
# para "untar" el pincel en el vaso y posteriormente "dibujar" tu inicial en el plano.
# Mínimo 50 puntos.
poses = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]

def move_robot():
    while True:
        for pose in poses:       
            # Calculate the inverse kinematics of the robot
            q = p.calculateInverseKinematics(robotic_arm, endEffectorLinkIndex = n_tcf, targetPosition = pose)
            # Set the joint angles of the robot
            p.setJointMotorControlArray(robotic_arm, range(4), p.POSITION_CONTROL, targetPositions = q)
            # Step the simulation
            p.stepSimulation()
            time.sleep(1/240)
            
# Call the move_robot function
move_robot()
