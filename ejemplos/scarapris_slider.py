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

# Load the roboticArmURDF into the simulation
robotic_arm = p.loadURDF("modelos/manipuladores/scarapris/scarapris.urdf", basePosition = [0, 0, 0], useFixedBase = True)
vaso = p.loadURDF("modelos/objetos/vaso.urdf", basePosition = [0.2, -0.15, 0], useFixedBase = True)

# Define the joint indices for each link of the robot
num_joints = p.getNumJoints(robotic_arm)
link_indices = [joint_index for joint_index in range(num_joints)]

# Create sliders for each joint
sliders = []
sliders.append(p.addUserDebugParameter("Link 1", -np.pi/2, np.pi/2, 0))
sliders.append(p.addUserDebugParameter("Link 2", -np.pi, np.pi, 0))
sliders.append(p.addUserDebugParameter("Link 3", 0, 0.1, 0))
sliders.append(p.addUserDebugParameter("Link 4", -0.06, 0, 0))

def move_robot():
    while True:
        # Get the slider values
        q = [p.readUserDebugParameter(slider) for slider in sliders]
        # Set the joint angles of the robot
        p.setJointMotorControlArray(robotic_arm, range(4), p.POSITION_CONTROL, targetPositions = q)
        # Step the simulation
        p.stepSimulation()
        time.sleep(1/240)
        print(p.getLinkState(robotic_arm, 4)[0])

# Call the move_robot function
move_robot()
