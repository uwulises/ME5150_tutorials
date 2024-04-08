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
robotic_arm = p.loadURDF("modelos/manipuladores/twolinks/twolinks.urdf", basePosition=[0,0,0], useFixedBase=True)

# Define the joint indices for each link of the robot
num_joints = p.getNumJoints(robotic_arm)
link_indices = [joint_index for joint_index in range(num_joints)]

# Create sliders for each joint
sliders = [p.addUserDebugParameter(f"Link {i+1}", -np.pi, np.pi, 0) for i in range(num_joints)]

def move_robot():
    while True:
        # Get the slider values
        angles = [p.readUserDebugParameter(slider) for slider in sliders]

        # Set the joint angles of the robot
        for i, angle in enumerate(angles):
            p.setJointMotorControl2(robotic_arm, link_indices[i], p.POSITION_CONTROL, targetPosition=angle)

        # Step the simulation
        p.stepSimulation()
        time.sleep(1/240)  # Control the simulation speed

# Call the move_robot function
move_robot()
