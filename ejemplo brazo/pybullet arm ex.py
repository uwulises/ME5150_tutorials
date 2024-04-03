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

#Load the roboticArmURDF into the simulation
robotic_arm = p.loadURDF("roboticarm/roboticarm.urdf", basePosition = [0,0,0], useFixedBase=True)

def move_robot():
    num_joints = p.getNumJoints(robotic_arm)
    print("Number of joints in the robot: ", num_joints)
    # Define the joint indices for each link of the robot
    link1_index = 0
    link2_index = 1
    link3_index = 2


    # Create sliders for each joint
    link1_slider = p.addUserDebugParameter("Link 1", -np.pi, np.pi, 0)
    link2_slider = p.addUserDebugParameter("Link 2", -np.pi, np.pi, 0)
    link3_slider = p.addUserDebugParameter("Link 3", -np.pi, np.pi, 0)
    while True:
        # Get the slider values
        link1_angle = p.readUserDebugParameter(link1_slider)
        link2_angle = p.readUserDebugParameter(link2_slider)
        link3_angle = p.readUserDebugParameter(link3_slider)

        # Set the joint angles of the robot
        p.setJointMotorControl2(robotic_arm, link1_index, p.POSITION_CONTROL, targetPosition=link1_angle)
        p.setJointMotorControl2(robotic_arm, link2_index, p.POSITION_CONTROL, targetPosition=link2_angle)
        p.setJointMotorControl2(robotic_arm, link3_index, p.POSITION_CONTROL, targetPosition=link3_angle)

        # Step the simulation
        p.stepSimulation()
        time.sleep(1/240)  # Control the simulation speed

# Call the move_robot function
move_robot()