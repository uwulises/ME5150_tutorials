import pybullet as p
import pybullet_data
import time
import neat
import numpy as np
import pickle

# NEAT parameters
NUM_INPUTS = 12  # Replace with the number of inputs based on your quadruped's state
NUM_OUTPUTS = 12  # Each leg has joint control
QUADRUPED_URDF_PATH = "../anymal/urdf/anymal.urdf"
LEG_JOINT_NUMBERS = [1, 2, 3, 6, 7, 8, 11, 12, 13, 16, 17, 18]
p.connect(p.GUI)  # or p.DIRECT for headless mode
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")  # Load the ground plane
p.setGravity(0, 0, -9.81)

quadruped = p.loadURDF(QUADRUPED_URDF_PATH, basePosition=[0, 0, 0.8])
joint_ids = LEG_JOINT_NUMBERS


def step(joints_state=np.zeros(12)):
    i = 0
    for index in joint_ids:
        p.setJointMotorControl2(quadruped, jointIndex=index,
                                controlMode=p.POSITION_CONTROL, targetPosition=joints_state[i])
        i += 1
    p.stepSimulation()


def get_quadruped_state():
    # Retrieve and return the state of the quadruped (e.g., position, orientation, velocities)
    # Implement this based on your specific needs

    joints_state = []
    for index in joint_ids:
        joints_state.append(p.getJointState(quadruped, jointIndex=index)[0])

    return joints_state


# NEAT setup
config_path = "config.cfg"  # Replace with the path to your NEAT configuration file
config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                     neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)


# load the winner genome
with open('checkpoint/best_genome', 'rb') as f:
    init_genome = pickle.load(f)

best_net = neat.nn.FeedForwardNetwork.create(init_genome, config)

while True:
    # Replace with appropriate inputs based on the quadruped's state
    inputs = get_quadruped_state()
    outputs = best_net.activate(inputs)
    joint_angles = outputs[:NUM_OUTPUTS]
    step(joint_angles)
    quadruped_state = get_quadruped_state()
