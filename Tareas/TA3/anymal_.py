import pybullet as p
import pybullet_data
import time
import neat
import numpy as np
import pickle

# NEAT parameters
NUM_INPUTS = 14  # Replace with the number of inputs based on your quadruped's state
NUM_OUTPUTS = 12  # Each leg has joint control
QUADRUPED_URDF_PATH = "anymal/urdf/anymal.urdf"
LEG_JOINT_NUMBERS = [1, 2, 3, 6, 7, 8, 11, 12, 13, 16, 17, 18]
NUM_STEPS = 5000
up_stairs_x=4
p.connect(p.GUI)  # or p.DIRECT for headless mode
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")  # Load the ground plane
p.loadURDF("objetos/stair.urdf", basePosition = [0, 0, 0], useFixedBase = True)
p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 2000)
quadruped = p.loadURDF(QUADRUPED_URDF_PATH, basePosition=[0, 0, 0.8])
joint_ids = LEG_JOINT_NUMBERS


def step(joints_state=np.zeros(12)):

    p.setJointMotorControlArray(quadruped, jointIndices=joint_ids, controlMode=p.POSITION_CONTROL, targetPositions=joints_state)
    p.stepSimulation()


def get_quadruped_state():
    # Retrieve and return the state of the quadruped (e.g., position, orientation, velocities)
    # Implement this based on your specific needs

    joints_state = []
    for index in joint_ids:
        joints_state.append(p.getJointState(quadruped, jointIndex=index)[0])
    distance_x = p.getLinkState(quadruped, 0)[0][0]
    joints_state.append(distance_x)
    joints_state.append(up_stairs_x)
    return joints_state


# load the winner genome
with open('Tareas/TA3/checkpoint/best_genome', 'rb') as f:
    init_genome = pickle.load(f)

# NEAT setup
config_path = "Tareas/TA3/config.cfg"  # Replace with the path to your NEAT configuration file
config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                     neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)

def eval_genomes(genomes, config):
    net = neat.nn.FeedForwardNetwork.create(genomes, config)

    # Evaluate the genome over a specified number of simulation steps
    for _ in range(NUM_STEPS):
        # Replace with appropriate inputs based on the quadruped's state
        inputs = get_quadruped_state() 
        outputs = net.activate(inputs)
        joints_states = outputs[:NUM_OUTPUTS]
        step(joints_states)

while True:
    eval_genomes(init_genome,config)
