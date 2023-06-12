import pybullet as p
import pybullet_data
import time
import neat
import random
import numpy as np
import gzip
import pickle
import csv
# Quadruped simulation parameters
QUADRUPED_URDF_PATH = "anymal/urdf/anymal.urdf"

LEG_JOINT_NUMBERS = [1, 2, 3, 6, 7, 8, 11, 12, 13, 16, 17, 18]
NUM_STEPS = 10
# NEAT parameters
NUM_INPUTS = 14  # Replace with the number of inputs based on your quadruped's state
NUM_OUTPUTS = 12  # Each leg has joint control
up_stairs_x = 4
# NEAT training loop
NUM_GENERATIONS = 200
last_distance = 0
global generation
# Quadruped environment class


class QuadrupedEnv:
    def __init__(self):
        p.connect(p.GUI)  # or p.DIRECT for headless mode
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")  # Load the ground plane
        p.loadURDF("objetos/stair.urdf", basePosition = [0, 0, 0], useFixedBase = True)
        p.setGravity(0, 0, -9.81)
        
        self.quadruped = p.loadURDF(QUADRUPED_URDF_PATH, basePosition=[0, 0, 0.8])
        self.joint_ids = LEG_JOINT_NUMBERS

        p.setTimeStep(1 / 100)  # Control the simulation speed

    def hard_reset(self):
        p.resetSimulation()
        p.loadURDF("plane.urdf")  # Load the ground plane
        p.loadURDF("objetos/stair.urdf", basePosition = [0, 0, 0], useFixedBase = True)
        self.quadruped = p.loadURDF(QUADRUPED_URDF_PATH, basePosition=[0, 0, 0.8])
        p.setGravity(0, 0, -9.81)

    def reset(self):
        # Reset the simulation
        p.setGravity(0, 0, -9.81)

        for index in self.joint_ids:
            p.resetJointState(self.quadruped, jointIndex=index, targetValue=0)

    def step(self, joints_state=np.zeros(12)):
        p.setJointMotorControlArray(self.quadruped, jointIndices=self.joint_ids, controlMode=p.POSITION_CONTROL, targetPositions=joints_state)
        p.stepSimulation()

    def get_quadruped_state(self):
        
        # Retrieve and return the state of the quadruped (e.g., position, orientation, velocities)
        # Implement this based on your specific needs

        joints_state = []
        for index in self.joint_ids:
            joints_state.append(p.getJointState(
                self.quadruped, jointIndex=index)[0])
        
        distance_x = p.getLinkState(self.quadruped, 0)[0][0]
        joints_state.append(distance_x)
        joints_state.append(up_stairs_x)
        return joints_state


    def get_fitness(self):
            # Compute and return the fitness score based on the performance of the quadruped
            # Implement this based on your specific objectives
            global last_distance
            fitness = 0.0
            # TODO

            distance_x = p.getLinkState(self.quadruped, 0)[0][0]
            
            distance_y = p.getLinkState(self.quadruped, 0)[0][1]
            ori = p.getLinkState(self.quadruped, 0)[1]
            angle_x = p.getEulerFromQuaternion(ori)[0]
            angle_y = p.getEulerFromQuaternion(ori)[1]
            angle_z = p.getEulerFromQuaternion(ori)[2]

            if (self.check_quadruped()):
                fitness=-1000
                last_distance=0.0

            elif (distance_x>last_distance):
                fitness = distance_x*np.cos(angle_z) - abs(distance_y)
                last_distance=distance_x
                

            elif (round(distance_x,1)==round(last_distance,1)):
                fitness = -1
                last_distance=distance_x

            elif(round(distance_x)==up_stairs_x):
                with open('Tareas/TA3/checkpoint/best_genome', 'wb') as f:
                    pickle.dump(best_genome, f)
                exit()
            return fitness

    def check_quadruped(self):
        ori = p.getLinkState(self.quadruped, 0)[1]
        angle_x = p.getEulerFromQuaternion(ori)[0]*180/np.pi
        pos_y = p.getLinkState(self.quadruped, 0)[0][1]
        if (angle_x < -60 or angle_x > 60):
            quadruped_env.hard_reset()
            return True
        if (abs(pos_y)>1.5):
            quadruped_env.hard_reset()
            return True
        return False

# NEAT fitness evaluation


def eval_genomes(genomes, config):
    for genome_id, genome in genomes:
        quadruped_env.reset()
        quadruped_state = quadruped_env.get_quadruped_state()
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        fitness = 0.0

        # Evaluate the genome over a specified number of simulation steps
        for _ in range(NUM_STEPS):
            # Replace with appropriate inputs based on the quadruped's state
            inputs = quadruped_state
            outputs = net.activate(inputs)
            joints_states = outputs[:NUM_OUTPUTS]
            quadruped_env.step(joints_states)
            quadruped_state = quadruped_env.get_quadruped_state()

            fitness += quadruped_env.get_fitness()

        # Assign fitness to the genome
        genome.fitness = fitness
        if (quadruped_env.check_quadruped()):
            genome.fitness = -100


# NEAT setup
config_path = "Tareas/TA3/config.cfg"  # Replace with the path to your NEAT configuration file
config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                     neat.DefaultSpeciesSet, neat.DefaultStagnation, config_path)
population = neat.Population(config)
population.add_reporter(neat.StdOutReporter(True))
stats = neat.StatisticsReporter()
population.add_reporter(stats)

# Quadruped environment initialization
quadruped_env = QuadrupedEnv()


for generation in range(NUM_GENERATIONS):

    best_genome = population.run(eval_genomes, 1)

    # Evaluate the best genome on a final test run
    quadruped_env.reset()
    quadruped_state = quadruped_env.get_quadruped_state()
    best_net = neat.nn.FeedForwardNetwork.create(best_genome, config)

    for _ in range(NUM_STEPS):
        # Replace with appropriate inputs based on the quadruped's state
        inputs = quadruped_state
        outputs = best_net.activate(inputs)
        joint_angles = outputs[:NUM_OUTPUTS]
        quadruped_env.step(joint_angles)
        quadruped_state = quadruped_env.get_quadruped_state()
    quadruped_env.check_quadruped()
    # Save the best genome.
    with open('Tareas/TA3/checkpoint/best_genome', 'wb') as f:
        pickle.dump(best_genome, f)


